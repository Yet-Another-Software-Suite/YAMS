// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.ArmConfig;
import yams.mechanisms.positional.Arm;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.remote.TalonFXWrapper;

/**
 * Single-pivot arm driven by a TalonFX with trapezoidal motion profiling.
 * Uses AdvantageKit to log hardware inputs so matches can be replayed offline
 * without re-running the physical arm. The TalonFX runs the PID loop on-device,
 * making network latency irrelevant to closed-loop response.
 */
public class ArmSubsystem extends SubsystemBase {
  /**
   * Named positions and tuned constants collected in one place so they can be
   * adjusted without hunting through the subsystem body.
   */
  public class ArmConstants {
    // Positions measured with a protractor, 0 deg = arm parallel to the floor.
    public static final Angle SOME_ANGLE    = Degrees.of(20);
    public static final Angle DOWN_ANGLE    = Degrees.of(-35);   // Near-floor intake
    public static final Angle L1_ANGLE      = Degrees.of(65);    // Level-1 scoring
    public static final Angle HANDOFF_ANGLE = Degrees.of(135);   // Shooter handoff

    // PID gains run on-device inside the TalonFX's 1 kHz control loop.
    // kP=18 is aggressive for a 12.5:1 arm -- lower if you see oscillation at
    // large setpoint errors. kD=0.2 damps the final approach.
    public static final double KP = 18;
    public static final double KI = 0;
    public static final double KD = 0.2;

    // ArmFeedforward gains.
    // kS=-0.1 compensates for static friction in the direction of gravity.
    // kG=1.2 is the gravity term (volts to hold arm horizontal); tune this first.
    // kV and kA are left at 0 until SysId characterization is run.
    public static final double KS = -0.1;
    public static final double KG = 1.2;
    public static final double KV = 0;
    public static final double KA = 0;

    // Trapezoidal profile constraints -- 458 deg/s cruise and 688 deg/s^2 accel
    // allow a full 170 deg travel in roughly 0.6 seconds.
    public static final double VELOCITY     = 458;
    public static final double ACCELERATION = 688;

    public static final int    MOTOR_ID             = 40;
    // 120 A stator limit: TalonFX can sustain this briefly; helps during impact
    // without triggering the breaker on short bursts.
    public static final double STATOR_CURRENT_LIMIT = 120;
    // MOI measured from CAD for the arm assembly (kg*m^2).
    public static final double MOI = 0.1055457256;
  }

  /*
   * ArmInputs is the replay boundary for this subsystem. @AutoLog generates
   * ArmInputsAutoLogged at compile time, adding the serialization glue that
   * Logger.processInputs() needs to stamp these fields with a common timestamp.
   *
   * During replay the logger overwrites these fields with the original recorded
   * values before periodic() continues, so all downstream logic sees the exact
   * same numbers that ran on the real robot.
   */
  @AutoLog
  public static class ArmInputs {
    // Actual pivot angle from the TalonFX internal encoder.
    public Angle           pivotPosition        = Degrees.of(0);
    public AngularVelocity pivotVelocity        = DegreesPerSecond.of(0);
    // Active closed-loop setpoint stored here so replay knows what was demanded.
    public Angle           pivotDesiredPosition = Degrees.of(0);
    // Phase voltage reported by the TalonFX.
    public Voltage         pivotAppliedVolts    = Volts.of(0);
    public Current         pivotCurrent         = Amps.of(0);
  }

  private final ArmInputsAutoLogged armInputs = new ArmInputsAutoLogged();

  private final TalonFX armMotor = new TalonFX(ArmConstants.MOTOR_ID);

  ///
  /// YAMS Configurations
  ///
  private SmartMotorControllerConfig smcConfig = new SmartMotorControllerConfig(this)
      .withControlMode(ControlMode.CLOSED_LOOP)
      .withClosedLoopController(ArmConstants.KP,
          ArmConstants.KI,
          ArmConstants.KD)
      // 458 deg/s cruise keeps the arm from overshooting during fast repositions.
      .withTrapezoidalProfile(DegreesPerSecond.of(ArmConstants.VELOCITY),
          DegreesPerSecondPerSecond.of(ArmConstants.ACCELERATION))
      // Sim uses the same gains so logged simulation data reflects real behavior.
      .withSimClosedLoopController(ArmConstants.KP,
          ArmConstants.KI,
          ArmConstants.KD)
      .withFeedforward(new ArmFeedforward(ArmConstants.KS,
          ArmConstants.KG,
          ArmConstants.KV,
          ArmConstants.KA))
      .withSimFeedforward(new ArmFeedforward(ArmConstants.KS,
          ArmConstants.KG,
          ArmConstants.KV,
          ArmConstants.KA))
      .withTelemetry("", TelemetryVerbosity.HIGH)
      // 12.5:1 single-stage reduction; enough holding torque to resist gravity
      // at the handoff angle without a brake.
      .withGearing(new MechanismGearing(GearBox.fromReductionStages(12.5, 1)))
      .withMotorInverted(false)
      // BRAKE keeps the arm from drifting when disabled.
      .withIdleMode(MotorMode.BRAKE)
      .withStatorCurrentLimit(Amps.of(ArmConstants.STATOR_CURRENT_LIMIT))
      // 141 deg matches the hard upper limit -- arm boots at its stored-transport
      // position and the encoder is trusted from power-on.
      .withStartingPosition(Degrees.of(141))
      .withMomentOfInertia(KilogramSquareMeters.of(ArmConstants.MOI));

  private SmartMotorController armSMC = new TalonFXWrapper(armMotor, DCMotor.getFalcon500(1), smcConfig);

  private ArmConfig armCfg = new ArmConfig()
      // Hard limits are the physical stops; soft limits (inside SmartMotorControllerConfig)
      // stop the profile before the arm ever reaches the hard stop.
      .withHardLimits(Degrees.of(-25), Degrees.of(141))
      // Arm is 14 inches long (14/12 ft); used by the sim to compute MOI if not
      // overridden by withMomentOfInertia() above.
      .withLength(Feet.of((14.0 / 12)))
      .withTelemetry("Arm", TelemetryVerbosity.HIGH);

  // Arm Mechanism
  private Arm arm = new Arm(armCfg, armSMC);

  /**
   * Populate ArmInputs from the SMC. Must run before Logger.processInputs() so
   * that all fields share the same timestamp in the log. During replay, the
   * logger overwrites these fields; writing them here is a no-op in that case.
   */
  public void updateInputs() {
    armInputs.pivotPosition = arm.getAngle();
    armInputs.pivotVelocity = armSMC.getMechanismVelocity();
    armInputs.pivotAppliedVolts = armSMC.getVoltage();
    armInputs.pivotCurrent = armSMC.getStatorCurrent();
  }

  /**
   * Set the angle of the arm.
   *
   * @param angle Angle to go to.
   */
  public Command setAngle(Angle angle) {
    return arm.setAngle(angle);
  }

  /**
   * Move the arm up and down.
   *
   * @param dutycycle [-1, 1] speed to set the arm too.
   */
  // public Command set(double dutycycle) { return arm.set(dutycycle);}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    updateInputs();
    // processInputs stamps all ArmInputs fields with the current log timestamp.
    // In replay mode it also overwrites them with recorded values -- this is the
    // moment the "replay bubble" closes around this subsystem.
    Logger.processInputs("Arm", armInputs);
    arm.updateTelemetry();
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    arm.simIterate();
  }

  // @AutoLogOutput publishes this as a computed output -- it is NOT replayed.
  // Replay will re-execute this getter and write the freshly computed setpoint
  // to the new log so you can compare commanded vs. achieved angle post-match.
  @AutoLogOutput
  public Angle getAngleSetpoint() {
    return armSMC.getMechanismPositionSetpoint().orElse(null);
  }

  public Angle getAngle() {
    // Read from armInputs so this returns the replayed value during log replay,
    // not a live hardware read.
    return armInputs.pivotPosition;
  }

  public AngularVelocity getVelocity() {
    return armInputs.pivotVelocity;
  }

  public Angle getSetpointAngle() {
    return armInputs.pivotDesiredPosition;
  }

  public Voltage getVoltage() {
    return armInputs.pivotAppliedVolts;
  }

  public Current getCurrent() {
    return armInputs.pivotCurrent;
  }
}
