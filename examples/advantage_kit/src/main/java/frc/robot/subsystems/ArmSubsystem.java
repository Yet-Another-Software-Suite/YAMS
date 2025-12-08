
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
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

public class ArmSubsystem extends SubsystemBase {

  public class ArmConstants {

    public static final Angle SOME_ANGLE = Degrees.of(20);
    public static final Angle DOWN_ANGLE = Degrees.of(-35);
    public static final Angle L1_ANGLE = Degrees.of(65);
    public static final Angle HANDOFF_ANGLE = Degrees.of(135);
    public static final double KP = 18;
    public static final double KI = 0;
    public static final double KD = 0.2;
    public static final double KS = -0.1;
    public static final double KG = 1.2;
    public static final double KV = 0;
    public static final double KA = 0;
    public static final double VELOCITY = 458;
    public static final double ACCELERATION = 688;
    public static final int MOTOR_ID = 40;
    public static final double STATOR_CURRENT_LIMIT = 120;
    public static final double MOI = 0.1055457256;

  }

  /**
   * AdvantageKit identifies inputs via the "Replay Bubble". Everything going to
   * the SMC is an Output. Everything coming
   * from the SMC is an Input.
   */
  @AutoLog
  public static class ArmInputs {

    public Angle pivotPosition = Degrees.of(0);
    public AngularVelocity pivotVelocity = DegreesPerSecond.of(0);
    public Angle pivotDesiredPosition = Degrees.of(0);
    public Voltage pivotAppliedVolts = Volts.of(0);
    public Current pivotCurrent = Amps.of(0);

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
          ArmConstants.KD,
          DegreesPerSecond.of(ArmConstants.VELOCITY),
          DegreesPerSecondPerSecond.of(ArmConstants.ACCELERATION))
      .withSimClosedLoopController(ArmConstants.KP,
          ArmConstants.KI,
          ArmConstants.KD,
          DegreesPerSecond.of(ArmConstants.VELOCITY),
          DegreesPerSecondPerSecond.of(ArmConstants.ACCELERATION))
      .withFeedforward(new ArmFeedforward(ArmConstants.KS,
          ArmConstants.KG,
          ArmConstants.KV,
          ArmConstants.KA))
      .withSimFeedforward(new ArmFeedforward(ArmConstants.KS,
          ArmConstants.KG,
          ArmConstants.KV,
          ArmConstants.KA))
      .withTelemetry("", TelemetryVerbosity.HIGH)
      .withGearing(new MechanismGearing(GearBox.fromReductionStages(12.5, 1)))
      .withMotorInverted(false)
      .withIdleMode(MotorMode.BRAKE)

      .withStatorCurrentLimit(Amps.of(ArmConstants.STATOR_CURRENT_LIMIT));

  private SmartMotorController armSMC = new TalonFXWrapper(armMotor, DCMotor.getFalcon500(1), smcConfig);

  private ArmConfig armCfg = new ArmConfig(armSMC)
      .withHardLimit(Degrees.of(-25), Degrees.of(141))
      .withStartingPosition(Degrees.of(141))
      .withLength(Feet.of((14.0 / 12)))
      .withMOI(ArmConstants.MOI)
      .withTelemetry("Arm", TelemetryVerbosity.HIGH);

  // Arm Mechanism
  private Arm arm = new Arm(armCfg);

  /**
   * Updates AdvantageKit inputs from the {@link Arm} to be used in the rest of
   * the program.
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

  /**
   * Run sysId on the {@link Arm}
   */
  public Command sysId() {
    return arm.sysId(Volts.of(7), Volts.of(2).per(Second), Seconds.of(4));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    updateInputs();
    Logger.processInputs("Arm", armInputs);
    arm.updateTelemetry();
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    arm.simIterate();
  }

  @AutoLogOutput
  public Angle getAngleSetpoint() {
    return armSMC.getMechanismPositionSetpoint().orElse(null);
  }

  public Angle getAngle() {
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