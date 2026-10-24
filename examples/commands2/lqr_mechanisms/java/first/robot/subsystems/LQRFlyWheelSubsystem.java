// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package first.robot.subsystems;

import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.Inches;
import static org.wpilib.units.Units.KilogramSquareMeters;
import static org.wpilib.units.Units.Pounds;
import static org.wpilib.units.Units.RPM;
import static org.wpilib.units.Units.Volts;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.util.CANPorts;
import first.robot.Constants;
import org.wpilib.command2.Command;
import org.wpilib.command2.SubsystemBase;
import org.wpilib.math.system.DCMotor;
import org.wpilib.units.measure.AngularVelocity;
import yams.commands2.config.SmartMotorControllerConfig;
import yams.commands2.mechanisms.FlyWheel;
import yams.core.gearing.MechanismGearing;
import yams.core.math.LQRConfig;
import yams.core.math.LQRController;
import yams.core.mechanisms.config.FlyWheelConfig;
import yams.core.motorcontrollers.SmartMotorController;
import yams.core.motorcontrollers.enums.ControlMode;
import yams.core.motorcontrollers.enums.MotorMode;
import yams.core.motorcontrollers.local.SparkWrapper;
import yams.core.telemetry.enums.TelemetryVerbosity;

/**
 * A flywheel run by an LQR instead of a PID controller: a NEO turning a 4 in, 1 lb wheel directly.
 *
 * <p>With HIGH telemetry, the LQR's values are tunable live in place of kP, kI and kD, under
 * {@code /Tuning/LQRFlyWheel/FlyWheelMotor/closedloop/lqr}: the velocity tolerance (qelms), the voltage effort
 * (relms), how much the model and the encoder are trusted, and the maximum voltage. Each change rebuilds the LQR
 * while it runs.
 */
public class LQRFlyWheelSubsystem extends SubsystemBase {
  private static final MechanismGearing GEARING = new MechanismGearing(1);
  private static final DCMotor MOTOR = DCMotor.getNEO(1);

  /** The flywheel's LQR: a 1 lb solid wheel of 2 in radius, ½·m·r². */
  private final LQRController lqr = new LQRController(new LQRConfig(MOTOR, GEARING, KilogramSquareMeters.of(0.000585))
      .withFlyWheel(RPM.of(100), RPM.of(30), RPM.of(1))
      .withControlEffort(Volts.of(12))
      .withMaxVoltage(Volts.of(12)));

  private final SmartMotorController motor = new SparkWrapper(
      new SparkMax(CANPorts.fromBusId(1), Constants.FLYWHEEL_MOTOR_ID, MotorType.kBrushless),
      MOTOR,
      new SmartMotorControllerConfig(this)
          .withControlMode(ControlMode.CLOSED_LOOP)
          .withClosedLoopController(lqr)
          .withGearing(GEARING)
          .withZeroPower(MotorMode.COAST)
          .withStatorCurrentLimit(Amps.of(40))
          .withMomentOfInertia(Inches.of(4), Pounds.of(1))
          .withTelemetry("FlyWheelMotor", TelemetryVerbosity.HIGH));

  private final FlyWheel flyWheel = new FlyWheel(new FlyWheelConfig()
      .withDiameter(Inches.of(4))
      .withTelemetry("LQRFlyWheel", TelemetryVerbosity.HIGH),
      motor);

  /** Coasts whenever nothing else is using the flywheel. */
  public LQRFlyWheelSubsystem() {
    setDefaultCommand(run(() -> flyWheel.setDutyCycleSetpoint(0)).withName("LQRFlyWheel Coast"));
  }

  /**
   * Spin the flywheel at a speed until interrupted.
   *
   * @param speed Speed to hold.
   * @return {@link Command}
   */
  public Command spinUp(AngularVelocity speed) {
    // The Supplier version starts the closed loop controller, which runs the LQR on the SystemCore. The coasting
    // default command stops it.
    return flyWheel.run(() -> speed);
  }

  /**
   * The flywheel's speed.
   *
   * @return Speed of the flywheel.
   */
  public AngularVelocity getSpeed() {
    return flyWheel.getSpeed();
  }

  /**
   * The flywheel's LQR, whose values live tuning changes.
   *
   * @return {@link LQRController}
   */
  public LQRController getLQR() {
    return lqr;
  }

  @Override
  public void periodic() {
    flyWheel.updateTelemetry();
  }

  @Override
  public void simulationPeriodic() {
    flyWheel.simIterate();
  }
}
