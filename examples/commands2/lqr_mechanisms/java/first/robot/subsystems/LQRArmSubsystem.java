// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package first.robot.subsystems;

import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.DegreesPerSecond;
import static org.wpilib.units.Units.DegreesPerSecondPerSecond;
import static org.wpilib.units.Units.KilogramSquareMeters;
import static org.wpilib.units.Units.Kilograms;
import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.Volts;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.util.CANPorts;
import first.robot.Constants;
import org.wpilib.command2.Command;
import org.wpilib.command2.SubsystemBase;
import org.wpilib.math.controller.ArmFeedforward;
import org.wpilib.math.system.DCMotor;
import org.wpilib.units.measure.Angle;
import yams.commands2.config.SmartMotorControllerConfig;
import yams.commands2.mechanisms.Arm;
import yams.core.gearing.MechanismGearing;
import yams.core.math.LQRConfig;
import yams.core.math.LQRController;
import yams.core.mechanisms.config.ArmConfig;
import yams.core.motorcontrollers.SmartMotorController;
import yams.core.motorcontrollers.enums.ControlMode;
import yams.core.motorcontrollers.enums.MotorMode;
import yams.core.motorcontrollers.local.SparkWrapper;
import yams.core.telemetry.enums.TelemetryVerbosity;

/**
 * An arm run by an LQR instead of a PID controller: a NEO on a 60:1 reduction, moving a 3 kg, 0.5 m arm.
 *
 * <p>With HIGH telemetry, the LQR's values are tunable live in place of kP, kI and kD, under
 * {@code /Tuning/LQRArm/ArmMotor/closedloop/lqr}: the position and velocity tolerances (qelms), the voltage effort
 * (relms), how much the model and the encoder are trusted, and the maximum voltage. Each change rebuilds the LQR
 * while it runs.
 */
public class LQRArmSubsystem extends SubsystemBase {
  private static final MechanismGearing GEARING = new MechanismGearing(60);
  private static final DCMotor MOTOR = DCMotor.getNEO(1);

  /** The arm's LQR. Its tolerances start loose, so there is something to tune. */
  private final LQRController lqr = new LQRController(new LQRConfig(MOTOR, GEARING, KilogramSquareMeters.of(0.25))
      .withArm(Degrees.of(5), DegreesPerSecond.of(60), Degrees.of(3), DegreesPerSecond.of(30), Degrees.of(0.5))
      .withControlEffort(Volts.of(12))
      .withMaxVoltage(Volts.of(12)));

  private final SmartMotorController motor = new SparkWrapper(
      new SparkMax(CANPorts.fromBusId(1), Constants.ARM_MOTOR_ID, MotorType.kBrushless),
      MOTOR,
      new SmartMotorControllerConfig(this)
          .withControlMode(ControlMode.CLOSED_LOOP)
          .withClosedLoopController(lqr)
          .withTrapezoidalProfile(DegreesPerSecond.of(90), DegreesPerSecondPerSecond.of(180))
          // The LQR has no gravity in its model, so the feedforward holds the arm up.
          .withFeedforward(new ArmFeedforward(0, 0.5, 0, 0))
          .withGearing(GEARING)
          .withSoftLimits(Degrees.of(-20), Degrees.of(100))
          .withStartingPosition(Degrees.of(-20))
          .withZeroPower(MotorMode.BRAKE)
          .withStatorCurrentLimit(Amps.of(40))
          .withMomentOfInertia(Meters.of(0.5), Kilograms.of(3))
          .withTelemetry("ArmMotor", TelemetryVerbosity.HIGH));

  /** Stowed on its lower hard stop, which is also its lower soft limit. */
  private final Arm arm = new Arm(new ArmConfig()
      .withLength(Meters.of(0.5))
      .withHardLimits(Degrees.of(-20), Degrees.of(110))
      .withTelemetry("LQRArm", TelemetryVerbosity.HIGH),
      motor);

  /**
   * Move the arm to an angle and hold it there.
   *
   * @param angle Angle to go to.
   * @return {@link Command}
   */
  public Command setAngle(Angle angle) {
    return arm.setAngle(angle);
  }

  /**
   * The arm's angle.
   *
   * @return Angle of the arm.
   */
  public Angle getAngle() {
    return arm.getAngle();
  }

  /**
   * The arm's LQR, whose values live tuning changes.
   *
   * @return {@link LQRController}
   */
  public LQRController getLQR() {
    return lqr;
  }

  @Override
  public void periodic() {
    arm.updateTelemetry();
  }

  @Override
  public void simulationPeriodic() {
    arm.simIterate();
  }
}
