// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package first.robot.subsystems;

import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Kilograms;
import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.Seconds;
import static org.wpilib.units.Units.Volts;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.util.CANPorts;
import first.robot.Constants;
import java.util.Optional;
import org.wpilib.command2.Command;
import org.wpilib.command2.SubsystemBase;
import org.wpilib.math.system.DCMotor;
import org.wpilib.units.measure.Angle;
import yams.commands2.config.SmartMotorControllerConfig;
import yams.commands2.mechanisms.Arm;
import yams.core.gearing.MechanismGearing;
import yams.core.math.ADRCConfig;
import yams.core.math.ADRCConfig.SystemOrder;
import yams.core.math.ADRCController;
import yams.core.mechanisms.config.ArmConfig;
import yams.core.motorcontrollers.SmartMotorController;
import yams.core.motorcontrollers.enums.ControlMode;
import yams.core.motorcontrollers.enums.MotorMode;
import yams.core.motorcontrollers.local.SparkWrapper;
import yams.core.telemetry.enums.TelemetryVerbosity;

/**
 * An arm run by an active disturbance rejection controller (ADRC) instead of a PID controller: a NEO on a 60:1
 * reduction, moving a 3 kg, 0.5 m arm. The ADRC was written by FRC Team 8331.
 *
 * <p>The arm's position is a second order plant: a voltage accelerates it, and the acceleration integrates twice into
 * its angle. The ADRC's observer estimates gravity, friction and anything that bumps the arm as one disturbance and
 * cancels it, so the arm needs no kG.
 *
 * <p>With HIGH telemetry, the ADRC's settings are tunable live in place of kP, kI and kD, under
 * {@code /Tuning/ADRCArm/ArmMotor/closedloop/adrc}.
 */
public class ADRCArmSubsystem extends SubsystemBase {
  private static final MechanismGearing GEARING = new MechanismGearing(60);
  private static final DCMotor MOTOR = DCMotor.getNEO(1);

  private final SmartMotorController motor = new SparkWrapper(
      new SparkMax(CANPorts.fromBusId(1), Constants.ARM_MOTOR_ID, MotorType.kBrushless),
      MOTOR,
      new SmartMotorControllerConfig(this)
          .withControlMode(ControlMode.CLOSED_LOOP)
          // The ADRC takes the place of kP, kI, kD and the feedforward.
          .withADRCController(new ADRCConfig()
              // A position loop on an arm is second order.
              .withSystemOrder(SystemOrder.ARM)
              // How quickly the arm should reach its setpoint.
              .withSettlingTime(Seconds.of(0.6))
)
          // The output limit and ramp of every closed loop, the ADRC's too.
          .withClosedLoopControllerMaximumVoltage(Volts.of(12))
          .withGearing(GEARING)
          // b0, how much one volt accelerates the arm, is derived from the motor, the gearing and this inertia.
          .withMomentOfInertia(Meters.of(0.5), Kilograms.of(3))
          .withSoftLimits(Degrees.of(-20), Degrees.of(100))
          .withStartingPosition(Degrees.of(-20))
          .withZeroPower(MotorMode.BRAKE)
          .withStatorCurrentLimit(Amps.of(40))
          .withTelemetry("ArmMotor", TelemetryVerbosity.HIGH));

  /** Stowed on its lower hard stop, which is also its lower soft limit. */
  private final Arm arm = new Arm(new ArmConfig()
      .withLength(Meters.of(0.5))
      .withHardLimits(Degrees.of(-20), Degrees.of(110))
      .withTelemetry("ADRCArm", TelemetryVerbosity.HIGH),
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
   * The arm's ADRC, built from its {@link ADRCConfig} by the motor controller.
   *
   * @return {@link ADRCController}, present once the motor controller is configured.
   */
  public Optional<ADRCController> getADRC() {
    return motor.getADRCController();
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
