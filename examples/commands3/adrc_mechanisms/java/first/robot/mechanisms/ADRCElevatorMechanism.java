// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package first.robot.mechanisms;

import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.Inches;
import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.Pounds;
import static org.wpilib.units.Units.Seconds;
import static org.wpilib.units.Units.Volts;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.util.CANPorts;
import first.robot.Constants;
import java.util.Optional;
import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;
import org.wpilib.math.system.DCMotor;
import org.wpilib.units.measure.Distance;
import yams.commands3.config.SmartMotorControllerConfig;
import yams.commands3.mechanisms.Elevator;
import yams.core.gearing.GearBox;
import yams.core.gearing.MechanismGearing;
import yams.core.math.ADRCConfig;
import yams.core.math.ADRCConfig.SystemOrder;
import yams.core.math.ADRCController;
import yams.core.mechanisms.config.ElevatorConfig;
import yams.core.motorcontrollers.SmartMotorController;
import yams.core.motorcontrollers.enums.ControlMode;
import yams.core.motorcontrollers.enums.MotorMode;
import yams.core.motorcontrollers.local.SparkWrapper;
import yams.core.telemetry.enums.TelemetryVerbosity;

/**
 * An elevator run by an active disturbance rejection controller (ADRC) instead of a PID controller: a NEO on a 12:1
 * reduction, lifting a 16 lb carriage on a 22 tooth, 1/4 in pitch sprocket. The ADRC was written by FRC Team 8331.
 *
 * <p>An elevator's speed is set by its motor's back-EMF more than by its inertia: each volt moves it at about the same
 * speed. So its position is treated as a first order plant, with that speed per volt as b0: SystemOrder.ELEVATOR
 * derives it from the motor, gearing and sprocket, in meters per second per volt, for the linear closed loop. The ADRC's
 * observer estimates gravity as part of the disturbance, so the elevator needs no kG.
 *
 * <p>With HIGH telemetry, the ADRC's settings are tunable live in place of kP, kI and kD, under
 * {@code /Tuning/ADRCElevator/ElevatorMotor/closedloop/adrc}.
 */
public class ADRCElevatorMechanism implements Mechanism {
  /** 3:1 then 4:1, 12 motor rotations per sprocket rotation. */
  private static final MechanismGearing GEARING = new MechanismGearing(GearBox.fromReductionStages(3, 4));
  private static final DCMotor MOTOR = DCMotor.getNEO(1);
  /** A 22 tooth sprocket on 1/4 in pitch chain moves the carriage 5.5 in per rotation. */
  private static final Distance CIRCUMFERENCE = Inches.of(0.25).times(22);

  private final SmartMotorController motor = new SparkWrapper(
      new SparkMax(CANPorts.fromBusId(1), Constants.ELEVATOR_MOTOR_ID, MotorType.kBrushless),
      MOTOR,
      new SmartMotorControllerConfig(this)
          .withControlMode(ControlMode.CLOSED_LOOP)
          // The ADRC takes the place of kP, kI, kD and the feedforward.
          .withADRCController(new ADRCConfig()
              // First order on position, with b0 its speed per volt from the motor, gearing and sprocket.
              .withSystemOrder(SystemOrder.ELEVATOR)
              .withSettlingTime(Seconds.of(0.3)))
          // The output limit and ramp of every closed loop, the ADRC's too.
          .withClosedLoopControllerMaximumVoltage(Volts.of(12))
          .withGearing(GEARING)
          .withMechanismCircumference(CIRCUMFERENCE)
          // The closed loop, and the ADRC, work in meters, as SystemOrder.ELEVATOR needs.
          .withLinearClosedLoopController(true)
          .withSoftLimits(Meters.of(0), Meters.of(1.5))
          .withStartingPosition(Meters.of(0))
          .withZeroPower(MotorMode.BRAKE)
          .withStatorCurrentLimit(Amps.of(40))
          .withTelemetry("ElevatorMotor", TelemetryVerbosity.HIGH));

  /** Starts on its lower hard stop. */
  private final Elevator elevator = new Elevator(new ElevatorConfig()
      .withCarriageWeight(Pounds.of(16))
      .withHardLimits(Meters.of(0), Meters.of(1.6))
      .withTelemetry("ADRCElevator", TelemetryVerbosity.HIGH),
      motor);

  /**
   * Move the elevator to a height and hold it there.
   *
   * @param height Height to go to.
   * @return {@link Command}
   */
  public Command setHeight(Distance height) {
    return elevator.setHeight(height);
  }

  /**
   * The elevator's height.
   *
   * @return Height of the carriage.
   */
  public Distance getHeight() {
    return elevator.getHeight();
  }

  /**
   * The elevator's ADRC, built from its {@link ADRCConfig} by the motor controller.
   *
   * @return {@link ADRCController}, present once the motor controller is configured.
   */
  public Optional<ADRCController> getADRC() {
    return motor.getADRCController();
  }

  /** Publishes YAMS telemetry. Called from {@link first.robot.Robot#robotPeriodic()}. */
  public void periodic() {
    elevator.updateTelemetry();
  }

  /** Steps the YAMS simulation. Run in its own Notifier, started in {@link first.robot.Robot#simulationInit()}. */
  public void simulationPeriodic() {
    elevator.simIterate();
  }
}
