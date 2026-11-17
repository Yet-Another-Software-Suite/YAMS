// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package first.robot.mechanisms;

import static first.robot.Constants.ArmConstants.*;
import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Seconds;

import com.revrobotics.spark.A301;
import com.revrobotics.util.CANPorts;
import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;
import org.wpilib.math.controller.ArmFeedforward;
import org.wpilib.units.measure.Angle;
import yams.commands3.config.SmartMotorControllerConfig;
import yams.commands3.mechanisms.Arm;
import yams.core.gearing.GearBox;
import yams.core.gearing.MechanismGearing;
import yams.core.mechanisms.config.ArmConfig;
import yams.core.motorcontrollers.SmartMotorController;
import yams.core.motorcontrollers.enums.ControlMode;
import yams.core.motorcontrollers.enums.MotorMode;
import yams.core.motorcontrollers.local.A301Wrapper;
import yams.core.telemetry.enums.TelemetryVerbosity;

/**
 * An arm driven by a FIRST A301. The A301 has no closed loop of its own, so YAMS runs the arm's
 * position loop and motion profile on the SystemCore and sends the A301 a voltage.
 */
public class ArmMechanism implements Mechanism {
  private final SmartMotorController motor = new A301Wrapper(
      new A301(CANPorts.fromBusId(CAN_BUS), MOTOR_ID),
      new SmartMotorControllerConfig(this)
          .withControlMode(ControlMode.CLOSED_LOOP)
          // Gains are per arm rotation: 30 V per rotation of error.
          .withClosedLoopController(30, 0, 0)
          .withTrapezoidalProfile(MAX_VELOCITY, MAX_ACCELERATION)
          // kG holds the arm level against gravity, with its mass at its middle; kV is 12 V over the
          // arm's free speed.
          .withFeedforward(new ArmFeedforward(0, 1.6, 10.2, 0))
          // From the A301's rotor to the arm.
          .withGearing(new MechanismGearing(GearBox.fromReductionStages(GEARBOX_REDUCTION, CHAIN_REDUCTION)))
          .withZeroPower(MotorMode.BRAKE)
          .withStartingPosition(START)
          // Simulation only: the arm's length and mass.
          .withMomentOfInertia(LENGTH, MASS)
          .withTelemetry("ArmMotor", TelemetryVerbosity.HIGH));

  private final Arm arm = new Arm(new ArmConfig()
      .withLength(LENGTH)
      .withHardLimits(MIN, MAX)
      .withTelemetry("Arm", TelemetryVerbosity.HIGH),
      motor);

  /** With nothing else running, the arm holds its stowed angle. */
  public ArmMechanism() {
    setDefaultCommand(setAngle(STOW));
  }

  /**
   * Move to an angle and hold it until canceled.
   *
   * @param angle Arm angle, 0 being level.
   * @return {@link Command}
   */
  public Command setAngle(Angle angle) {
    return arm.setAngle(angle);
  }

  /**
   * Wait until the arm is within {@link first.robot.Constants.ArmConstants#TOLERANCE} of an angle,
   * giving up after 3 s, then wait a second more. It requires nothing, so run it beside
   * {@link #setAngle(Angle)}, which holds the arm there.
   *
   * @param angle Arm angle to wait at.
   * @return {@link Command}
   */
  public Command settleAt(Angle angle) {
    return Command.noRequirements(coroutine -> {
      coroutine.waitUntil(() -> arm.isNear(angle, TOLERANCE), Seconds.of(3));
      coroutine.wait(Seconds.of(1));
    }).named("Arm Settle At " + angle.in(Degrees) + " degrees");
  }

  /**
   * The arm's angle.
   *
   * @return Arm angle, 0 being level.
   */
  public Angle getAngle() {
    return arm.getAngle();
  }

  /** Publishes YAMS telemetry. Called from {@link first.robot.Robot#robotPeriodic()}. */
  public void periodic() {
    arm.updateTelemetry();
  }

  /** Steps the YAMS simulation. Run in its own Notifier, started in {@link first.robot.Robot#simulationInit()}. */
  public void simulationPeriodic() {
    arm.simIterate();
  }
}
