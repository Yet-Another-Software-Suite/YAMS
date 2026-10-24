// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package first.robot.mechanisms;

import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.Inches;
import static org.wpilib.units.Units.Pounds;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.util.CANPorts;
import first.robot.Constants.DoubleFlyWheelConstants;
import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;
import org.wpilib.command3.Trigger;
import org.wpilib.math.controller.SimpleMotorFeedforward;
import org.wpilib.math.system.DCMotor;
import org.wpilib.units.measure.AngularVelocity;
import yams.commands3.config.SmartMotorControllerConfig;
import yams.commands3.mechanisms.FlyWheel;
import yams.commands3.telemetry.SmartMotorControllerCommandRegistry;
import yams.core.gearing.MechanismGearing;
import yams.core.mechanisms.config.FlyWheelConfig;
import yams.core.motorcontrollers.SmartMotorController;
import yams.core.motorcontrollers.enums.ControlMode;
import yams.core.motorcontrollers.enums.MotorMode;
import yams.core.motorcontrollers.local.SparkWrapper;
import yams.core.telemetry.enums.TelemetryVerbosity;

/**
 * A double flywheel: two {@link FlyWheel}s, a lower and an upper wheel, in one mechanism. Running them at different
 * speeds puts spin on the game piece, which changes its arc.
 *
 * <p>Both wheels publish HIGH telemetry, so each one registers its live tuning on this mechanism. They share one
 * "Live Tuning" command, published at {@code /Tuning/DoubleFlyWheelMechanism/Live Tuning}: while it runs, each wheel
 * applies the values under its own {@code /Tuning/<wheel>} table. The YAMS Live Tuner tunes both wheels with that
 * one command.
 */
public class DoubleFlyWheelMechanism implements Mechanism {
  private final SmartMotorController lowerMotor = new SparkWrapper(
      new SparkMax(CANPorts.fromBusId(1), DoubleFlyWheelConstants.LOWER_MOTOR_ID, MotorType.kBrushless),
      DCMotor.getNEO(1),
      motorConfig("LowerMotor", false));
  private final FlyWheel lowerFlyWheel = new FlyWheel(new FlyWheelConfig()
      .withDiameter(DoubleFlyWheelConstants.WHEEL_DIAMETER)
      .withTelemetry("LowerFlyWheel", TelemetryVerbosity.HIGH),
      lowerMotor);

  private final SmartMotorController upperMotor = new SparkWrapper(
      new SparkMax(CANPorts.fromBusId(1), DoubleFlyWheelConstants.UPPER_MOTOR_ID, MotorType.kBrushless),
      DCMotor.getNEO(1),
      // The upper wheel faces the lower one, so it spins the other way to push the game piece out.
      motorConfig("UpperMotor", true));
  private final FlyWheel upperFlyWheel = new FlyWheel(new FlyWheelConfig()
      .withDiameter(DoubleFlyWheelConstants.WHEEL_DIAMETER)
      .withTelemetry("UpperFlyWheel", TelemetryVerbosity.HIGH),
      upperMotor);

  /**
   * The configuration both wheels' motors share. The feedforward starts a little low and the feedback small, so there
   * is something to tune live.
   *
   * @param telemetryName Telemetry name of the motor.
   * @param inverted      Whether the motor is inverted.
   * @return {@link SmartMotorControllerConfig} for one wheel's motor.
   */
  private SmartMotorControllerConfig motorConfig(String telemetryName, boolean inverted) {
    return new SmartMotorControllerConfig(this)
        .withControlMode(ControlMode.CLOSED_LOOP)
        // A flywheel's velocity loop runs without a motion profile.
        .withClosedLoopController(0.002, 0, 0)
        .withFeedforward(new SimpleMotorFeedforward(0, 0.1, 0))
        .withGearing(new MechanismGearing(1))
        .withZeroPower(MotorMode.COAST)
        .withStatorCurrentLimit(Amps.of(40))
        .withMotorInverted(inverted)
        // Simulation only: a 4 in, 1 lb wheel.
        .withMomentOfInertia(Inches.of(4), Pounds.of(1))
        .withTelemetry(telemetryName, TelemetryVerbosity.HIGH);
  }

  /**
   * Spin each wheel at its own speed until interrupted.
   *
   * @param lower Lower wheel speed.
   * @param upper Upper wheel speed.
   * @return {@link Command} that holds both speeds.
   */
  public Command spinUp(AngularVelocity lower, AngularVelocity upper) {
    return run(coroutine -> {
      while (true) {
        lowerFlyWheel.setMechanismVelocitySetpoint(lower);
        upperFlyWheel.setMechanismVelocitySetpoint(upper);
        coroutine.yield();
      }
    }).named("DoubleFlyWheel.SpinUp");
  }

  /** Lets both wheels coast down whenever nothing else is using them. */
  @Override
  public Command idle() {
    return run(coroutine -> {
      lowerFlyWheel.setDutyCycleSetpoint(0);
      upperFlyWheel.setDutyCycleSetpoint(0);
      coroutine.park();
    }).withPriority(Command.LOWEST_PRIORITY).named("DoubleFlyWheel.Coast");
  }

  /**
   * The "Live Tuning" command both wheels registered on this mechanism. The YAMS Live Tuner starts and stops it over
   * NetworkTables; it can also be bound to a button.
   *
   * @return The shared Live Tuning {@link Command}.
   */
  public Command liveTune() {
    return SmartMotorControllerCommandRegistry.getCommand("Live Tuning", this).orElseThrow();
  }

  /**
   * True while both wheels are within {@code tolerance} of the given speeds.
   *
   * @param lower     Lower wheel speed.
   * @param upper     Upper wheel speed.
   * @param tolerance Allowed error for each wheel.
   * @return {@link Trigger}
   */
  public Trigger atSpeed(AngularVelocity lower, AngularVelocity upper, AngularVelocity tolerance) {
    return lowerFlyWheel.near(lower, tolerance).and(upperFlyWheel.near(upper, tolerance));
  }

  /**
   * The lower wheel's speed.
   *
   * @return Lower wheel speed.
   */
  public AngularVelocity getLowerSpeed() {
    return lowerFlyWheel.getSpeed();
  }

  /**
   * The upper wheel's speed.
   *
   * @return Upper wheel speed.
   */
  public AngularVelocity getUpperSpeed() {
    return upperFlyWheel.getSpeed();
  }

  /** Publishes YAMS telemetry. Called from {@link first.robot.Robot#robotPeriodic()}. */
  public void periodic() {
    lowerFlyWheel.updateTelemetry();
    upperFlyWheel.updateTelemetry();
  }

  /** Steps the YAMS simulation. Run in its own Notifier, started in {@link first.robot.Robot#simulationInit()}. */
  public void simulationPeriodic() {
    lowerFlyWheel.simIterate();
    upperFlyWheel.simIterate();
  }
}
