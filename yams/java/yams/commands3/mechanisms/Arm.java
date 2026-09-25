// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package yams.commands3.mechanisms;

import static org.wpilib.units.Units.Seconds;

import java.util.function.Supplier;
import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;
import org.wpilib.command3.Trigger;
import org.wpilib.math.filter.Debouncer.DebounceType;
import org.wpilib.units.measure.Angle;
import yams.core.mechanisms.config.ArmConfig;
import yams.core.motorcontrollers.SmartMotorController;

/**
 * Command-based extension of {@link yams.core.mechanisms.positional.Arm} that adds the
 * {@link Mechanism} binding and {@link Command}/{@link Trigger} factories.
 *
 * <h2>Usage Example</h2>
 *
 * <pre>{@code
 * // Build and construct
 * SmartMotorController motor = new SparkWrapper(
 *     new SparkMax(1, MotorType.kBrushless), DCMotor.getNEO(1),
 *     new
 * SmartMotorControllerConfig(this).withClosedLoopController(0.2,0,0).withStatorCurrentLimit(Amps.of(40)));
 * ArmConfig armConfig = new ArmConfig().withLength(Meters.of(0.5));
 * Arm arm = new Arm(armConfig, motor);
 *
 * // Schedule a setpoint command
 * Command moveToScore = arm.setAngle(Degrees.of(80));
 * Command holdAtZero = arm.runTo(Degrees.of(0), Degrees.of(2));
 *
 * // Bind triggers
 * arm.near(Degrees.of(80), Degrees.of(2)).onTrue(indexer.run());
 * arm.max().onTrue(Command.noRequirements(co -> System.out.println("Arm at max!")).named("Print"));
 *
 * // Call in robotPeriodic() or a subsystem's periodic():
 * arm.simIterate();
 * arm.updateTelemetry();
 * }</pre>
 */
public class Arm extends yams.core.mechanisms.positional.Arm implements CommandMechanism {
  /** Mechanism the arm's commands should require. */
  private final Mechanism mechanism;

  /**
   * Constructor for the Arm mechanism.
   *
   * @param config {@link ArmConfig} to use.
   * @param smc    {@link SmartMotorController} for the Arm.
   * @implNote {@code smc}'s config must be a {@link yams.commands3.config.SmartMotorControllerConfig}
   *           with a {@link Mechanism} set via {@code withMechanism(Mechanism)}.
   */
  public Arm(ArmConfig config, SmartMotorController smc) {
    super(config, smc);
    this.mechanism = ((yams.commands3.config.SmartMotorControllerConfig) smc.getConfig()).getMechanism();
  }

  @Override
  public Mechanism getMechanism() {
    return mechanism;
  }

  /**
   * Set the arm to the given angle.
   *
   * @param angle Arm angle to go to.
   * @return {@link Command} that sets the arm to the desired angle.
   */
  public Command setAngle(Angle angle) {
    return mechanism.run(coroutine -> {
      while (true) {
        getMotorController().setPosition(angle);
        coroutine.yield();
      }
    }).named(mechanism.getName() + " SetAngle");
  }

  /**
   * Set the arm to the given angle via a supplier.
   *
   * @param angle Supplier for the arm angle to go to.
   * @return {@link Command} that sets the arm to the desired angle.
   */
  public Command setAngle(Supplier<Angle> angle) {
    return mechanism.run(coroutine -> {
      while (true) {
        getMotorController().setPosition(angle.get());
        coroutine.yield();
      }
    }).named(mechanism.getName() + " SetAngle Supplier");
  }

  /**
   * Set the arm to the given angle.
   *
   * @param angle Arm angle to go to.
   * @return {@link Command} that sets the arm to the desired angle.
   */
  public Command run(Angle angle) {
    return mechanism.run(coroutine -> {
      while (true) {
        getMotorController().setPosition(angle);
        coroutine.yield();
      }
    }).named(mechanism.getName() + " SetAngle");
  }

  /**
   * Set the arm to the given angle via a supplier.
   *
   * @param angle Supplier for the arm angle to go to.
   * @return {@link Command} that sets the arm to the desired angle.
   */
  public Command run(Supplier<Angle> angle) {
    return mechanism.run(coroutine -> {
      while (true) {
        getMotorController().setPosition(angle.get());
        coroutine.yield();
      }
    }).named(mechanism.getName() + " RunAngle Supplier");
  }

  /**
   * Set the arm to the given angle then end the command.
   *
   * @param angle     {@link Angle} to go to.
   * @param tolerance Tolerance {@link Angle}
   * @return {@link Command} that sets the arm to the desired angle.
   * @implNote This command will not stop. It should NOT be used when there is a default command on
   *           the Mechanism.
   */
  public Command runTo(Angle angle, Angle tolerance) {
    return mechanism.run(coroutine -> {
      getMotorController().setPosition(angle);
      coroutine.waitUntil(near(angle, tolerance).debounce(Seconds.of(0.1), DebounceType.RISING));
    }).named(mechanism.getName() + " RunTo Angle");
  }

  /**
   * Set the arm to the given angle then end the command.
   *
   * @param angle     {@link Angle} to go to.
   * @param tolerance Tolerance {@link Angle}
   * @return {@link Command} that sets the arm to the desired angle.
   * @implNote This command will stop, but the last control request to the motor controller will
   *           continue. It should NOT be used when there is a default command on the Mechanism.
   */
  public Command runTo(Supplier<Angle> angle, Angle tolerance) {
    return mechanism.run(coroutine -> {
      Angle target = angle.get();
      getMotorController().setPosition(target);
      coroutine.waitUntil(near(target, tolerance).debounce(Seconds.of(0.1), DebounceType.RISING));
    }).named(mechanism.getName() + " RunTo Angle Supplier");
  }

  /**
   * Arm is near an angle.
   *
   * @param angle  {@link Angle} to be near.
   * @param within {@link Angle} within.
   * @return {@link Trigger} on when the arm is near another angle.
   */
  public Trigger near(Angle angle, Angle within) {
    return new Trigger(() -> isNear(angle, within));
  }

  /**
   * {@link yams.core.mechanisms.positional.Arm} is at max, defined by the soft limit or hard limit
   * on the arm.
   *
   * @return {@link Trigger} on maximum of the arm.
   */
  public Trigger max() {
    return new Trigger(this::isAtMax);
  }

  /**
   * Minimum angle of the arm given by the soft limit or hard limit of the arm.
   *
   * @return {@link Trigger} on minimum of the arm.
   */
  public Trigger min() {
    return new Trigger(this::isAtMin);
  }

  /**
   * Between two angles.
   *
   * @param start Start angle.
   * @param end   End angle
   * @return {@link Trigger}
   */
  public Trigger between(Angle start, Angle end) {
    return gte(start).and(lte(end));
  }

  /**
   * Less than or equal to angle
   *
   * @param angle {@link Angle} to check against
   * @return {@link Trigger}
   */
  public Trigger lte(Angle angle) {
    return new Trigger(() -> isLte(angle));
  }

  /**
   * Greater than or equal to angle.
   *
   * @param angle Angle to check against.
   * @return {@link Trigger} for Arm.
   */
  public Trigger gte(Angle angle) {
    return new Trigger(() -> isGte(angle));
  }
}
