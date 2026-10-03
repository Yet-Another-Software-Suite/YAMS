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
import yams.core.exceptions.PivotConfigurationException;
import yams.core.exceptions.SmartMotorControllerConfigurationException;
import yams.core.mechanisms.config.PivotConfig;
import yams.core.motorcontrollers.SmartMotorController;

/**
 * Command-based extension of {@link yams.core.mechanisms.positional.Pivot} that adds the
 * {@link Mechanism} binding and {@link Command}/{@link Trigger} factories.
 *
 * <h2>Usage Example</h2>
 * <pre>{@code
 * // --- Instantiation ---
 * Pivot pivot = new Pivot(pivotConfig, motor);
 *
 * // --- Commands ---
 * Command aimHigh = pivot.setAngle(Degrees.of(45));
 * Command stowPivot = pivot.runTo(Degrees.of(0), Degrees.of(1));
 *
 * // --- Trigger bindings ---
 * pivot.near(Degrees.of(45), Degrees.of(2)).onTrue(shooter.runShooter());
 * }</pre>
 */
public class Pivot extends yams.core.mechanisms.positional.Pivot implements CommandMechanism {
  /** Mechanism the pivot's commands should require. */
  private final Mechanism mechanism;

  /**
   * Construct the Pivot class
   *
   * @param config Pivot configuration.
   * @param smc    {@link SmartMotorController} driving the pivot.
   * @implNote {@code smc}'s config must be a {@link yams.commands3.config.SmartMotorControllerConfig}
   *           with a {@link Mechanism} set via {@code withMechanism(Mechanism)}.
   * @throws PivotConfigurationException if running in simulation and the pivot lower or upper hard
   *                                     limit is not configured.
   * @throws PivotConfigurationException if running in simulation and the starting position is not
   *                                     configured.
   * @throws PivotConfigurationException if running in simulation and the starting position is
   *                                     outside the hard limits.
   * @throws SmartMotorControllerConfigurationException if {@code smc}'s config does not have a
   *                                                    {@link Mechanism} set via
   *                                                    {@code withMechanism(Mechanism)}.
   */
  public Pivot(PivotConfig config, SmartMotorController smc) {
    super(config, smc);
    this.mechanism = ((yams.commands3.config.SmartMotorControllerConfig) smc.getConfig()).getMechanism();
  }

  @Override
  public Mechanism getMechanism() {
    return mechanism;
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
   * Greater than or equal to angle.
   *
   * @param angle Angle to check against.
   * @return {@link Trigger} for Pivot.
   */
  public Trigger gte(Angle angle) {
    return new Trigger(() -> isGte(angle));
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
   * Set the pivot to the given angle.
   *
   * @param angle Pivot angle to go to.
   * @return {@link Command} that sets the pivot to the desired angle.
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
   * Set the pivot to the given angle.
   *
   * @param angle Pivot angle to go to.
   * @return {@link Command} that sets the pivot to the desired angle.
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
   * Set the pivot to the given angle.
   *
   * @param angle Pivot angle to go to.
   * @return {@link Command} that sets the pivot to the desired angle.
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
   * Set the pivot to the given angle via a supplier.
   *
   * @param angle Supplier for the pivot angle to go to.
   * @return {@link Command} that sets the pivot to the desired angle.
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
   * Set the pivot to the given {@link Angle} then end the command.
   *
   * @param angle     {@link Angle} to go to.
   * @param tolerance Tolerance {@link Angle}
   * @return {@link Command} that sets the pivot to the desired angle.
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
   * Set the pivot to the given angle then end the command.
   *
   * @param angle     {@link Angle} to go to.
   * @param tolerance Tolerance {@link Angle}
   * @return {@link Command} that sets the pivot to the desired angle.
   * @implNote This command will not stop. It should NOT be used when there is a default command on
   *           the Mechanism.
   */
  public Command runTo(Supplier<Angle> angle, Angle tolerance) {
    return mechanism.run(coroutine -> {
      Angle target = angle.get();
      getMotorController().setPosition(target);
      coroutine.waitUntil(near(target, tolerance).debounce(Seconds.of(0.1), DebounceType.RISING));
    }).named(mechanism.getName() + " RunTo Angle Supplier");
  }

  /**
   * Pivot is near an angle.
   *
   * @param angle  {@link Angle} to be near.
   * @param within {@link Angle} within.
   * @return {@link Trigger} on when the pivot is near another angle.
   */
  public Trigger near(Angle angle, Angle within) {
    return new Trigger(() -> isNear(angle, within));
  }

  /**
   * {@link yams.core.mechanisms.positional.Pivot} is at max, defined by the soft limit or hard
   * limit on the pivot.
   *
   * @return {@link Trigger} on maximum of the pivot.
   * @throws PivotConfigurationException when the returned trigger is evaluated, if neither a motor
   *                                     controller upper soft limit nor a pivot upper hard limit is
   *                                     configured.
   */
  public Trigger max() {
    return new Trigger(this::isAtMax);
  }

  /**
   * Minimum angle of the pivot given by the soft limit or hard limit of the pivot.
   *
   * @return {@link Trigger} on minimum of the pivot.
   * @throws PivotConfigurationException when the returned trigger is evaluated, if neither a motor
   *                                     controller lower soft limit nor a pivot lower hard limit is
   *                                     configured.
   */
  public Trigger min() {
    return new Trigger(this::isAtMin);
  }
}
