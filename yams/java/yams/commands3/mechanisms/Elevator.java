// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package yams.commands3.mechanisms;

import static org.wpilib.units.Units.Seconds;

import java.util.function.Supplier;
import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;
import org.wpilib.command3.Trigger;
import org.wpilib.math.filter.Debouncer.DebounceType;
import org.wpilib.units.measure.Distance;
import yams.core.exceptions.ElevatorConfigurationException;
import yams.core.exceptions.SmartMotorControllerConfigurationException;
import yams.core.mechanisms.config.ElevatorConfig;
import yams.core.motorcontrollers.SmartMotorController;

/**
 * Command-based extension of {@link yams.core.mechanisms.positional.Elevator} that adds the
 * {@link Mechanism} binding and {@link Command}/{@link Trigger} factories.
 *
 * <h2>Usage Example</h2>
 * <pre>{@code
 * SmartMotorController motor = new TalonFXWrapper(
 *     new TalonFX(4), DCMotor.getKrakenX60(1),
 *     new SmartMotorControllerConfig(this).withClosedLoopController(0.5,0,0).withFeedforward(new
 * ElevatorFeedforward(0.4,0,0)); Elevator elevator = new Elevator( new
 * ElevatorConfig(motor).withDrumRadius(Inches.of(1.0)));
 *
 * // Move to heights
 * Command toHigh = elevator.setHeight(Meters.of(1.2));
 * Command toLow  = elevator.runTo(Meters.of(0.05), Meters.of(0.01));
 *
 * // Trigger bindings
 * elevator.near(Meters.of(1.2), Meters.of(0.02)).onTrue(shooter.shoot());
 *
 * // In periodic():
 * elevator.simIterate();
 * elevator.updateTelemetry();
 * }</pre>
 */
public class Elevator extends yams.core.mechanisms.positional.Elevator implements CommandMechanism {
  /** Mechanism the elevator's commands should require. */
  private final Mechanism mechanism;

  /**
   * Construct the {@link Elevator} class for easy manipulation of an elevator.
   *
   * @param config {@link ElevatorConfig} to set.
   * @param smc    {@link SmartMotorController} to use for the Elevator
   * @implNote {@code smc}'s config must be a {@link yams.commands3.config.SmartMotorControllerConfig}
   *           with a {@link Mechanism} set via {@code withMechanism(Mechanism)}.
   * @throws ElevatorConfigurationException if running in simulation and the carriage mass is not
   *                                        configured.
   * @throws ElevatorConfigurationException if running in simulation and the minimum or maximum
   *                                        height is not configured.
   * @throws SmartMotorControllerConfigurationException if {@code smc}'s config does not have a
   *                                                    {@link Mechanism} set via
   *                                                    {@code withMechanism(Mechanism)}.
   * @throws SmartMotorControllerConfigurationException if running in simulation and the starting
   *                                                    position is not configured.
   * @throws SmartMotorControllerConfigurationException if running in simulation and the mechanism
   *                                                    circumference is not configured.
   * @throws SmartMotorControllerConfigurationException if running in simulation and the starting
   *                                                    height is outside the hard limits.
   */
  public Elevator(ElevatorConfig config, SmartMotorController smc) {
    super(config, smc);
    this.mechanism = ((yams.commands3.config.SmartMotorControllerConfig) smc.getConfig()).getMechanism();
  }

  @Override
  public Mechanism getMechanism() {
    return mechanism;
  }

  /**
   * Set the height of the elevator.
   *
   * @param height Height of the elevator to reach.
   * @return {@link Command} that sets the elevator height, stops immediately.
   */
  public Command setHeight(Distance height) {
    return mechanism.run(coroutine -> {
      while (true) {
        getMotorController().setPosition(height);
        coroutine.yield();
      }
    }).named(mechanism.getName() + " SetHeight");
  }

  /**
   * Set the height of the elevator.
   *
   * @param height Height of the elevator to reach.
   * @return {@link Command} that sets the elevator height, stops immediately.
   */
  public Command setHeight(Supplier<Distance> height) {
    return mechanism.run(coroutine -> {
      while (true) {
        getMotorController().setPosition(height.get());
        coroutine.yield();
      }
    }).named(mechanism.getName() + " SetHeight Supplier");
  }

  /**
   * Set the height of the elevator.
   *
   * @param height Height of the elevator to reach.
   * @return {@link Command} that sets the elevator height, does not stop.
   */
  public Command run(Distance height) {
    return mechanism.run(coroutine -> {
      while (true) {
        getMotorController().setPosition(height);
        coroutine.yield();
      }
    }).named(mechanism.getName() + " Run Height");
  }

  /**
   * Set the height of the elevator.
   *
   * @param height Height of the elevator to reach.
   * @return {@link Command} that sets the elevator height, stops immediately.
   */
  public Command run(Supplier<Distance> height) {
    return mechanism.run(coroutine -> {
      while (true) {
        getMotorController().setPosition(height.get());
        coroutine.yield();
      }
    }).named(mechanism.getName() + " Run Height Supplier");
  }

  /**
   * Run the elevator to the desired height with a tolerance, then move on.
   *
   * @param height    Height to reach.
   * @param tolerance The acceptable tolerance
   * @return {@link Command} which will run the elevator to the desired height with a tolerance,
   *         then move on.
   * @implNote This should NOT be used with a default command, the mechanism will not stop running
   *           after this to allow for easy chaining.
   */
  public Command runTo(Distance height, Distance tolerance) {
    return mechanism.run(coroutine -> {
      getMotorController().setPosition(height);
      coroutine.waitUntil(near(height, tolerance).debounce(Seconds.of(0.1), DebounceType.RISING));
    }).named(mechanism.getName() + " Run To Height");
  }

  /**
   * Run the elevator to the desired height with a tolerance, then move on.
   *
   * @param height    Height to reach.
   * @param tolerance The acceptable tolerance
   * @return {@link Command} which will run the elevator to the desired height with a tolerance,
   *         then move on.
   * @implNote This should NOT be used with a default command, the mechanism will not stop running
   *           after this to allow for easy chaining.
   */
  public Command runTo(Supplier<Distance> height, Distance tolerance) {
    return mechanism.run(coroutine -> {
      Distance target = height.get();
      getMotorController().setPosition(target);
      coroutine.waitUntil(near(target, tolerance).debounce(Seconds.of(0.1), DebounceType.RISING));
    }).named(mechanism.getName() + " Run To Height Supplier");
  }

  /**
   * Elevator is near a height.
   *
   * @param height {@link Distance} to be near.
   * @param within {@link Distance} within.
   * @return Trigger on when the elevator is near another height.
   */
  public Trigger near(Distance height, Distance within) {
    return new Trigger(() -> isNear(height, within));
  }

  /**
   * {@link yams.core.mechanisms.positional.Elevator} is at max, defined by the soft limit or hard
   * limit on the elevator.
   *
   * @return {@link Trigger} on maximum of the elevator.
   * @throws ElevatorConfigurationException when the returned trigger is evaluated, if neither a
   *                                        motor controller upper soft limit nor an elevator
   *                                        maximum height is configured.
   */
  public Trigger max() {
    return new Trigger(this::isAtMax);
  }

  /**
   * Minimum height of the elevator given by the soft limit or hard limit of the elevator.
   *
   * @return {@link Trigger} on minimum of the elevator.
   * @throws ElevatorConfigurationException when the returned trigger is evaluated, if neither a
   *                                        motor controller lower soft limit nor an elevator
   *                                        minimum height is configured.
   */
  public Trigger min() {
    return new Trigger(this::isAtMin);
  }

  /**
   * Between two heights.
   *
   * @param start Start height.
   * @param end   End height.
   * @return {@link Trigger}
   */
  public Trigger between(Distance start, Distance end) {
    return gte(start).and(lte(end));
  }

  /**
   * Less than or equal to height
   *
   * @param height {@link Distance} to check against
   * @return {@link Trigger}
   */
  public Trigger lte(Distance height) {
    return new Trigger(() -> isLte(height));
  }

  /**
   * Greater than or equal to height.
   *
   * @param height Height to check against.
   * @return {@link Trigger} for elevator.
   */
  public Trigger gte(Distance height) {
    return new Trigger(() -> isGte(height));
  }
}
