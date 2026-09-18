// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package yams.commands2.mechanisms;

import java.util.function.Supplier;
import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import org.wpilib.command2.Subsystem;
import org.wpilib.command2.button.Trigger;
import org.wpilib.math.filter.Debouncer.DebounceType;
import org.wpilib.units.measure.Distance;
import yams.core.mechanisms.config.ElevatorConfig;
import yams.core.motorcontrollers.SmartMotorController;

/**
 * Command-based extension of {@link yams.core.mechanisms.positional.Elevator} that adds the
 * {@link Subsystem} binding and {@link Command}/{@link Trigger} factories.
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
 * elevator.max().onTrue(Commands.print("Elevator at max"));
 *
 * // In periodic():
 * elevator.simIterate();
 * elevator.updateTelemetry();
 * }</pre>
 */
public class Elevator extends yams.core.mechanisms.positional.Elevator implements CommandMechanism {
  /** Subsystem the elevator's commands should require. */
  private final Subsystem subsystem;

  /**
   * Construct the {@link Elevator} class for easy manipulation of an elevator.
   *
   * @param config {@link ElevatorConfig} to set.
   * @param smc    {@link SmartMotorController} to use for the Elevator
   * @implNote {@code smc}'s config must be a {@link yams.commands2.config.SmartMotorControllerConfig}
   *           with a {@link Subsystem} set via {@code withSubsystem(Subsystem)}.
   */
  public Elevator(ElevatorConfig config, SmartMotorController smc) {
    super(config, smc);
    this.subsystem = ((yams.commands2.config.SmartMotorControllerConfig) smc.getConfig()).getSubsystem();
  }

  @Override
  public Subsystem getSubsystem() {
    return subsystem;
  }

  /**
   * Set the height of the elevator.
   *
   * @param height Height of the elevator to reach.
   * @return {@link Command} that sets the elevator height, stops immediately.
   */
  public Command setHeight(Distance height) {
    return run(height).withName(subsystem.getName() + " SetHeight");
  }

  /**
   * Set the height of the elevator.
   *
   * @param height Height of the elevator to reach.
   * @return {@link Command} that sets the elevator height, stops immediately.
   */
  public Command setHeight(Supplier<Distance> height) {
    return run(height).withName(subsystem.getName() + " SetHeight Supplier");
  }

  /**
   * Set the height of the elevator.
   *
   * @param height Height of the elevator to reach.
   * @return {@link Command} that sets the elevator height, does not stop.
   */
  public Command run(Distance height) {
    return Commands.run(() -> getMotorController().setPosition(height), subsystem).withName(subsystem.getName() + " Run Height");
  }

  /**
   * Set the height of the elevator.
   *
   * @param height Height of the elevator to reach.
   * @return {@link Command} that sets the elevator height, stops immediately.
   */
  public Command run(Supplier<Distance> height) {
    return Commands.run(() -> getMotorController().setPosition(height.get()), subsystem).withName(subsystem.getName() + " Run Height Supplier");
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
    return Commands.runOnce(() -> getMotorController().setPosition(height), subsystem).andThen(Commands.waitUntil(near(height, tolerance).debounce(0.1, DebounceType.RISING))).withName(subsystem.getName() + " Run To Height");
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
    return Commands.runOnce(() -> getMotorController().setPosition(height.get()), subsystem).andThen(Commands.waitUntil(near(height.get(), tolerance).debounce(0.1, DebounceType.RISING))).withName(subsystem.getName() + " Run To Height Supplier");
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
   */
  public Trigger max() {
    return new Trigger(this::isAtMax);
  }

  /**
   * Minimum height of the elevator given by the soft limit or hard limit of the elevator.
   *
   * @return {@link Trigger} on minimum of the elevator.
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
