// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package yams.commands2.mechanisms;

import java.util.function.Supplier;
import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import org.wpilib.command2.Subsystem;
import org.wpilib.command2.button.Trigger;
import org.wpilib.math.filter.Debouncer.DebounceType;
import org.wpilib.units.measure.Angle;
import yams.core.exceptions.ArmConfigurationException;
import yams.core.exceptions.SmartMotorControllerConfigurationException;
import yams.core.mechanisms.config.ArmConfig;
import yams.core.motorcontrollers.SmartMotorController;

/**
 * Command-based extension of {@link yams.core.mechanisms.positional.Arm} that adds the
 * {@link Subsystem} binding and {@link Command}/{@link Trigger} factories.
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
 * arm.max().onTrue(Commands.print("Arm at max!"));
 *
 * // Call in robotPeriodic() or a subsystem's periodic():
 * arm.simIterate();
 * arm.updateTelemetry();
 * }</pre>
 */
public class Arm extends yams.core.mechanisms.positional.Arm implements CommandMechanism {
  /** Subsystem the arm's commands should require. */
  private final Subsystem subsystem;

  /**
   * Constructor for the Arm mechanism.
   *
   * @param config {@link ArmConfig} to use.
   * @param smc    {@link SmartMotorController} for the Arm.
   * @implNote {@code smc}'s config must be a {@link yams.commands2.config.SmartMotorControllerConfig}
   *           with a {@link Subsystem} set via {@code withSubsystem(Subsystem)}.
   * @throws ArmConfigurationException if running in simulation and the arm length is not
   *                                   configured.
   * @throws ArmConfigurationException if running in simulation and the lower or upper hard limit is
   *                                   not configured.
   * @throws ArmConfigurationException if running in simulation and neither a starting position nor
   *                                   an external encoder zero offset is configured.
   * @throws ArmConfigurationException if running in simulation and the starting position is outside
   *                                   the hard limits.
   * @throws SmartMotorControllerConfigurationException if {@code smc}'s config does not have a
   *                                                    {@link Subsystem} set via
   *                                                    {@code withSubsystem(Subsystem)}.
   */
  public Arm(ArmConfig config, SmartMotorController smc) {
    super(config, smc);
    this.subsystem = ((yams.commands2.config.SmartMotorControllerConfig) smc.getConfig()).getSubsystem();
  }

  @Override
  public Subsystem getSubsystem() {
    return subsystem;
  }

  /**
   * Set the arm to the given angle.
   *
   * @param angle Arm angle to go to.
   * @return {@link Command} that sets the arm to the desired angle.
   */
  public Command setAngle(Angle angle) {
    return run(angle).withName(subsystem.getName() + " SetAngle");
  }

  /**
   * Set the arm to the given angle via a supplier.
   *
   * @param angle Supplier for the arm angle to go to.
   * @return {@link Command} that sets the arm to the desired angle.
   */
  public Command setAngle(Supplier<Angle> angle) {
    return run(angle).withName(subsystem.getName() + " SetAngle Supplier");
  }

  /**
   * Set the arm to the given angle.
   *
   * @param angle Arm angle to go to.
   * @return {@link Command} that sets the arm to the desired angle.
   */
  public Command run(Angle angle) {
    return Commands.run(() -> getMotorController().setPosition(angle), subsystem).withName(subsystem.getName() + " SetAngle");
  }

  /**
   * Set the arm to the given angle via a supplier.
   *
   * @param angle Supplier for the arm angle to go to.
   * @return {@link Command} that sets the arm to the desired angle.
   */
  public Command run(Supplier<Angle> angle) {
    return Commands.run(() -> getMotorController().setPosition(angle.get()), subsystem).withName(subsystem.getName() + " RunAngle Supplier");
  }

  /**
   * Set the arm to the given angle then end the command.
   *
   * @param angle     {@link Angle} to go to.
   * @param tolerance Tolerance {@link Angle}
   * @return {@link Command} that sets the arm to the desired angle.
   * @implNote This command will not stop. It should NOT be used when there is a default command on
   *           the Subsystem.
   */
  public Command runTo(Angle angle, Angle tolerance) {
    return Commands.runOnce(() -> getMotorController().setPosition(angle), subsystem).andThen(Commands.waitUntil(near(angle, tolerance).debounce(0.1, DebounceType.RISING))).withName(subsystem.getName() + " RunTo Angle");
  }

  /**
   * Set the arm to the given angle then end the command.
   *
   * @param angle     {@link Angle} to go to.
   * @param tolerance Tolerance {@link Angle}
   * @return {@link Command} that sets the arm to the desired angle.
   * @implNote This command will stop, but the last control request to the motor controller will
   *           continue. It should NOT be used when there is a default command on the Subsystem.
   */
  public Command runTo(Supplier<Angle> angle, Angle tolerance) {
    return Commands.runOnce(() -> getMotorController().setPosition(angle.get()), subsystem).andThen(Commands.waitUntil(near(angle.get(), tolerance).debounce(0.1, DebounceType.RISING))).withName(subsystem.getName() + " RunTo Angle Supplier");
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
   * @throws ArmConfigurationException when the returned trigger is evaluated, if neither a motor
   *                                   controller upper soft limit nor an arm upper hard limit is
   *                                   configured.
   */
  public Trigger max() {
    return new Trigger(this::isAtMax);
  }

  /**
   * Minimum angle of the arm given by the soft limit or hard limit of the arm.
   *
   * @return {@link Trigger} on minimum of the arm.
   * @throws ArmConfigurationException when the returned trigger is evaluated, if neither a motor
   *                                   controller lower soft limit nor an arm lower hard limit is
   *                                   configured.
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
