// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package yams.commands2.mechanisms;

import java.util.function.Supplier;
import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import org.wpilib.command2.Subsystem;
import org.wpilib.command2.button.Trigger;
import org.wpilib.math.filter.Debouncer.DebounceType;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.LinearVelocity;
import yams.core.mechanisms.config.FlyWheelConfig;
import yams.core.motorcontrollers.SmartMotorController;

/**
 * Command-based extension of {@link yams.core.mechanisms.velocity.FlyWheel} that adds the
 * {@link Subsystem} binding and {@link Command}/{@link Trigger} factories.
 *
 * <h2>Usage Example</h2>
 * <pre>{@code
 * // Construct using a fully configured FlyWheelConfig
 * FlyWheel shooter = new FlyWheel(config);
 *
 * // Spin at a fixed target RPM (runs continuously as a RunCommand)
 * Command spinUp = shooter.run(RPM.of(3000));
 *
 * // Block until the wheel reaches 3000 RPM within ±50 RPM, then finish
 * Command spinToSpeed = shooter.runTo(RPM.of(3000), RPM.of(50));
 *
 * // Trigger that is true whenever the wheel is within ±50 RPM of target
 * Trigger atSpeed = shooter.near(RPM.of(3000), RPM.of(50));
 * atSpeed.onTrue(Commands.print("Shooter at speed!"));
 * }</pre>
 */
public class FlyWheel extends yams.core.mechanisms.velocity.FlyWheel implements CommandMechanism {
  /** Subsystem the FlyWheel's commands should require. */
  private final Subsystem subsystem;

  /**
   * Construct the FlyWheel class
   *
   * @param config FlyWheel configuration.
   * @param smc    {@link SmartMotorController} for the Mechanism
   * @implNote {@code smc}'s config must be a {@link yams.commands2.config.SmartMotorControllerConfig}
   *           with a {@link Subsystem} set via {@code withSubsystem(Subsystem)}.
   */
  public FlyWheel(FlyWheelConfig config, SmartMotorController smc) {
    super(config, smc);
    this.subsystem = ((yams.commands2.config.SmartMotorControllerConfig) smc.getConfig()).getSubsystem();
  }

  @Override
  public Subsystem getSubsystem() {
    return subsystem;
  }

  /**
   * Between two velocities.
   *
   * @param start Start Velocity.
   * @param end   End velocity
   * @return {@link Trigger}
   */
  public Trigger between(AngularVelocity start, AngularVelocity end) {
    return gte(start).and(lte(end));
  }

  /**
   * Greater than or equal to angular velocity.
   *
   * @param speed {@link AngularVelocity} to check against.
   * @return {@link Trigger} for FlyWheel.
   */
  public Trigger gte(AngularVelocity speed) {
    return new Trigger(() -> isGte(speed));
  }

  /**
   * Less than or equal to angular velocity
   *
   * @param speed {@link AngularVelocity} to check against
   * @return {@link Trigger}
   */
  public Trigger lte(AngularVelocity speed) {
    return new Trigger(() -> isLte(speed));
  }

  /**
   * FlyWheel is near a speed.
   *
   * @param speed  {@link AngularVelocity} to be near.
   * @param within {@link AngularVelocity} within.
   * @return Trigger on when the FlyWheel is near another speed.
   */
  public Trigger near(AngularVelocity speed, AngularVelocity within) {
    return new Trigger(() -> isNear(speed, within));
  }

  /**
   * Run the FlyWheel to a given velocity
   *
   * @param velocity {@link Supplier} of {@link LinearVelocity} or {@link AngularVelocity}
   * @param <T>      Must be a {@link LinearVelocity} or {@link AngularVelocity}
   * @return {@link Command} which runs the FlyWheel to the desired velocity with the closed loop
   *         controller.
   */
  public <T> Command run(Supplier<T> velocity) {
    SmartMotorController smc = getMotorController();
    var cmdName = subsystem.getName() + " RunSpeed Supplier";
    if (velocity.get() instanceof AngularVelocity) {
      return Commands.startRun(smc::startClosedLoopController, () -> smc.setVelocity((AngularVelocity) velocity.get()), subsystem).withName(cmdName);
    } else if (velocity.get() instanceof LinearVelocity) {
      getShooterConfig().getCircumference(); // Circumference check
      return Commands.startRun(smc::startClosedLoopController, () -> smc.setVelocity(getShooterConfig().getAngularVelocity((LinearVelocity) velocity.get())), subsystem).withName(cmdName);
    }
    throw new IllegalArgumentException("Velocity must be an AngularVelocity or LinearVelocity");
  }

  /**
   * Set the FlyWheel to the given speed.
   *
   * @param velocity FlyWheel speed to go to.
   * @return {@link Command} that sets the FlyWheel to the desired speed.
   */
  public Command run(AngularVelocity velocity) {
    return Commands.run(() -> getMotorController().setVelocity(velocity), subsystem).withName(subsystem.getName() + " " + getName() + " SetSpeed");
  }

  /**
   * Run the FlyWheel to a velocity within a tolerance, then end the command.
   *
   * @param velocity  {@link Supplier} of {@link AngularVelocity}
   * @param tolerance {@link AngularVelocity} tolerance
   * @return {@link Command} that runs the FlyWheel to the desired velocity then moves on.
   * @implNote If you are using this function, try not to have a default command or else the default
   *           command will override the setting after this command ends.
   */
  public Command runTo(Supplier<AngularVelocity> velocity, AngularVelocity tolerance) {
    SmartMotorController smc = getMotorController();
    return Commands.runOnce(smc::startClosedLoopController, subsystem).andThen(Commands.runOnce(() -> smc.setVelocity(velocity.get()), subsystem)).andThen(Commands.waitUntil(near(velocity.get(), tolerance).debounce(0.1, DebounceType.RISING)))
        .withName(subsystem.getName() + " RunToVelocity Supplier");
  }

  /**
   * Run the FlyWheel to a velocity within a tolerance, then end the command.
   *
   * @param velocity  {@link AngularVelocity} to go to.
   * @param tolerance {@link AngularVelocity} tolerance
   * @return {@link Command} that runs the FlyWheel to the desired velocity then moves on.
   * @implNote If you are using this function, try not to have a default command or else the default
   *           command will override the setting after this command ends.
   */
  public Command runTo(AngularVelocity velocity, AngularVelocity tolerance) {
    SmartMotorController smc = getMotorController();
    return Commands.runOnce(smc::startClosedLoopController, subsystem).andThen(Commands.runOnce(() -> smc.setVelocity(velocity), subsystem)).andThen(Commands.waitUntil(near(velocity, tolerance).debounce(0.1, DebounceType.RISING))).withName(subsystem
        .getName() + " RunToVelocity");
  }

  /**
   * Run the FlyWheel to a velocity within a tolerance then end the command.
   *
   * @param velocity  {@link LinearVelocity} to go to.
   * @param tolerance {@link LinearVelocity} tolerance
   * @return {@link Command} that runs the FlyWheel to the desired velocity then moves on.
   * @implNote If you are using this function, try not to have a default command or else the default
   *           command will override the setting after this command ends.
   */
  public Command runTo(LinearVelocity velocity, LinearVelocity tolerance) {
    getShooterConfig().getCircumference(); // Circumference check
    return runTo(getShooterConfig().getAngularVelocity(velocity), getShooterConfig().getAngularVelocity(tolerance));
  }

  /**
   * Run the FlyWheel to a velocity within a tolerance then end the command.
   *
   * @param velocity  {@link LinearVelocity} to go to.
   * @param tolerance {@link LinearVelocity} tolerance
   * @return {@link Command} that runs the FlyWheel to the desired velocity then moves on.
   * @implNote If you are using this function, try not to have a default command or else the default
   *           command will override the setting after this command ends.
   */
  public Command runTo(Supplier<LinearVelocity> velocity, LinearVelocity tolerance) {
    getShooterConfig().getCircumference(); // Circumference check
    return runTo(() -> getShooterConfig().getAngularVelocity(velocity.get()), getShooterConfig().getAngularVelocity(tolerance));
  }

  /**
   * Set the FlyWheel to the given speed.
   *
   * @param speed FlyWheel speed to go to.
   * @return {@link Command} that sets the FlyWheel to the desired speed.
   */
  public Command run(LinearVelocity speed) {
    return run(getShooterConfig().getAngularVelocity(speed)).withName(subsystem.getName() + " RunSpeed");
  }
}
