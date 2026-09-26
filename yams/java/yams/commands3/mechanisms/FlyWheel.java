// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package yams.commands3.mechanisms;

import static org.wpilib.units.Units.Seconds;

import java.util.function.Supplier;
import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;
import org.wpilib.command3.Trigger;
import org.wpilib.math.filter.Debouncer.DebounceType;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.LinearVelocity;
import yams.core.exceptions.FlyWheelConfigurationException;
import yams.core.exceptions.SmartMotorControllerConfigurationException;
import yams.core.mechanisms.config.FlyWheelConfig;
import yams.core.motorcontrollers.SmartMotorController;

/**
 * Command-based extension of {@link yams.core.mechanisms.velocity.FlyWheel} that adds the
 * {@link Mechanism} binding and {@link Command}/{@link Trigger} factories.
 *
 * <h2>Usage Example</h2>
 * <pre>{@code
 * // Construct using a fully configured FlyWheelConfig
 * FlyWheel shooter = new FlyWheel(config);
 *
 * // Spin at a fixed target RPM (runs continuously as a repeated command)
 * Command spinUp = shooter.run(RPM.of(3000));
 *
 * // Block until the wheel reaches 3000 RPM within ±50 RPM, then finish
 * Command spinToSpeed = shooter.runTo(RPM.of(3000), RPM.of(50));
 *
 * // Trigger that is true whenever the wheel is within ±50 RPM of target
 * Trigger atSpeed = shooter.near(RPM.of(3000), RPM.of(50));
 * }</pre>
 */
public class FlyWheel extends yams.core.mechanisms.velocity.FlyWheel implements CommandMechanism {
  /** Mechanism the FlyWheel's commands should require. */
  private final Mechanism mechanism;

  /**
   * Construct the FlyWheel class
   *
   * @param config FlyWheel configuration.
   * @param smc    {@link SmartMotorController} for the Mechanism
   * @implNote {@code smc}'s config must be a {@link yams.commands3.config.SmartMotorControllerConfig}
   *           with a {@link Mechanism} set via {@code withMechanism(Mechanism)}.
   * @throws SmartMotorControllerConfigurationException if {@code smc}'s config does not have a
   *                                                    {@link Mechanism} set via
   *                                                    {@code withMechanism(Mechanism)}.
   */
  public FlyWheel(FlyWheelConfig config, SmartMotorController smc) {
    super(config, smc);
    this.mechanism = ((yams.commands3.config.SmartMotorControllerConfig) smc.getConfig()).getMechanism();
  }

  @Override
  public Mechanism getMechanism() {
    return mechanism;
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
   * @throws IllegalArgumentException if the value returned by {@code velocity} when this method is
   *                                  called is neither an {@link AngularVelocity} nor a
   *                                  {@link LinearVelocity}.
   * @throws FlyWheelConfigurationException if the value returned by {@code velocity} is a
   *                                        {@link LinearVelocity} and the FlyWheel diameter is not
   *                                        configured.
   */
  public <T> Command run(Supplier<T> velocity) {
    SmartMotorController smc = getMotorController();
    var cmdName = mechanism.getName() + " RunSpeed Supplier";
    if (velocity.get() instanceof AngularVelocity) {
      return mechanism.run(coroutine -> {
        smc.startClosedLoopController();
        while (true) {
          smc.setVelocity((AngularVelocity) velocity.get());
          coroutine.yield();
        }
      }).named(cmdName);
    } else if (velocity.get() instanceof LinearVelocity) {
      getShooterConfig().getCircumference(); // Circumference check
      return mechanism.run(coroutine -> {
        smc.startClosedLoopController();
        while (true) {
          smc.setVelocity(getShooterConfig().getAngularVelocity((LinearVelocity) velocity.get()));
          coroutine.yield();
        }
      }).named(cmdName);
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
    return mechanism.run(coroutine -> {
      while (true) {
        getMotorController().setVelocity(velocity);
        coroutine.yield();
      }
    }).named(mechanism.getName() + " " + getName() + " SetSpeed");
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
    return mechanism.run(coroutine -> {
      smc.startClosedLoopController();
      AngularVelocity target = velocity.get();
      smc.setVelocity(target);
      coroutine.waitUntil(near(target, tolerance).debounce(Seconds.of(0.1), DebounceType.RISING));
    }).named(mechanism.getName() + " RunToVelocity Supplier");
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
    return mechanism.run(coroutine -> {
      smc.startClosedLoopController();
      smc.setVelocity(velocity);
      coroutine.waitUntil(near(velocity, tolerance).debounce(Seconds.of(0.1), DebounceType.RISING));
    }).named(mechanism.getName() + " RunToVelocity");
  }

  /**
   * Run the FlyWheel to a velocity within a tolerance then end the command.
   *
   * @param velocity  {@link LinearVelocity} to go to.
   * @param tolerance {@link LinearVelocity} tolerance
   * @return {@link Command} that runs the FlyWheel to the desired velocity then moves on.
   * @implNote If you are using this function, try not to have a default command or else the default
   *           command will override the setting after this command ends.
   * @throws FlyWheelConfigurationException if the FlyWheel diameter is not configured, since it is
   *                                        needed to convert the linear velocity to an angular
   *                                        velocity.
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
   * @throws FlyWheelConfigurationException if the FlyWheel diameter is not configured, since it is
   *                                        needed to convert the linear velocity to an angular
   *                                        velocity.
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
   * @throws FlyWheelConfigurationException if the FlyWheel diameter is not configured, since it is
   *                                        needed to convert the linear velocity to an angular
   *                                        velocity.
   */
  public Command run(LinearVelocity speed) {
    AngularVelocity target = getShooterConfig().getAngularVelocity(speed);
    return mechanism.run(coroutine -> {
      while (true) {
        getMotorController().setVelocity(target);
        coroutine.yield();
      }
    }).named(mechanism.getName() + " RunSpeed");
  }
}
