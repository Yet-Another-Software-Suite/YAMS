// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package yams.commands3.mechanisms;

import java.util.function.Supplier;
import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.Distance;
import yams.core.exceptions.DoubleJointedArmConfigurationException;
import yams.core.mechanisms.config.ArmConfig;
import yams.core.motorcontrollers.SmartMotorController;

/**
 * Command-based extension of {@link yams.core.mechanisms.positional.DoubleJointedArm} that adds
 * the {@link Mechanism} binding and {@link Command} factories.
 *
 * <h2>Control Examples</h2>
 * <pre>{@code
 * // Set explicit joint angles: shoulder at 45°, elbow at 90°.
 * Command positionCommand = arm.setAngle(Degrees.of(45), Degrees.of(90));
 *
 * // Move to a Cartesian target (0.4 m forward, 0.6 m up) using inverse kinematics.
 * Command ikCommand = arm.setPosition(new Translation2d(0.4, 0.6), false);
 *
 * // Continuously track a target while within 5 cm tolerance, then finish.
 * Command trackCommand = arm.runTo(new Translation2d(0.5, 0.7), false, Centimeters.of(5));
 * }</pre>
 */
public class DoubleJointedArm extends yams.core.mechanisms.positional.DoubleJointedArm implements CommandMechanism {
  /** Mechanism the arm's commands should require. */
  private final Mechanism mechanism;

  /**
   * Constructor for the Arm mechanism.
   *
   * @param lowerConfig Lower {@link ArmConfig} to use.
   * @param lowerSMC    {@link SmartMotorController} driving the lower joint.
   * @param upperConfig Upper {@link ArmConfig} to use.
   * @param upperSMC    {@link SmartMotorController} driving the upper joint.
   * @implNote Both motor controllers' configs must be {@link
   *           yams.commands3.config.SmartMotorControllerConfig}s that share the same
   *           {@link Mechanism} set via {@code withMechanism(Mechanism)}.
   */
  public DoubleJointedArm(ArmConfig lowerConfig, SmartMotorController lowerSMC, ArmConfig upperConfig, SmartMotorController upperSMC) {
    super(lowerConfig, lowerSMC, upperConfig, upperSMC);
    yams.commands3.config.SmartMotorControllerConfig lowerCfg = (yams.commands3.config.SmartMotorControllerConfig) lowerSMC.getConfig();
    yams.commands3.config.SmartMotorControllerConfig upperCfg = (yams.commands3.config.SmartMotorControllerConfig) upperSMC.getConfig();
    if (lowerCfg.getMechanism() != upperCfg.getMechanism()) {
      throw new DoubleJointedArmConfigurationException("SmartMotorControllers do not have the same mechanism!", "Cannot create commands for single mechanism.", "withMechanism(this)");
    }
    this.mechanism = lowerCfg.getMechanism();
  }

  @Override
  public Mechanism getMechanism() {
    return mechanism;
  }

  /**
   * Set the position of the DoubleJointedArm to be at pose in meters.
   *
   * @param translation {@link Translation2d} where X is away from root, and Y is up.
   * @param invert      Invert the eblow.
   * @return {@link Command} that will reach the specified goal.
   */
  public Command setPosition(Translation2d translation, boolean invert) {
    return Command.noRequirements(coroutine -> {
      var thetas = getAnglesForPosition(translation, invert);
      coroutine.await(setAngle(thetas.getFirst(), thetas.getSecond()));
    }).named(mechanism.getName() + " SetPosition");
  }

  /**
   * Set the position of the DoubleJointedArm to be at pose in meters.
   *
   * @param translation2d {@link Translation2d} where X is away from root, and Y is up.
   * @param invert        Invert the eblow.
   * @return {@link Command} that will reach the specified goal.
   */
  public Command run(Translation2d translation2d, boolean invert) {
    return run(() -> translation2d, () -> invert);
  }

  /**
   * Set the position of the DoubleJointedArm to be at pose in meters.
   *
   * @param translation {@link Supplier<Translation2d>} where X is away from root, and Y is up.
   * @param invert      {@link Supplier<Boolean>} Invert the eblow.
   * @return {@link Command} that will reach the specified goal.
   */
  public Command run(Supplier<Translation2d> translation, Supplier<Boolean> invert) {
    SmartMotorController lower = getLowerMotorController();
    SmartMotorController upper = getUpperMotorController();
    return mechanism.run(coroutine -> {
      while (true) {
        var thetas = getAnglesForPosition(translation.get(), invert.get());
        lower.setPosition(thetas.getFirst());
        upper.setPosition(thetas.getSecond());
        coroutine.yield();
      }
    }).named(mechanism.getName() + " Run Position");
  }

  /**
   * Runs to the given target and stops the command. Note, if you have a default command it will
   * take over.
   *
   * @param translation2d Target position
   * @param invert        Invert the elbow direction.
   * @param tolerance     Tolerance
   * @return {@link Command} that will reach the specified goal.
   */
  public Command runTo(Translation2d translation2d, boolean invert, Distance tolerance) {
    return runTo(() -> translation2d, () -> invert, tolerance);
  }

  /**
   * Runs to the given target and stops the command. Note, if you have a default command it will
   * take over.
   *
   * @param translation Target position
   * @param invert      Invert the elbow direction.
   * @param tolerance   Tolerance
   * @return {@link Command} that will reach the specified goal.
   */
  public Command runTo(Supplier<Translation2d> translation, Supplier<Boolean> invert, Distance tolerance) {
    SmartMotorController lower = getLowerMotorController();
    SmartMotorController upper = getUpperMotorController();
    return mechanism.run(coroutine -> {
      while (!isNear(translation.get(), tolerance)) {
        var thetas = getAnglesForPosition(translation.get(), invert.get());
        lower.setPosition(thetas.getFirst());
        upper.setPosition(thetas.getSecond());
        coroutine.yield();
      }
    }).named(mechanism.getName() + " RunTo Position");
  }

  /**
   * Set the shoulder and elbow angle of the DoubleJointedArm.
   *
   * @param lowerAngle {@link Angle} of the shoulder.
   * @param upperAngle {@link Angle} of the elbow.
   * @return {@link Command} that will set the angles.
   */
  public Command setAngle(Angle lowerAngle, Angle upperAngle) {
    SmartMotorController lower = getLowerMotorController();
    SmartMotorController upper = getUpperMotorController();
    return mechanism.run(coroutine -> {
      while (true) {
        if (lowerAngle != null) {
          lower.setPosition(lowerAngle);
        }
        if (upperAngle != null) {
          upper.setPosition(upperAngle);
        }
        coroutine.yield();
      }
    }).named(mechanism.getName() + " SetAngle");
  }

  /**
   * Simple duty cycle command.
   *
   * @param lowerDutycycle Dutycycle of the shoulder.
   * @param upperDutycycle DutyCycle of the elbow.
   * @return {@link Command} to set the DutyCycle.
   */
  public Command set(Double lowerDutycycle, Double upperDutycycle) {
    SmartMotorController lower = getLowerMotorController();
    SmartMotorController upper = getUpperMotorController();
    return mechanism.run(coroutine -> {
      if (lowerDutycycle != null) {
        lower.stopClosedLoopController();
      }
      if (upperDutycycle != null) {
        upper.stopClosedLoopController();
      }
      while (true) {
        if (lowerDutycycle != null) {
          lower.setDutyCycle(lowerDutycycle);
        }
        if (upperDutycycle != null) {
          upper.setDutyCycle(upperDutycycle);
        }
        coroutine.yield();
      }
    }).whenCanceled(() -> {
      if (lowerDutycycle != null) {
        lower.startClosedLoopController();
      }
      if (upperDutycycle != null) {
        upper.startClosedLoopController();
      }
    }).named(mechanism.getName() + " SetDutyCycle");
  }
}
