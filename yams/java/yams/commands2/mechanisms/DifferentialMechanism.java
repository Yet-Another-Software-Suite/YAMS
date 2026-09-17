// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package yams.commands2.mechanisms;

import java.util.function.Supplier;
import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import org.wpilib.command2.Subsystem;
import org.wpilib.command2.button.Trigger;
import org.wpilib.units.measure.Angle;
import yams.core.exceptions.DifferentialMechanismConfigurationException;
import yams.core.mechanisms.config.DifferentialMechanismConfig;
import yams.core.motorcontrollers.SmartMotorController;

/**
 * Command-based extension of {@link yams.core.mechanisms.positional.DifferentialMechanism} that
 * adds the {@link Subsystem} binding and {@link Command}/{@link Trigger} factories.
 *
 * <h2>Usage Example</h2>
 * <pre>{@code
 * DifferentialMechanismConfig config = new DifferentialMechanismConfig(leftSMC, rightSMC)
 *     .withStartingPosition(Degrees.of(30), Degrees.of(0))  // tilt=30°, twist=0°
 *     .withLength(Inches.of(14))
 *     .withMOI(Inches.of(14), Pounds.of(2))
 *     .withTelemetry("DifferentialWrist", TelemetryVerbosity.HIGH);
 *
 * DifferentialMechanism wrist = new DifferentialMechanism(config);
 *
 * // Command: tilt to 60° and twist to 45°.
 * Command scoreCommand = wrist.setPosition(Degrees.of(60), Degrees.of(45));
 *
 * // Command: continuously follow a joystick (suppliers).
 * Command manualCommand = wrist.run(
 *     () -> Degrees.of(driverController.getLeftY() * 90),
 *     () -> Degrees.of(driverController.getRightX() * 90));
 * }</pre>
 */
public class DifferentialMechanism extends yams.core.mechanisms.positional.DifferentialMechanism
    implements CommandMechanism {
  /** Subsystem the mechanism's commands should require. */
  private final Subsystem subsystem;

  /**
   * Constructor for the Differential mechanism.
   *
   * @param diffConfig Lower {@link DifferentialMechanismConfig} to use.
   * @implNote Both motor controllers' configs must be {@link
   *           yams.commands2.config.SmartMotorControllerConfig}s that share the same
   *           {@link Subsystem} set via {@code withSubsystem(Subsystem)}.
   */
  public DifferentialMechanism(DifferentialMechanismConfig diffConfig) {
    super(diffConfig);
    yams.commands2.config.SmartMotorControllerConfig leftConfig =
        (yams.commands2.config.SmartMotorControllerConfig) getLeftMotorController().getConfig();
    yams.commands2.config.SmartMotorControllerConfig rightConfig =
        (yams.commands2.config.SmartMotorControllerConfig) getRightMotorController().getConfig();
    if (leftConfig.getSubsystem() != rightConfig.getSubsystem()) {
      throw new DifferentialMechanismConfigurationException(
          "SmartMotorControllers do not have the same subsystem!",
          "Cannot create commands for single subsystem.", "withSubsystem(this)");
    }
    this.subsystem = leftConfig.getSubsystem();
  }

  @Override
  public Subsystem getSubsystem() {
    return subsystem;
  }

  /**
   * Set the position of the differential mechanism using suppliers.
   *
   * @param tilt  Tilt of the Differential Mechanism.
   * @param twist Twist of the Differential Mechanism.
   * @return {@link Command} to set the position.
   */
  public Command setPosition(Supplier<Angle> tilt, Supplier<Angle> twist) {
    SmartMotorController left = getLeftMotorController();
    SmartMotorController right = getRightMotorController();
    var config = getDifferentialMechanismConfig();
    return Commands
        .run(
            ()
                -> {
              left.setPosition(config.getLeftMechanismPosition(tilt.get(), twist.get()));
              right.setPosition(config.getRightMechanismPosition(tilt.get(), twist.get()));
            },
            subsystem)
        .withName(getName() + " set position");
  }

  /**
   * Set the dutycycle of the differential mechanism.
   *
   * @param twist Twist dutycycle.
   * @param tilt  Tilt dutycycle.
   * @return {@link Command} to set the differential mechanism duty cycle.
   */
  public Command set(double twist, double tilt) {
    SmartMotorController left = getLeftMotorController();
    SmartMotorController right = getRightMotorController();
    return Commands
        .startRun(
            ()
                -> {
              left.stopClosedLoopController();
              right.stopClosedLoopController();
            },
            ()
                -> {
              left.setDutyCycle(tilt - twist);
              right.setDutyCycle(tilt + twist);
            },
            subsystem)
        .finallyDo(() -> {
          left.startClosedLoopController();
          right.startClosedLoopController();
        })
        .withName(getName() + " set dutycycle");
  }

  /**
   * Set the position of the differential mechanism.
   *
   * @param tilt  Tilt of the differential mechanism.
   * @param twist Twist of the differential mechanism.
   * @return {@link Command} to set the position.
   */
  public Command setPosition(Angle tilt, Angle twist) {
    SmartMotorController left = getLeftMotorController();
    SmartMotorController right = getRightMotorController();
    var config = getDifferentialMechanismConfig();
    return Commands
        .run(
            ()
                -> {
              left.setPosition(config.getLeftMechanismPosition(tilt, twist));
              right.setPosition(config.getRightMechanismPosition(tilt, twist));
            },
            subsystem)
        .withName(getName() + " set position");
  }

  /**
   * Set the position of the differential mechanism.
   *
   * @param tilt  Tilt of the differential mechanism.
   * @param twist Twist of the differential mechanism.
   * @return {@link Command} to set the position.
   */
  public Command run(Angle tilt, Angle twist) {
    return setPosition(tilt, twist);
  }

  /**
   * Run the differential mechanism to a position.
   *
   * @param tilt  Supplier of the tilt angle.
   * @param twist Supplier of the twist angle.
   * @return {@link Command} to run the differential mechanism to the position.
   */
  public Command run(Supplier<Angle> tilt, Supplier<Angle> twist) {
    return setPosition(tilt, twist);
  }

  /**
   * {@link yams.core.mechanisms.positional.DifferentialMechanism} does not support positional
   * limits.
   *
   * @return Never returns.
   * @throws RuntimeException Always.
   */
  public Trigger max() {
    throw new RuntimeException("Unsupported operation");
  }

  /**
   * {@link yams.core.mechanisms.positional.DifferentialMechanism} does not support positional
   * limits.
   *
   * @return Never returns.
   * @throws RuntimeException Always.
   */
  public Trigger min() {
    throw new RuntimeException("Unsupported operation");
  }
}
