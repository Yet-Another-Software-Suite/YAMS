// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package yams.commands3.mechanisms;

import java.util.NoSuchElementException;
import java.util.function.Supplier;
import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;
import org.wpilib.units.measure.Angle;
import yams.core.exceptions.DifferentialMechanismConfigurationException;
import yams.core.exceptions.SmartMotorControllerConfigurationException;
import yams.core.mechanisms.config.DifferentialMechanismConfig;
import yams.core.motorcontrollers.SmartMotorController;

/**
 * Command-based extension of {@link yams.core.mechanisms.positional.DifferentialMechanism} that
 * adds the {@link Mechanism} binding and {@link Command} factories.
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
public class DifferentialMechanism extends yams.core.mechanisms.positional.DifferentialMechanism implements CommandMechanism {
  /** Mechanism the mechanism's commands should require. */
  private final Mechanism mechanism;

  /**
   * Constructor for the Differential mechanism.
   *
   * @param diffConfig Lower {@link DifferentialMechanismConfig} to use.
   * @implNote Both motor controllers' configs must be {@link yams.commands3.config.SmartMotorControllerConfig}s that share the same
   *           {@link Mechanism} set via {@code withMechanism(Mechanism)}.
   * @throws DifferentialMechanismConfigurationException if the starting tilt or twist angle is not
   *                                                     configured.
   * @throws DifferentialMechanismConfigurationException if the length is not configured.
   * @throws DifferentialMechanismConfigurationException if running in simulation and the MOI is not
   *                                                     configured.
   * @throws DifferentialMechanismConfigurationException if the left and right motor controllers'
   *                                                     configs do not share the same
   *                                                     {@link Mechanism}.
   * @throws NoSuchElementException if the left or right {@link SmartMotorController} was never set
   *                                on {@code diffConfig}.
   * @throws SmartMotorControllerConfigurationException if either motor controller's config does not
   *                                                    have a {@link Mechanism} set via
   *                                                    {@code withMechanism(Mechanism)}.
   */
  public DifferentialMechanism(DifferentialMechanismConfig diffConfig) {
    super(diffConfig);
    yams.commands3.config.SmartMotorControllerConfig leftConfig = (yams.commands3.config.SmartMotorControllerConfig) getLeftMotorController().getConfig();
    yams.commands3.config.SmartMotorControllerConfig rightConfig = (yams.commands3.config.SmartMotorControllerConfig) getRightMotorController().getConfig();
    if (leftConfig.getMechanism() != rightConfig.getMechanism()) {
      throw new DifferentialMechanismConfigurationException("SmartMotorControllers do not have the same mechanism!", "Cannot create commands for single mechanism.", "withMechanism(this)");
    }
    this.mechanism = leftConfig.getMechanism();
  }

  @Override
  public Mechanism getMechanism() {
    return mechanism;
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
    return mechanism.run(coroutine -> {
      while (true) {
        left.setPosition(config.getLeftMechanismPosition(tilt.get(), twist.get()));
        right.setPosition(config.getRightMechanismPosition(tilt.get(), twist.get()));
        coroutine.yield();
      }
    }).named(getName() + " set position");
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
    return mechanism.run(coroutine -> {
      left.stopClosedLoopController();
      right.stopClosedLoopController();
      while (true) {
        left.setDutyCycle(tilt - twist);
        right.setDutyCycle(tilt + twist);
        coroutine.yield();
      }
    }).whenCanceled(() -> {
      left.startClosedLoopController();
      right.startClosedLoopController();
    }).named(getName() + " set dutycycle");
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
    return mechanism.run(coroutine -> {
      while (true) {
        left.setPosition(config.getLeftMechanismPosition(tilt, twist));
        right.setPosition(config.getRightMechanismPosition(tilt, twist));
        coroutine.yield();
      }
    }).named(getName() + " set position");
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
}
