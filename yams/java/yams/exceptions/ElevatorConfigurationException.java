// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package yams.exceptions;

/**
 * Thrown by {@link yams.mechanisms.positional.Elevator} when a required
 * {@link yams.mechanisms.config.ElevatorConfig} field is missing or invalid.
 *
 * <p>Common triggers include:
 * <ul>
 *   <li>Carriage mass not set ({@code withMass(Mass)}) — required for simulation</li>
 *   <li>Hard limits ({@code withHardLimits(Distance, Distance)}) missing minimum or maximum height</li>
 *   <li>Starting height not configured ({@code withStartingHeight(Distance)})</li>
 *   <li>No motor controller provided</li>
 * </ul>
 *
 * <p><b>Resolution:</b> Ensure {@code ElevatorConfig.withSmartMotorController(SmartMotorController)}
 * is called, {@code ElevatorConfig.withHardLimits(Distance, Distance)} defines both the minimum
 * and maximum travel distances, {@code ElevatorConfig.withMass(Mass)} provides the carriage mass,
 * and {@code ElevatorConfig.withStartingHeight(Distance)} sets the initial position.
 *
 * <p>Example minimal configuration:
 * <pre>{@code
 * ElevatorConfig config = new ElevatorConfig()
 *     .withSmartMotorController(SmartMotorController.create(SPARK_MAX, DCMotor.getNEO(1)))
 *     .withHardLimits(Inches.of(0), Inches.of(48))
 *     .withMass(Pounds.of(10))
 *     .withStartingHeight(Inches.of(0));
 * Elevator elevator = new Elevator(config);
 * }</pre>
 *
 * @see yams.mechanisms.positional.Elevator
 * @see yams.mechanisms.config.ElevatorConfig
 */
public class ElevatorConfigurationException extends RuntimeException
{
  /**
   * Elevator configuration exception.
   *
   * @param message        Message to display.
   * @param result         Result of the configuration.
   * @param remedyFunction Remedy function to use.
   */
  public ElevatorConfigurationException(String message, String result, String remedyFunction)
  {
    super(message + "!\n" + result + "\nPlease use ElevatorConfig." + remedyFunction + " to fix this error.");
  }
}
