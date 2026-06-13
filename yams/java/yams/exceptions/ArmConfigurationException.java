// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package yams.exceptions;

/**
 * Thrown by {@link yams.mechanisms.positional.Arm} when a required
 * {@link yams.mechanisms.config.ArmConfig} field is missing or invalid.
 *
 * <p>Common triggers include:
 * <ul>
 *   <li>No motor controller provided (motor is absent from the config)</li>
 *   <li>Arm length not set (needed for simulation)</li>
 *   <li>Hard limits ({@code withHardLimits(Angle, Angle)}) missing or inconsistent</li>
 *   <li>Starting angle not configured when no zero-offset is present on the motor</li>
 *   <li>MOI (moment of inertia) not derivable from length/mass and not explicitly set</li>
 *   <li>{@code withSmartMotorController()} called twice on the same config object</li>
 * </ul>
 *
 * <p><b>Resolution:</b> Ensure {@code ArmConfig.withSmartMotorController(SmartMotorController)}
 * is called exactly once, that {@code ArmConfig.withHardLimits(Angle, Angle)} provides valid
 * lower and upper bounds, that {@code ArmConfig.withLength(Distance)} is set for simulation,
 * and that a starting angle is supplied via
 * {@code SmartMotorControllerConfig.withStartingPosition(Angle)} or an encoder zero-offset.
 *
 * <p>Example minimal configuration:
 * <pre>{@code
 * ArmConfig config = new ArmConfig()
 *     .withSmartMotorController(SmartMotorController.create(SPARK_MAX, DCMotor.getNEO(1)))
 *     .withHardLimits(Degrees.of(-90), Degrees.of(90))
 *     .withLength(Inches.of(18));
 * Arm arm = new Arm(config);
 * }</pre>
 *
 * @see yams.mechanisms.positional.Arm
 * @see yams.mechanisms.config.ArmConfig
 */
public class ArmConfigurationException extends RuntimeException
{
  /**
   * Arm configuration exception.
   *
   * @param message        Message to display.
   * @param result         Result of the configuration.
   * @param remedyFunction Remedy function to use.
   */
  public ArmConfigurationException(String message, String result, String remedyFunction)
  {
    super(message + "!\n" + result + "\nPlease use ArmConfig." + remedyFunction + " to fix this error.");
  }
}
