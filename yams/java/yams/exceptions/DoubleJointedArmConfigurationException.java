// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package yams.exceptions;

/**
 * Thrown by {@link yams.mechanisms.positional.DoubleJointedArm} when a required field in the
 * lower-joint or upper-joint {@link yams.mechanisms.config.ArmConfig} is missing or invalid.
 *
 * <p>Common triggers include:
 * <ul>
 *   <li>The lower and upper joint motor controllers do not share the same WPILib
 *       {@code Subsystem} instance — both must reference the same subsystem so that a
 *       single command can require it</li>
 *   <li>Starting angle missing on either joint config — both the lower and upper
 *       {@code ArmConfig} must have a starting position set via
 *       {@code SmartMotorControllerConfig.withStartingPosition(Angle)}</li>
 * </ul>
 *
 * <p><b>Resolution:</b> Ensure both joint configs supply motors via
 * {@code ArmConfig.withSmartMotorController(SmartMotorController)}, that both motors
 * are bound to the same subsystem via {@code SmartMotorControllerConfig.withSubsystem(Subsystem)},
 * and that both configs include {@code SmartMotorControllerConfig.withStartingPosition(Angle)}.
 *
 * <p>Example minimal configuration:
 * <pre>{@code
 * ArmConfig lowerConfig = new ArmConfig()
 *     .withSmartMotorController(SmartMotorController.create(SPARK_MAX, DCMotor.getNEO(1), lowerMotorCfg));
 * ArmConfig upperConfig = new ArmConfig()
 *     .withSmartMotorController(SmartMotorController.create(SPARK_MAX, DCMotor.getNEO(1), upperMotorCfg));
 * DoubleJointedArm arm = new DoubleJointedArm(lowerConfig, upperConfig);
 * }</pre>
 *
 * @see yams.mechanisms.positional.DoubleJointedArm
 * @see yams.mechanisms.config.ArmConfig
 */
public class DoubleJointedArmConfigurationException extends RuntimeException
{
  /**
   * Arm configuration exception.
   *
   * @param message        Message to display.
   * @param result         Result of the configuration.
   * @param remedyFunction Remedy function to use.
   */
  public DoubleJointedArmConfigurationException(String message, String result, String remedyFunction)
  {
    super(message + "!\n" + result + "\nPlease use ArmConfig." + remedyFunction + " to fix this error.");
  }
}
