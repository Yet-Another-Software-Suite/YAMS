// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package yams.exceptions;

/**
 * Thrown by {@link yams.mechanisms.positional.Pivot} or {@link yams.mechanisms.config.PivotConfig}
 * when a required field in {@link yams.mechanisms.config.PivotConfig} is missing or invalid.
 *
 * <p>Common triggers include:
 * <ul>
 *   <li>{@code withSmartMotorController(SmartMotorController)} called more than once on the same config</li>
 *   <li>MOI (moment of inertia) not derivable — neither length+mass nor an explicit MOI was configured
 *       on the underlying {@link yams.motorcontrollers.SmartMotorControllerConfig}</li>
 * </ul>
 *
 * <p><b>Resolution:</b> Ensure {@code PivotConfig.withSmartMotorController(SmartMotorController)}
 * is called exactly once, and that the motor's {@code SmartMotorControllerConfig} includes
 * {@code withMomentOfInertia(Distance, Mass)} or {@code withMomentOfInertia(MomentOfInertia)}
 * when simulation is used.
 *
 * <p>Example minimal configuration:
 * <pre>{@code
 * SmartMotorControllerConfig motorCfg = new SmartMotorControllerConfig()
 *     .withMomentOfInertia(Inches.of(10), Pounds.of(2));
 * PivotConfig config = new PivotConfig()
 *     .withSmartMotorController(SmartMotorController.create(SPARK_MAX, DCMotor.getNEO(1), motorCfg));
 * Pivot pivot = new Pivot(config);
 * }</pre>
 *
 * @see yams.mechanisms.positional.Pivot
 * @see yams.mechanisms.config.PivotConfig
 */
public class PivotConfigurationException extends RuntimeException
{
  /**
   * Pivot configuration exception.
   *
   * @param message        Message to display.
   * @param result         Result of the configuration.
   * @param remedyFunction Remedy function to use.
   */
  public PivotConfigurationException(String message, String result, String remedyFunction)
  {
    super(message + "!\n" + result + "\nPlease use PivotConfig." + remedyFunction + " to fix this error.");
  }
}
