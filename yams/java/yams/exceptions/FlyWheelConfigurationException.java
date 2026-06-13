// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package yams.exceptions;

/**
 * Thrown by {@link yams.mechanisms.velocity.FlyWheel} or {@link yams.mechanisms.config.FlyWheelConfig}
 * when a required field in {@link yams.mechanisms.config.FlyWheelConfig} is missing or invalid.
 *
 * <p>Common triggers include:
 * <ul>
 *   <li>{@code withSmartMotorController(SmartMotorController)} called more than once on the same config</li>
 *   <li>Speedometer max velocity not set when speedometer simulation is requested
 *       ({@code withSpeedometerSimulation(AngularVelocity)})</li>
 *   <li>Flywheel diameter not set when surface speed calculations are needed
 *       ({@code withDiameter(Distance)})</li>
 *   <li>MOI (moment of inertia) not derivable — neither diameter+mass nor an explicit MOI was provided</li>
 * </ul>
 *
 * <p><b>Resolution:</b> Ensure {@code FlyWheelConfig.withSmartMotorController(SmartMotorController)}
 * is called exactly once, {@code FlyWheelConfig.withDiameter(Distance)} and
 * {@code FlyWheelConfig.withMass(Mass)} are set (or provide an explicit MOI via
 * {@code FlyWheelConfig.withMOI()}), and — when using speedometer simulation —
 * {@code FlyWheelConfig.withSpeedometerSimulation(AngularVelocity)} is called with a nonzero max velocity.
 *
 * <p>Example minimal configuration:
 * <pre>{@code
 * FlyWheelConfig config = new FlyWheelConfig()
 *     .withSmartMotorController(SmartMotorController.create(SPARK_MAX, DCMotor.getNEO(1)))
 *     .withDiameter(Inches.of(4))
 *     .withMass(Pounds.of(0.5));
 * FlyWheel flyWheel = new FlyWheel(config);
 * }</pre>
 *
 * @see yams.mechanisms.velocity.FlyWheel
 * @see yams.mechanisms.config.FlyWheelConfig
 */
public class FlyWheelConfigurationException extends RuntimeException
{
  /**
   * FlyWheel configuration exception.
   *
   * @param message        Message to display.
   * @param result         Result of the configuration.
   * @param remedyFunction Remedy function to use.
   */
  public FlyWheelConfigurationException(String message, String result, String remedyFunction)
  {
    super(message + "!\n" + result + "\nPlease use FlyWheelConfig." + remedyFunction + " to fix this error.");
  }
}
