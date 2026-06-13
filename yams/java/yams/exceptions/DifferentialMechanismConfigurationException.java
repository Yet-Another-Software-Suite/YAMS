// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package yams.exceptions;

/**
 * Thrown by {@link yams.mechanisms.positional.DifferentialMechanism} or
 * {@link yams.mechanisms.config.DifferentialMechanismConfig} when a required field is missing
 * or an attempt is made to violate a config invariant.
 *
 * <p>Common triggers include:
 * <ul>
 *   <li>{@code withSmartMotorControllers(SmartMotorController, SmartMotorController)} called more
 *       than once — the left or right motor controller has already been set</li>
 *   <li>The left and right motor controllers do not share the same WPILib {@code Subsystem} instance</li>
 *   <li>Starting tilt or twist angle not configured ({@code withTiltStartingPosition(Angle)} /
 *       {@code withTwistStartingPosition(Angle)})</li>
 *   <li>Mechanism length not set ({@code withLength(Distance)}) — required to compute
 *       the current end-effector position</li>
 *   <li>Twist MOI not set ({@code withMOI()}) — required for simulation</li>
 * </ul>
 *
 * <p><b>Resolution:</b> Call {@code DifferentialMechanismConfig.withSmartMotorControllers(left, right)}
 * exactly once, ensure both motors reference the same subsystem via
 * {@code SmartMotorControllerConfig.withSubsystem(Subsystem)}, and provide all required geometry
 * and simulation parameters.
 *
 * <p>Example minimal configuration:
 * <pre>{@code
 * DifferentialMechanismConfig config = new DifferentialMechanismConfig()
 *     .withSmartMotorControllers(leftMotor, rightMotor)
 *     .withLength(Inches.of(20))
 *     .withTiltStartingPosition(Degrees.of(0))
 *     .withTwistStartingPosition(Degrees.of(0));
 * DifferentialMechanism mech = new DifferentialMechanism(config);
 * }</pre>
 *
 * @see yams.mechanisms.positional.DifferentialMechanism
 * @see yams.mechanisms.config.DifferentialMechanismConfig
 */
public class DifferentialMechanismConfigurationException extends RuntimeException
{
  /**
   * Differential Mechanism configuration exception.
   *
   * @param message        Message to display.
   * @param result         Result of the configuration.
   * @param remedyFunction Remedy function to use.
   */
  public DifferentialMechanismConfigurationException(String message, String result, String remedyFunction)
  {
    super(message + "!\n" + result + "\nPlease use DifferentialMechanism." + remedyFunction + " to fix this error.");
  }
}
