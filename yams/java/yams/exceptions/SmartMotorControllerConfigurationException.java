// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package yams.exceptions;

/**
 * Thrown by {@link yams.motorcontrollers.SmartMotorControllerConfig} or a
 * {@link yams.motorcontrollers.SmartMotorController} wrapper when the applied configuration
 * contains conflicting, incomplete, or incompatible settings.
 *
 * <p>Common triggers include:
 * <ul>
 *   <li>{@code withSubsystem(Subsystem)} called more than once on the same config</li>
 *   <li>Auto-synchronization threshold set while a distance-based (linear) mechanism circumference
 *       is configured — the two options are mutually exclusive</li>
 *   <li>Mechanism circumference undefined when a conversion method (e.g.,
 *       {@code withZeroOffset()}, {@code withSoftLimits()}, or linear unit conversions)
 *       requires it — fix with {@code withMechanismCircumference(Distance)}</li>
 *   <li>External encoder discontinuity point outside the allowed values (must be
 *       {@code Rotations.of(0.5)} or {@code Rotations.of(1)})</li>
 *   <li>Continuous wrapping requested while soft limits are also set, or while a
 *       distance-based (linear) mechanism circumference is configured</li>
 *   <li>Continuous wrapping or closed-loop tolerance requested without a PID controller
 *       ({@code withClosedLoopController()})</li>
 *   <li>Distance-based tolerance setter called on an angular controller (missing
 *       {@code withLinearClosedLoopController(true)})</li>
 *   <li>MOI parameters ({@code withMOI(Distance, Mass)}) called with {@code null} length or mass</li>
 *   <li>NEO 550 motor used without a stator current limit, or with a limit exceeding 40 A</li>
 *   <li>Vendor-specific control request type incompatible with the underlying motor controller
 *       (e.g., a TalonFXS control request not supported by the configured motor type)</li>
 *   <li>Required basic or external-encoder options not all provided before calling {@code get()}</li>
 *   <li>Live tuning requested while the control mode is {@code OPEN_LOOP} — requires
 *       {@code withControlMode(ControlMode.CLOSED_LOOP)}</li>
 * </ul>
 *
 * <p><b>Resolution:</b> Read the exception message; it always names the specific
 * {@code SmartMotorControllerConfig} method to call (or to remove) to resolve the conflict.
 *
 * <p>Example triggering continuous-wrapping conflict:
 * <pre>{@code
 * // WRONG — soft limits and continuous wrapping cannot coexist
 * SmartMotorControllerConfig config = new SmartMotorControllerConfig()
 *     .withSoftLimits(Degrees.of(-90), Degrees.of(90))
 *     .withContinuousWrapping(Rotations.of(0), Rotations.of(1)); // throws
 *
 * // CORRECT — remove withSoftLimits() when using continuous wrapping
 * SmartMotorControllerConfig config = new SmartMotorControllerConfig()
 *     .withContinuousWrapping(Rotations.of(0), Rotations.of(1));
 * }</pre>
 *
 * @see yams.motorcontrollers.SmartMotorControllerConfig
 * @see yams.motorcontrollers.SmartMotorController
 */
public class SmartMotorControllerConfigurationException extends RuntimeException
{
  /**
   * SmartMotorControllerConfigurationException constructor.
   *
   * @param message        Message to display.
   * @param result         Result of the configuration.
   * @param remedyFunction Remedy function to use.
   */
  public SmartMotorControllerConfigurationException(String message, String result, String remedyFunction)
  {
    super(
        message + "!\n" + result + "\nPlease use SmartMotorControllerConfig." + remedyFunction + " to fix this error.");
  }
}
