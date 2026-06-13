// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package yams.exceptions;

import yams.mechanisms.SmartMechanism;

/**
 * Thrown by a {@link yams.mechanisms.SmartMechanism} subclass when the primary
 * {@link yams.motorcontrollers.SmartMotorController} has not been set before the mechanism
 * attempts to use it.
 *
 * <p>This exception signals that the mechanism's motor slot is {@code null} — either because
 * no motor was ever assigned, or because the mechanism was constructed before the motor
 * was initialized.
 *
 * <p>Common triggers include:
 * <ul>
 *   <li>Constructing or activating a mechanism without calling the appropriate
 *       {@code withSmartMotorController()} builder method on its config</li>
 *   <li>Passing {@code null} as the motor argument to {@code setMotor()} or the config builder</li>
 * </ul>
 *
 * <p><b>Resolution:</b> Provide a valid motor controller before constructing the mechanism:
 * <pre>{@code
 * SmartMotorController motor = SmartMotorController.create(
 *     SPARK_MAX, DCMotor.getNEO(1));
 * // Then pass the motor to the relevant mechanism config, e.g.:
 * ArmConfig config = new ArmConfig().withSmartMotorController(motor);
 * }</pre>
 *
 * @see yams.mechanisms.SmartMechanism
 * @see yams.motorcontrollers.SmartMotorController
 */
public class MotorNotPresentException extends RuntimeException
{
  /**
   * Create {@link RuntimeException} for {@link SmartMechanism}
   *
   * @param mechanismType Name of the mechanism
   */
  public MotorNotPresentException(String mechanismType)
  {
    super(mechanismType +
          " primary motor not present! Please set one using `setMotor(SmartMotorController.create(MOTOR_CONTROLLER, DCMotor.getNEO(1))`");
  }
}
