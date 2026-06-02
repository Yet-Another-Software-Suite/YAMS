// Copyright (c) 2026 YAMS Contributors
// SPDX-License-Identifier: LGPL-3.0-or-later

package yams.exceptions;

import yams.mechanisms.SmartMechanism;

/**
 * Custom exception for when there is no motor in the mechanism.
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
