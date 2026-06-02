// Copyright (c) 2026 YAMS Contributors
// SPDX-License-Identifier: LGPL-3.0-or-later

package yams.exceptions;

/**
 * Exception for when the SmartMotorController is configured incorrectly.
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
