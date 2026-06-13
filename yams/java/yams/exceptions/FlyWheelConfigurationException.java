// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package yams.exceptions;

/**
 * Exception for when the FlyWheel is configured incorrectly.
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
