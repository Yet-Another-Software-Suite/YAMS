// Copyright (c) 2026 YAMS Contributors
// SPDX-License-Identifier: LGPL-3.0-or-later

package yams.exceptions;

/**
 * Exception for when the Pivot is configured incorrectly.
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
