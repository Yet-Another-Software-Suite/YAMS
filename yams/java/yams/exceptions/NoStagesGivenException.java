// Copyright (c) 2026 YAMS Contributors
// SPDX-License-Identifier: LGPL-3.0-or-later

package yams.exceptions;

/**
 * Exception for math errors when trying to find the sensor to mechanism ratio.
 */
public class NoStagesGivenException extends RuntimeException
{
  /**
   * Constructs exception for failure to provide stages.
   */
  public NoStagesGivenException()
  {
    super("No stages given!");
  }
}
