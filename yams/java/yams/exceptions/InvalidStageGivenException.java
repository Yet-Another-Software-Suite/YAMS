// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package yams.exceptions;

/**
 * Thrown by {@link yams.gearing.GearBox} and {@link yams.gearing.Sprocket} when a gear stage
 * string is not in the required {@code "IN:OUT"} format.
 *
 * <p>Each stage passed to a {@code GearBox} or {@code Sprocket} constructor must be a
 * {@code String} containing exactly one colon ({@code :}) separating the driver tooth count
 * from the driven tooth count (e.g., {@code "12:60"}).  A stage that is missing the colon
 * separator, is empty, or cannot be parsed triggers this exception.
 *
 * <p><b>Resolution:</b> Correct the malformed stage string so it follows the
 * {@code "driverTeeth:drivenTeeth"} pattern.
 *
 * <p>Example correct usage:
 * <pre>{@code
 * // WRONG — missing colon separator
 * GearBox bad = new GearBox("1260");  // throws InvalidStageGivenException
 *
 * // CORRECT
 * GearBox good = new GearBox("12:60");
 * }</pre>
 *
 * @see yams.gearing.GearBox
 * @see yams.gearing.Sprocket
 * @see NoStagesGivenException
 */
public class InvalidStageGivenException extends RuntimeException
{
  /**
   * Constructs exception for failure to provide stages.
   *
   * @param stage Stage given.
   */
  public InvalidStageGivenException(String stage)
  {
    super("Invalid stage given! '" + stage + "'; should be in the format of 'IN:OUT'!");
  }
}
