// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package yams.exceptions;

/**
 * Thrown by {@link yams.gearing.GearBox}, {@link yams.gearing.Sprocket}, and
 * {@link yams.math.SmartMath} when a gear reduction calculation is requested but
 * no gear stages were supplied.
 *
 * <p>At least one reduction stage (in the {@code "IN:OUT"} string format) is required
 * for the ratio calculation to be mathematically valid.  Passing an empty stage array
 * or constructing a {@code GearBox}/{@code Sprocket} with zero stages triggers this exception.
 *
 * <p><b>Resolution:</b> Provide one or more reduction stages. Each stage must be a
 * {@code String} in the form {@code "driverTeeth:drivenTeeth"} (e.g., {@code "12:60"}).
 *
 * <p>Example correct usage:
 * <pre>{@code
 * // Single-stage 5:1 reduction
 * GearBox gearBox = new GearBox("12:60");
 *
 * // Two-stage reduction
 * GearBox gearBox = new GearBox("12:60", "18:36");
 * }</pre>
 *
 * @see yams.gearing.GearBox
 * @see yams.gearing.Sprocket
 * @see yams.math.SmartMath
 * @see InvalidStageGivenException
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
