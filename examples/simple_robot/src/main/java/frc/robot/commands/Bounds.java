// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package frc.robot.commands;


import org.wpilib.math.MathUtil;
import org.wpilib.math.geometry.Translation2d;

public record Bounds(double minX, double maxX, double minY, double maxY)
{
  /**
   * Whether the translation is contained within the bounds.
   */
  public boolean contains(Translation2d translation)
  {
    return translation.getX() >= minX()
           && translation.getX() <= maxX()
           && translation.getY() >= minY()
           && translation.getY() <= maxY();
  }

  /**
   * Clamps the translation to the bounds.
   */
  public Translation2d clamp(Translation2d translation)
  {
    return new Translation2d(
        MathUtil.clamp(translation.getX(), minX(), maxX()),
        MathUtil.clamp(translation.getY(), minY(), maxY()));
  }
}
