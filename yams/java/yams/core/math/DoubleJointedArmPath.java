// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package yams.core.math;

import static org.wpilib.units.Units.Radians;

import org.wpilib.units.measure.Angle;

/**
 * Joint angles for a double jointed arm to move through in order, ending at its goal, planned by
 * {@link DoubleJointedArmPathPlanner}. Both angles are from horizontal.
 */
public final class DoubleJointedArmPath {
  /** Lower arm angle of each waypoint, in radians. */
  private final double[] lower;
  /** Upper arm angle of each waypoint, in radians. */
  private final double[] upper;

  /**
   * Create a path from the waypoints' angles. The arrays are owned by the path from then on.
   *
   * @param lower Lower arm angle of each waypoint, in radians.
   * @param upper Upper arm angle of each waypoint, in radians.
   * @throws IllegalArgumentException if the arrays differ in length or are empty.
   */
  DoubleJointedArmPath(double[] lower, double[] upper) {
    if (lower.length != upper.length || lower.length == 0) {
      throw new IllegalArgumentException("A path needs the same number of lower and upper angles, and at least one");
    }
    this.lower = lower;
    this.upper = upper;
  }

  /**
   * A path straight to a goal, with the goal as its only waypoint.
   *
   * @param lowerAngle Goal lower arm angle.
   * @param upperAngle Goal upper arm angle.
   * @return {@link DoubleJointedArmPath} to the goal.
   */
  public static DoubleJointedArmPath to(Angle lowerAngle, Angle upperAngle) {
    return new DoubleJointedArmPath(new double[]{lowerAngle.in(Radians)}, new double[]{upperAngle.in(Radians)});
  }

  /**
   * Get the number of waypoints.
   *
   * @return Number of waypoints, at least 1.
   */
  public int size() {
    return lower.length;
  }

  /**
   * Get a waypoint's lower arm angle.
   *
   * @param index Waypoint index.
   * @return Lower arm angle, in radians.
   */
  public double getLowerRadians(int index) {
    return lower[index];
  }

  /**
   * Get a waypoint's upper arm angle.
   *
   * @param index Waypoint index.
   * @return Upper arm angle, in radians.
   */
  public double getUpperRadians(int index) {
    return upper[index];
  }

  /**
   * Get a waypoint's lower arm angle.
   *
   * @param index Waypoint index.
   * @return Lower arm {@link Angle}.
   */
  public Angle getLowerAngle(int index) {
    return Radians.of(lower[index]);
  }

  /**
   * Get a waypoint's upper arm angle.
   *
   * @param index Waypoint index.
   * @return Upper arm {@link Angle}.
   */
  public Angle getUpperAngle(int index) {
    return Radians.of(upper[index]);
  }
}
