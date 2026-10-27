// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package yams.core.mechanisms.positional;

import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Radians;

import java.util.Optional;
import org.wpilib.driverstation.internal.DriverStationBackend;
import org.wpilib.units.measure.Angle;
import yams.core.math.DoubleJointedArmPath;

/**
 * Moves a {@link DoubleJointedArm} to a goal along a path that stays out of its exclusion zones, one waypoint at a time.
 * Each arm has one, from {@link DoubleJointedArm#getPathFollower()}, which every command moving the arm shares by
 * giving it a new goal as it starts.
 *
 * <p>Each loop {@link #execute()} compensates each joint's kG for where the arm is (see
 * {@link DoubleJointedArm#updateKg()}), and gives both joints' closed loop controllers the current waypoint. It moves on
 * to the next waypoint once both joints are within the path tolerance of the current one, or as soon as the arm can
 * move straight from where it is to the next one and stay clear of the zones. So the arm also moves on when gravity
 * holds a joint further from a waypoint than the tolerance, and skips waypoints it has a clear way past. Without
 * exclusion zones the goal is the only waypoint.
 *
 * <p>When the goal can't be reached without entering an exclusion zone, or is in one, the arm holds where it was and a
 * warning is reported.
 */
public class DoubleJointedArmPathFollower {
  /** Arm to move. */
  private final DoubleJointedArm         arm;
  /** Path being followed, or empty while holding. */
  private Optional<DoubleJointedArmPath> path      = Optional.empty();
  /** Waypoint being moved to. */
  private int                            index     = 0;
  /** Goal lower joint angle, in radians. */
  private double                         goalLower = Double.NaN;
  /** Goal upper joint angle, in radians. */
  private double                         goalUpper = Double.NaN;
  /** Lower joint angle held when there's no path, in radians. */
  private double                         holdLower = Double.NaN;
  /** Upper joint angle held when there's no path, in radians. */
  private double                         holdUpper = Double.NaN;

  /**
   * Create the path follower of an arm. Each arm has one, from {@link DoubleJointedArm#getPathFollower()}.
   *
   * @param arm {@link DoubleJointedArm} to move.
   */
  DoubleJointedArmPathFollower(DoubleJointedArm arm) {
    this.arm = arm;
  }

  /**
   * Plan a path from where the arm is to a goal, and start following it.
   *
   * @param lowerAngle Goal lower joint angle.
   * @param upperAngle Goal upper joint angle.
   * @return True if a path was found, false if the arm holds where it is instead.
   */
  public boolean setGoal(Angle lowerAngle, Angle upperAngle) {
    goalLower = lowerAngle.in(Radians);
    goalUpper = upperAngle.in(Radians);
    index = 0;
    path = arm.planPath(lowerAngle, upperAngle);
    if (path.isEmpty()) {
      holdLower = arm.getLowerAngle().in(Radians);
      holdUpper = arm.getUpperAngle().in(Radians);
      DriverStationBackend.reportWarning("[WARNING] " + arm.getName() + " has no path to " + lowerAngle.in(Degrees) + "°, " + upperAngle.in(Degrees) +
                                         "° that stays out of its exclusion zones; holding where it is.", false);
      return false;
    }
    return true;
  }

  /**
   * Whether the goal is within the path tolerance of the one being followed, so it doesn't need planning again.
   *
   * @param lowerAngle Lower joint angle.
   * @param upperAngle Upper joint angle.
   * @return True if the goal is the same, within the path tolerance.
   */
  public boolean hasGoal(Angle lowerAngle, Angle upperAngle) {
    final double tolerance = arm.getPathTolerance().in(Radians);
    return Math.abs(lowerAngle.in(Radians) - goalLower) <= tolerance && Math.abs(upperAngle.in(Radians) - goalUpper) <= tolerance;
  }

  /** Compensate kG, and give the joints the current waypoint, moving on as soon as the arm safely can. */
  public void execute() {
    arm.updateKg();
    if (path.isEmpty()) {
      if (!Double.isNaN(holdLower)) {
        arm.getLowerMotorController().setPosition(Radians.of(holdLower));
        arm.getUpperMotorController().setPosition(Radians.of(holdUpper));
      }
      return;
    }
    final DoubleJointedArmPath waypoints = path.get();
    final double lower = arm.getLowerAngle().in(Radians);
    final double upper = arm.getUpperAngle().in(Radians);
    while (index < waypoints.size() - 1 &&
           (isNear(waypoints, index, lower, upper) || arm.isMoveClear(lower, upper, waypoints.getLowerRadians(index + 1), waypoints.getUpperRadians(index + 1)))) {
      index++;
    }
    arm.getLowerMotorController().setPosition(waypoints.getLowerAngle(index));
    arm.getUpperMotorController().setPosition(waypoints.getUpperAngle(index));
  }

  /**
   * Whether the arm is at the goal: following the last waypoint, and within the path tolerance of it.
   *
   * @return True at the goal.
   */
  public boolean isAtGoal() {
    return path.isPresent() && index == path.get().size() - 1 && isNear(path.get(), index, arm.getLowerAngle().in(Radians), arm.getUpperAngle().in(Radians));
  }

  /**
   * Get the path being followed.
   *
   * @return The {@link DoubleJointedArmPath}, or empty while holding.
   */
  public Optional<DoubleJointedArmPath> getPath() {
    return path;
  }

  /**
   * Get the index of the waypoint being moved to.
   *
   * @return Waypoint index.
   */
  public int getWaypointIndex() {
    return index;
  }

  /**
   * Get the goal lower joint angle.
   *
   * @return Goal, in radians, or NaN before a goal is set.
   */
  public double getGoalLowerRadians() {
    return goalLower;
  }

  /**
   * Get the goal upper joint angle.
   *
   * @return Goal, in radians, or NaN before a goal is set.
   */
  public double getGoalUpperRadians() {
    return goalUpper;
  }

  /** Whether both joints are within the path tolerance of a waypoint. */
  private boolean isNear(DoubleJointedArmPath waypoints, int waypoint, double lower, double upper) {
    final double tolerance = arm.getPathTolerance().in(Radians);
    return Math.abs(lower - waypoints.getLowerRadians(waypoint)) <= tolerance && Math.abs(upper - waypoints.getUpperRadians(waypoint)) <= tolerance;
  }
}
