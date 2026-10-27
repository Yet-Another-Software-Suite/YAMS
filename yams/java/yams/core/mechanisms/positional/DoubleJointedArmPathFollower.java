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
 *
 * <p>Each loop {@link #execute()} gives both joints' closed loop controllers the current waypoint, and moves on to the
 * next once both joints are within the path tolerance of it, the following error the path was planned with. Without
 * exclusion zones the goal is the only waypoint.
 *
 * <p>When the goal can't be reached without entering an exclusion zone, or is in one, the arm holds where it was and a
 * warning is reported.
 */
public class DoubleJointedArmPathFollower {
  /** Arm to move. */
  private final DoubleJointedArm               arm;
  /** Path being followed, or empty while holding. */
  private Optional<DoubleJointedArmPath>       path      = Optional.empty();
  /** Waypoint being moved to. */
  private int                                  index     = 0;
  /** Goal, in radians. */
  private double                               goalLower = Double.NaN;
  /** Goal, in radians. */
  private double                               goalUpper = Double.NaN;
  /** Angles held when there's no path, in radians. */
  private double                               holdLower = Double.NaN;
  /** Angles held when there's no path, in radians. */
  private double                               holdUpper = Double.NaN;

  /**
   * Create a path follower for an arm.
   *
   * @param arm {@link DoubleJointedArm} to move.
   */
  public DoubleJointedArmPathFollower(DoubleJointedArm arm) {
    this.arm = arm;
  }

  /**
   * Plan a path from where the arm is to a goal, and start following it.
   *
   * @param lowerAngle Goal lower arm angle, from horizontal.
   * @param upperAngle Goal upper arm angle, from horizontal.
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
   * @param lowerAngle Lower arm angle.
   * @param upperAngle Upper arm angle.
   * @return True if the goal is the same, within the path tolerance.
   */
  public boolean hasGoal(Angle lowerAngle, Angle upperAngle) {
    final double tolerance = arm.getPathTolerance().in(Radians);
    return Math.abs(lowerAngle.in(Radians) - goalLower) <= tolerance && Math.abs(upperAngle.in(Radians) - goalUpper) <= tolerance;
  }

  /** Give the joints the current waypoint, moving on to the next once the arm has reached it. */
  public void execute() {
    if (path.isEmpty()) {
      if (!Double.isNaN(holdLower)) {
        arm.getLowerMotorController().setPosition(Radians.of(holdLower));
        arm.getUpperMotorController().setPosition(Radians.of(holdUpper));
      }
      return;
    }
    final DoubleJointedArmPath waypoints = path.get();
    while (index < waypoints.size() - 1 && isNear(waypoints, index)) {
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
    return path.isPresent() && index == path.get().size() - 1 && isNear(path.get(), index);
  }

  /**
   * Get the path being followed.
   *
   * @return The {@link DoubleJointedArmPath}, or empty while holding.
   */
  public Optional<DoubleJointedArmPath> getPath() {
    return path;
  }

  /** Whether both joints are within the path tolerance of a waypoint. */
  private boolean isNear(DoubleJointedArmPath waypoints, int waypoint) {
    final double tolerance = arm.getPathTolerance().in(Radians);
    return Math.abs(arm.getLowerAngle().in(Radians) - waypoints.getLowerRadians(waypoint)) <= tolerance &&
           Math.abs(arm.getUpperAngle().in(Radians) - waypoints.getUpperRadians(waypoint)) <= tolerance;
  }
}
