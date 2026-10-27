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
 * move straight from where it is to the next one and stay clear of the zones. That is line of sight path simplification,
 * as PathPlanner does to its paths, but from where the arm really is: the arm skips to the farthest waypoint it has a
 * clear way to, and also moves on when gravity holds a joint further from a waypoint than the tolerance. Without
 * exclusion zones the goal is the only waypoint.
 *
 * <p>The path is planned again from where the arm is, as PathPlanner's pathfinding does, when:
 *
 * <ul>
 *   <li>the dynamic exclusion zones change and block the rest of it
 *   (see {@link DoubleJointedArm#setDynamicExclusionZones(java.util.List)}).</li>
 *   <li>the arm is pushed off it, so moving on to its waypoint is no longer clear.</li>
 * </ul>
 *
 * <p>When the goal can't be reached without entering an exclusion zone, or is in one, the arm holds where it was and a
 * warning is reported, or with
 * {@link yams.core.mechanisms.config.DoubleJointedArmConfig.GoalInExclusionZone#NEAREST_CLEAR} moves to the nearest
 * clear angles to it.
 */
public class DoubleJointedArmPathFollower {
  /** Joint angle difference that is the same goal, in radians: floating point error, not a move. */
  private static final double kSameGoal = 1e-9;
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
  /** Dynamic exclusion zone change the path was planned or checked against. */
  private int                            zoneVersion;
  /** Whether the path was planned again after the arm was pushed off it, until it is back on it. */
  private boolean                        replannedOffPath = false;

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
    replannedOffPath = false;
    return plan();
  }

  /** Plan a path from where the arm is to the goal. */
  private boolean plan() {
    final Angle lowerAngle = Radians.of(goalLower);
    final Angle upperAngle = Radians.of(goalUpper);
    index = 0;
    zoneVersion = arm.getExclusionZoneVersion();
    path = arm.planPath(lowerAngle, upperAngle);
    path.ifPresent(waypoints -> {
      final int last = waypoints.size() - 1;
      if (Math.abs(waypoints.getLowerRadians(last) - goalLower) > 1e-9 || Math.abs(waypoints.getUpperRadians(last) - goalUpper) > 1e-9) {
        DriverStationBackend.reportWarning("[WARNING] " + arm.getName() + " goal " + lowerAngle.in(Degrees) + "°, " + upperAngle.in(Degrees) +
                                           "° is in an exclusion zone; moving to the nearest clear angles, " + waypoints.getLowerAngle(last).in(Degrees) + "°, " +
                                           waypoints.getUpperAngle(last).in(Degrees) + "°.", false);
      }
    });
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
   * Whether the goal is the one being followed, so it doesn't need planning again. Any real change is a new goal, however
   * small: goals a few inches apart are only a degree or two apart, often less than the path tolerance, and the arm must
   * still end at the new one. A clear move to it is planned without a search.
   *
   * @param lowerAngle Lower joint angle.
   * @param upperAngle Upper joint angle.
   * @return True if the goal is the same, to within floating point error.
   */
  public boolean hasGoal(Angle lowerAngle, Angle upperAngle) {
    return Math.abs(lowerAngle.in(Radians) - goalLower) <= kSameGoal && Math.abs(upperAngle.in(Radians) - goalUpper) <= kSameGoal;
  }

  /** Compensate kG, and give the joints the current waypoint, moving on as soon as the arm safely can. */
  public void execute() {
    arm.updateKg();
    if (Double.isNaN(goalLower)) {
      // No goal yet.
      return;
    }
    final double lower = arm.getLowerAngle().in(Radians);
    final double upper = arm.getUpperAngle().in(Radians);
    if (zoneVersion != arm.getExclusionZoneVersion()) {
      // The dynamic zones changed: plan again if they block the path, or may have cleared the way to the goal.
      zoneVersion = arm.getExclusionZoneVersion();
      if (!isRestOfPathClear(lower, upper)) {
        plan();
      }
    } else if (path.isPresent() && !arm.isMoveClear(lower, upper, path.get().getLowerRadians(index), path.get().getUpperRadians(index))) {
      // Pushed off the path: plan again from here, once, until the arm is back on a path.
      if (!replannedOffPath) {
        replannedOffPath = true;
        plan();
      }
    } else {
      replannedOffPath = false;
    }
    if (path.isEmpty()) {
      arm.getLowerMotorController().setPosition(Radians.of(holdLower));
      arm.getUpperMotorController().setPosition(Radians.of(holdUpper));
      return;
    }
    final DoubleJointedArmPath waypoints = path.get();
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

  /** Whether the arm can still move from where it is through the rest of the path's waypoints. */
  private boolean isRestOfPathClear(double lower, double upper) {
    if (path.isEmpty()) {
      return false;
    }
    final DoubleJointedArmPath waypoints = path.get();
    double fromLower = lower;
    double fromUpper = upper;
    for (int k = index; k < waypoints.size(); k++) {
      if (!arm.isMoveClear(fromLower, fromUpper, waypoints.getLowerRadians(k), waypoints.getUpperRadians(k))) {
        return false;
      }
      fromLower = waypoints.getLowerRadians(k);
      fromUpper = waypoints.getUpperRadians(k);
    }
    return true;
  }

  /** Whether both joints are within the path tolerance of a waypoint. */
  private boolean isNear(DoubleJointedArmPath waypoints, int waypoint, double lower, double upper) {
    final double tolerance = arm.getPathTolerance().in(Radians);
    return Math.abs(lower - waypoints.getLowerRadians(waypoint)) <= tolerance && Math.abs(upper - waypoints.getUpperRadians(waypoint)) <= tolerance;
  }
}
