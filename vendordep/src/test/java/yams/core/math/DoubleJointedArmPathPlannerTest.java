// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package yams.core.math;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.Radians;

import java.util.List;
import java.util.Optional;
import org.junit.jupiter.api.Test;
import org.wpilib.math.geometry.Translation2d;
import yams.core.mechanisms.config.DoubleJointedArmConfig.ExclusionZone;

class DoubleJointedArmPathPlannerTest {
  private static final double kLowerLength = 1.0;
  private static final double kUpperLength = 0.8;
  private static final double kTolerance = 1e-9;

  /** Over the shoulder: the arm can't swing over the top. */
  private static final ExclusionZone kAbove =
      new ExclusionZone(new Translation2d(-0.3, 0.2), new Translation2d(0.3, 2.5));

  /** Under the shoulder: the arm can't swing under the bottom. */
  private static final ExclusionZone kBelow =
      new ExclusionZone(new Translation2d(0.3, -2.5), new Translation2d(-0.3, -0.2));

  private static DoubleJointedArmPathPlanner planner(ExclusionZone... zones) {
    return new DoubleJointedArmPathPlanner(
        Meters.of(kLowerLength),
        Meters.of(kUpperLength),
        Degrees.of(-270),
        Degrees.of(270),
        Degrees.of(-270),
        Degrees.of(270),
        List.of(zones),
        Meters.of(0),
        Degrees.of(2),
        Degrees.of(1));
  }

  private static Optional<DoubleJointedArmPath> plan(
      DoubleJointedArmPathPlanner planner,
      double startLower,
      double startUpper,
      double goalLower,
      double goalUpper) {
    return planner.plan(
        Math.toRadians(startLower),
        Math.toRadians(startUpper),
        Math.toRadians(goalLower),
        Math.toRadians(goalUpper));
  }

  /** Whether the arm touches a zone, by sampling points along both segments. Independent of the planner's check. */
  private static boolean touches(double lower, double upper, List<ExclusionZone> zones) {
    final double elbowX = kLowerLength * Math.cos(lower);
    final double elbowY = kLowerLength * Math.sin(lower);
    for (int k = 0; k <= 200; k++) {
      final double t = k / 200.0;
      final double[][] points = {
        {elbowX * t, elbowY * t},
        {elbowX + kUpperLength * Math.cos(upper) * t, elbowY + kUpperLength * Math.sin(upper) * t}
      };
      for (double[] point : points) {
        for (ExclusionZone zone : zones) {
          if (point[0] >= zone.min().getX()
              && point[0] <= zone.max().getX()
              && point[1] >= zone.min().getY()
              && point[1] <= zone.max().getY()) {
            return true;
          }
        }
      }
    }
    return false;
  }

  /**
   * Whether the arm touches a zone anywhere it can be while following a step: the box of angles between the waypoints,
   * widened by the path tolerance, as each joint can get to its next angle before the other.
   */
  private static boolean stepTouches(
      double fromLower,
      double fromUpper,
      double toLower,
      double toUpper,
      List<ExclusionZone> zones) {
    final double tolerance = Math.toRadians(1);
    final double minLower = Math.min(fromLower, toLower) - tolerance;
    final double minUpper = Math.min(fromUpper, toUpper) - tolerance;
    final double maxLower = Math.max(fromLower, toLower) + tolerance;
    final double maxUpper = Math.max(fromUpper, toUpper) + tolerance;
    for (int i = 0; i <= 6; i++) {
      for (int j = 0; j <= 6; j++) {
        if (touches(
            minLower + (maxLower - minLower) * i / 6,
            minUpper + (maxUpper - minUpper) * j / 6,
            zones)) {
          return true;
        }
      }
    }
    return false;
  }

  private static void assertPathAvoids(
      DoubleJointedArmPath path, double startLower, double startUpper, List<ExclusionZone> zones) {
    double lower = Math.toRadians(startLower);
    double upper = Math.toRadians(startUpper);
    for (int k = 0; k < path.size(); k++) {
      assertFalse(
          stepTouches(lower, upper, path.getLowerRadians(k), path.getUpperRadians(k), zones),
          "step " + k + " enters an exclusion zone");
      lower = path.getLowerRadians(k);
      upper = path.getUpperRadians(k);
    }
  }

  private static void assertEndsAt(DoubleJointedArmPath path, double goalLower, double goalUpper) {
    assertEquals(Math.toRadians(goalLower), path.getLowerRadians(path.size() - 1), kTolerance);
    assertEquals(Math.toRadians(goalUpper), path.getUpperRadians(path.size() - 1), kTolerance);
  }

  @Test
  void withoutExclusionZonesThePathIsTheGoal() {
    DoubleJointedArmPath path = plan(planner(), 0, 0, 180, 180).orElseThrow();

    assertEquals(1, path.size());
    assertEndsAt(path, 180, 180);
  }

  @Test
  void thePathGoesAroundAnExclusionZone() {
    // From reaching forward to reaching back, which is -180 degrees on both joints once the arm
    // swings under.
    DoubleJointedArmPath path = plan(planner(kAbove), 0, 0, -180, -180).orElseThrow();

    assertPathAvoids(path, 0, 0, List.of(kAbove));
    assertEndsAt(path, -180, -180);
    boolean under = false;
    for (int k = 0; k < path.size(); k++) {
      under |= path.getLowerRadians(k) < 0 || path.getLowerRadians(k) > Math.PI;
    }
    assertTrue(under, "the lower arm swings under the shoulder");
  }

  @Test
  void thePathFoldsTheArmPastAnExclusionZone() {
    // A wall in front of the arm, between where it is and where it goes: it can only get past by
    // folding.
    ExclusionZone wall =
        new ExclusionZone(new Translation2d(1.2, -0.4), new Translation2d(1.4, 0.4));
    DoubleJointedArmPath path = plan(planner(wall), 30, 30, -30, -30).orElseThrow();

    assertPathAvoids(path, 30, 30, List.of(wall));
    assertEndsAt(path, -30, -30);
  }

  @Test
  void aGoalInAnExclusionZoneHasNoPath() {
    assertTrue(plan(planner(kAbove), 0, 0, 90, 90).isEmpty());
  }

  @Test
  void theElbowStaysOutOfTheExclusionZones() {
    // A small zone around where the elbow is at 45 degrees, clear of where the rest of the arm is
    // anywhere near there.
    final double elbow = kLowerLength * Math.cos(Math.PI / 4);
    ExclusionZone zone =
        new ExclusionZone(
            new Translation2d(elbow - 0.02, elbow - 0.02),
            new Translation2d(elbow + 0.02, elbow + 0.02));
    DoubleJointedArmPathPlanner planner = planner(zone);

    assertTrue(
        planner.isInExclusionZone(Degrees.of(45), Degrees.of(-60)),
        "only the elbow is in the zone");
    assertTrue(plan(planner, 0, 0, 45, -60).isEmpty());
    // The elbow is always on a circle around the shoulder, so the lower arm can't get past 45
    // degrees this way round.
    assertTrue(plan(planner, 0, -60, 90, -60).isEmpty());
    DoubleJointedArmPath path = plan(planner, 0, -60, 40, -60).orElseThrow();
    for (int k = 0; k < path.size(); k++) {
      double elbowX = kLowerLength * Math.cos(path.getLowerRadians(k));
      double elbowY = kLowerLength * Math.sin(path.getLowerRadians(k));
      assertFalse(
          elbowX >= zone.min().getX()
              && elbowX <= zone.max().getX()
              && elbowY >= zone.min().getY()
              && elbowY <= zone.max().getY(),
          "the elbow is in the zone at waypoint " + k);
    }
    assertPathAvoids(path, 0, -60, List.of(zone));
    assertEndsAt(path, 40, -60);
  }

  @Test
  void theUpperArmCrossingAWallCollides() {
    // Only the upper arm reaches the wall, crossing it near its top corner.
    ExclusionZone wall =
        new ExclusionZone(new Translation2d(0.75, -0.3), new Translation2d(0.9, 0.3));
    DoubleJointedArmPathPlanner planner =
        new DoubleJointedArmPathPlanner(
            Meters.of(0.6),
            Meters.of(0.5),
            Degrees.of(-270),
            Degrees.of(270),
            Degrees.of(-270),
            Degrees.of(270),
            List.of(wall),
            Meters.of(0),
            Degrees.of(2),
            Degrees.of(1));

    assertTrue(planner.isInExclusionZone(Degrees.of(21), Degrees.of(22)));
    assertTrue(planner.isInExclusionZone(Degrees.of(0), Degrees.of(0)));
  }

  @Test
  void aStartAndGoalNearAZoneDontLetThePathThroughIt() {
    // Start and goal are within the following error of the wall, but the path must still go around
    // it.
    ExclusionZone wall =
        new ExclusionZone(new Translation2d(0.75, -0.3), new Translation2d(0.9, 0.3));
    DoubleJointedArmPathPlanner planner =
        new DoubleJointedArmPathPlanner(
            Meters.of(0.6),
            Meters.of(0.5),
            Degrees.of(-270),
            Degrees.of(270),
            Degrees.of(-270),
            Degrees.of(270),
            List.of(wall),
            Meters.of(0.03),
            Degrees.of(2),
            Degrees.of(3));
    DoubleJointedArmPath path = plan(planner, 30, 30, -30, -30).orElseThrow();

    double lower = Math.toRadians(30);
    double upper = Math.toRadians(30);
    for (int k = 0; k < path.size(); k++) {
      double nextLower = path.getLowerRadians(k);
      double nextUpper = path.getUpperRadians(k);
      for (int i = 0; i <= 10; i++) {
        double l = lower + (nextLower - lower) * i / 10;
        double u = upper + (nextUpper - upper) * i / 10;
        double elbowX = 0.6 * Math.cos(l);
        double elbowY = 0.6 * Math.sin(l);
        for (int j = 0; j <= 50; j++) {
          double x = elbowX + 0.5 * Math.cos(u) * j / 50;
          double y = elbowY + 0.5 * Math.sin(u) * j / 50;
          assertFalse(
              x >= 0.75 && x <= 0.9 && y >= -0.3 && y <= 0.3,
              "step " + k + " goes through the wall");
        }
      }
      lower = nextLower;
      upper = nextUpper;
    }
    assertEndsAt(path, -30, -30);
  }

  @Test
  void aGoalOutsideTheJointLimitsHasNoPath() {
    assertTrue(plan(planner(kAbove), 0, 0, 300, 0).isEmpty());
  }

  @Test
  void aGoalThatCanOnlyBeReachedThroughAnExclusionZoneHasNoPath() {
    // The angles are absolute, so reaching back at 180 degrees means swinging the lower arm over
    // the top.
    assertTrue(plan(planner(kAbove), 0, 0, 180, 180).isEmpty());
    // Over the top and under the bottom are both blocked, so the lower arm can never get to the
    // other side.
    assertTrue(plan(planner(kAbove, kBelow), 0, 0, -180, -180).isEmpty());
  }

  @Test
  void anArmStartingInAnExclusionZoneLeavesItAndNeverEntersAnother() {
    DoubleJointedArmPath path = plan(planner(kAbove), 80, 80, 0, 0).orElseThrow();

    assertTrue(touches(Math.toRadians(80), Math.toRadians(80), List.of(kAbove)));
    boolean left = false;
    for (int k = 0; k < path.size(); k++) {
      boolean inZone = touches(path.getLowerRadians(k), path.getUpperRadians(k), List.of(kAbove));
      assertFalse(left && inZone, "waypoint " + k + " enters the exclusion zone again");
      left |= !inZone;
    }
    assertTrue(left);
    assertEndsAt(path, 0, 0);
  }

  @Test
  void theClearanceKeepsTheArmAwayFromTheZones() {
    // Out of the lower arm's reach, 5 cm above the arm reaching forward.
    ExclusionZone zone =
        new ExclusionZone(new Translation2d(1.3, 0.05), new Translation2d(1.5, 0.5));
    DoubleJointedArmPathPlanner planner =
        new DoubleJointedArmPathPlanner(
            Meters.of(kLowerLength),
            Meters.of(kUpperLength),
            Degrees.of(-270),
            Degrees.of(270),
            Degrees.of(-270),
            Degrees.of(270),
            List.of(zone),
            Meters.of(0.1),
            Degrees.of(2),
            Degrees.of(1));

    assertFalse(planner(zone).isInExclusionZone(Degrees.of(0), Degrees.of(0)));
    assertTrue(planner.isInExclusionZone(Degrees.of(0), Degrees.of(0)), "within the clearance");
    // Raising the straight arm would sweep through the zone, so the upper arm folds out of the way.
    DoubleJointedArmPath path = plan(planner, -30, -30, 60, 60).orElseThrow();
    assertPathAvoids(
        path,
        -30,
        -30,
        List.of(new ExclusionZone(new Translation2d(1.2, -0.05), new Translation2d(1.6, 0.6))));
    assertEndsAt(path, 60, 60);
  }

  @Test
  void stepsMovingOneJointAreMerged() {
    // Only the upper arm moves, and nothing is in its way.
    ExclusionZone farAway = new ExclusionZone(new Translation2d(10, 10), new Translation2d(11, 11));
    DoubleJointedArmPath path = plan(planner(farAway), 0, 0, 0, 90).orElseThrow();

    assertTrue(
        path.size() <= 3,
        "a straight move is a few waypoints, not one a grid step: " + path.size());
    assertEndsAt(path, 0, 90);
  }

  @Test
  void aPlannerCanBeUsedAgain() {
    DoubleJointedArmPathPlanner planner = planner(kAbove);
    DoubleJointedArmPath first = plan(planner, 0, 0, -180, -180).orElseThrow();
    assertTrue(plan(planner, 0, 0, 90, 90).isEmpty());
    // A smaller grid, then the first plan again.
    DoubleJointedArmPathPlanner small =
        new DoubleJointedArmPathPlanner(
            Meters.of(kLowerLength),
            Meters.of(kUpperLength),
            Degrees.of(-10),
            Degrees.of(10),
            Degrees.of(-10),
            Degrees.of(10),
            List.of(kAbove),
            Meters.of(0),
            Degrees.of(2),
            Degrees.of(1));
    assertTrue(plan(small, 0, 0, 5, 5).isPresent());
    DoubleJointedArmPath again = plan(planner, 0, 0, -180, -180).orElseThrow();

    assertEquals(first.size(), again.size());
    for (int k = 0; k < first.size(); k++) {
      assertEquals(first.getLowerRadians(k), again.getLowerRadians(k), kTolerance);
      assertEquals(first.getUpperRadians(k), again.getUpperRadians(k), kTolerance);
    }
  }

  @Test
  void jointsWithoutLimitsCanPlanAnywhere() {
    DoubleJointedArmPathPlanner planner =
        new DoubleJointedArmPathPlanner(
            Meters.of(kLowerLength),
            Meters.of(kUpperLength),
            Radians.of(Double.NEGATIVE_INFINITY),
            Radians.of(Double.POSITIVE_INFINITY),
            Radians.of(Double.NEGATIVE_INFINITY),
            Radians.of(Double.POSITIVE_INFINITY),
            List.of(kAbove),
            Meters.of(0),
            Degrees.of(2),
            Degrees.of(1));
    DoubleJointedArmPath path = plan(planner, 0, 0, -180, -180).orElseThrow();

    assertPathAvoids(path, 0, 0, List.of(kAbove));
    assertEndsAt(path, -180, -180);
  }

  @Test
  void exclusionZoneCornersAreSorted() {
    ExclusionZone zone = new ExclusionZone(new Translation2d(1, -1), new Translation2d(-2, 3));

    assertEquals(new Translation2d(-2, -1), zone.min());
    assertEquals(new Translation2d(1, 3), zone.max());
  }
}
