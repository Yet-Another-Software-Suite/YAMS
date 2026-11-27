// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package first.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.RPM;
import static org.wpilib.units.Units.Seconds;

import org.junit.jupiter.api.Test;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.kinematics.ChassisVelocities;

/** The three firing strategies, checked against each other and against what a lead should look like. */
class ShotCalculatorTest {
  /** About 3.3 m from the target, where the example robot starts. */
  private static final Pose2d ROBOT = new Pose2d(2, 2, Rotation2d.ZERO);

  @Test
  void standingStillEveryStrategyAimsAtTheRealTarget() {
    var still = new ChassisVelocities(0, 0, 0);
    var standing = ShotCalculator.standing(ROBOT);
    assertEquals(ShotCalculator.TARGET, standing.target().getTranslation());
    assertNear(ShotCalculator.TARGET, ShotCalculator.recursive(ROBOT, still).target().getTranslation(), 1e-9);
    assertNear(ShotCalculator.TARGET, ShotCalculator.newton(ROBOT, still).target().getTranslation(), 1e-9);
    assertEquals(standing.flywheelSpeed().in(RPM), ShotCalculator.recursive(ROBOT, still).flywheelSpeed().in(RPM), 1e-9);
  }

  @Test
  void movingTheAimPointLeadsAgainstTheRobotsVelocity() {
    // Driving 1.5 m/s along +X, past the target.
    var moving = new ChassisVelocities(1.5, 0, 0);
    Translation2d lead = ShotCalculator.TARGET.minus(ShotCalculator.recursive(ROBOT, moving).target().getTranslation());
    // The aim point moves straight back along -X...
    assertEquals(0, lead.getY(), 1e-9);
    assertTrue(lead.getX() > 0, "the aim point is behind the target, against the velocity: " + lead);
    // ...by the velocity times a time of flight from the table, which runs from 0.45 s to 0.95 s.
    assertTrue(lead.getX() > 1.5 * 0.45 && lead.getX() < 1.5 * 0.95, "lead " + lead.getX() + " m");
  }

  @Test
  void theLeadIsTheVelocityTimesTheTimeOfFlightToTheAimPoint() {
    // Toward, away from and across the target, at a few speeds. The right aim point is the one
    // whose own time of flight, times the robot's velocity, is how far it is from the target.
    // Each keeps the aim point within the 1.5 to 4.5 m the tables cover; past their ends the
    // tables go flat, and no strategy can settle on a time the table does not have.
    double[][] velocities = {{1, 0}, {0, 1.5}, {-1, 0.5}, {2, 2}, {-0.5, -0.5}};
    for (double[] v : velocities) {
      var velocity = new ChassisVelocities(v[0], v[1], 0);
      // Newton's method settles on it in three passes.
      assertSettled(ShotCalculator.newton(ROBOT, velocity), velocity, 0.01);
      // The recursion's three passes come close: within 15 cm at these speeds. Driving fast toward
      // or away from the target leaves it furthest off, which is why Newton's method exists.
      assertSettled(ShotCalculator.recursive(ROBOT, velocity), velocity, 0.15);
    }
  }

  private static void assertSettled(ShotCalculator.Shot shot, ChassisVelocities velocity, double meters) {
    Translation2d aimPoint = shot.target().getTranslation();
    double timeOfFlight =
        ShotCalculator.timeOfFlight(Meters.of(ROBOT.getTranslation().getDistance(aimPoint))).in(Seconds);
    Translation2d expected =
        ShotCalculator.TARGET.minus(new Translation2d(velocity.vx * timeOfFlight, velocity.vy * timeOfFlight));
    assertNear(expected, aimPoint, meters);
  }

  @Test
  void inRangeIsTheDistanceTheTablesCover() {
    assertTrue(ShotCalculator.inRange(ROBOT));
    assertFalse(ShotCalculator.inRange(new Pose2d(0, 0, Rotation2d.ZERO)), "6.1 m away is too far");
    assertFalse(ShotCalculator.inRange(new Pose2d(4.6, 3.0, Rotation2d.ZERO)), "1 m away is too close");
    assertTrue(ShotCalculator.inRange(new Pose2d(4.6, 2.5, Rotation2d.ZERO)), "1.5 m away is the closest");
  }

  private static void assertNear(Translation2d expected, Translation2d actual, double meters) {
    assertTrue(expected.getDistance(actual) <= meters,
        "expected " + expected + " but was " + actual + ", " + expected.getDistance(actual) + " m away");
  }
}
