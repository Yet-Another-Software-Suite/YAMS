// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package first.robot;

import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.RPM;
import static org.wpilib.units.Units.Seconds;

import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.interpolation.InterpolatingDoubleTreeMap;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Distance;
import org.wpilib.units.measure.Time;

/**
 * Works out where to aim and how fast to spin the flywheel, from where the robot is and how it is moving.
 *
 * <p>The tables hold what was measured standing still: the flywheel speed that scores from each distance, and how
 * long that shot is in the air. The three strategies are the three of WPILib's fire control pages. The two for a moving
 * robot lead the target by the time of flight: the game piece keeps the robot's velocity after it leaves, so aim that
 * far the other way.
 */
public final class ShotCalculator {
  /**
   * Where to aim, and the flywheel speed that reaches it.
   *
   * @param target The point to aim at. For a moving robot, this is the virtual target, not the real one.
   * @param flywheelSpeed The flywheel speed for the distance to that point.
   */
  public record Shot(Pose2d target, AngularVelocity flywheelSpeed) {}

  /** Where the real target is on the field. A placeholder: put your field's target here. */
  public static final Translation2d TARGET = new Translation2d(4.6, 4.0);

  /** The closest distance the tables cover. */
  public static final Distance MIN_DISTANCE = Meters.of(1.5);
  /** The farthest distance the tables cover. */
  public static final Distance MAX_DISTANCE = Meters.of(4.5);

  // Distance to the target in meters, to flywheel speed in RPM.
  private static final InterpolatingDoubleTreeMap FLYWHEEL_RPM = new InterpolatingDoubleTreeMap();
  // Distance to the target in meters, to how long the shot is in the air in seconds.
  private static final InterpolatingDoubleTreeMap TIME_OF_FLIGHT = new InterpolatingDoubleTreeMap();

  static {
    // Measure these on your own robot, standing still. These are placeholders, all well below the shooter's top
    // speed of about 3800 RPM.
    FLYWHEEL_RPM.put(1.5, 2000.0);
    FLYWHEEL_RPM.put(3.0, 2500.0);
    FLYWHEEL_RPM.put(4.5, 3000.0);

    TIME_OF_FLIGHT.put(1.5, 0.45);
    TIME_OF_FLIGHT.put(3.0, 0.70);
    TIME_OF_FLIGHT.put(4.5, 0.95);
  }

  private ShotCalculator() {}

  /**
   * Whether the robot is at a distance the tables cover. Outside it, a table only repeats its last entry, and the shot
   * misses.
   *
   * @param robotPose Where the robot is.
   * @return True between {@link #MIN_DISTANCE} and {@link #MAX_DISTANCE} from the target.
   */
  public static boolean inRange(Pose2d robotPose) {
    double distance = robotPose.getTranslation().getDistance(TARGET);
    return distance >= MIN_DISTANCE.in(Meters) && distance <= MAX_DISTANCE.in(Meters);
  }

  /**
   * How long a shot from a distance is in the air, from the table measured standing still.
   *
   * @param distance Distance to the point aimed at.
   * @return The time of flight.
   */
  public static Time timeOfFlight(Distance distance) {
    return Seconds.of(TIME_OF_FLIGHT.get(distance.in(Meters)));
  }

  /**
   * Aim at the real target. Only right while the robot is standing still.
   *
   * @param robotPose Where the robot is.
   * @return The shot.
   */
  public static Shot standing(Pose2d robotPose) {
    return shotAt(robotPose, TARGET);
  }

  /**
   * Lead the target by the time of flight, and repeat until the time settles: the virtual target most teams use.
   *
   * @param robotPose Where the robot is.
   * @param fieldVelocity How fast the robot is moving on the field.
   * @return The shot.
   */
  public static Shot recursive(Pose2d robotPose, ChassisVelocities fieldVelocity) {
    Translation2d virtualTarget = TARGET;
    // Each pass uses the distance from the pass before, so the answer settles after a few.
    for (int i = 0; i < 3; i++) {
      double distance = robotPose.getTranslation().getDistance(virtualTarget);
      double timeOfFlight = TIME_OF_FLIGHT.get(distance);
      // The shot drifts by the robot's velocity times the time of flight, so aim that far the other way.
      virtualTarget = TARGET.minus(new Translation2d(fieldVelocity.vx * timeOfFlight, fieldVelocity.vy * timeOfFlight));
    }
    return shotAt(robotPose, virtualTarget);
  }

  /**
   * Find the same time of flight as {@link #recursive} with Newton's method, which needs fewer passes. That helps when
   * driving fast toward or away from the target.
   *
   * @param robotPose Where the robot is.
   * @param fieldVelocity How fast the robot is moving on the field.
   * @return The shot.
   */
  public static Shot newton(Pose2d robotPose, ChassisVelocities fieldVelocity) {
    Translation2d toTarget = TARGET.minus(robotPose.getTranslation());
    double distance = toTarget.getNorm();
    // How fast the game piece covers ground, from the standing shot at this distance.
    double projectileSpeed = distance / TIME_OF_FLIGHT.get(distance);
    // How fast the robot is closing on the target. Negative while it drives away.
    double closingSpeed = (toTarget.getX() * fieldVelocity.vx + toTarget.getY() * fieldVelocity.vy) / distance;

    // A first guess that already allows for the robot's motion toward or away from the target.
    double timeOfFlight = distance / (projectileSpeed + closingSpeed);
    for (int i = 0; i < 3; i++) {
      // The virtual target for this guess.
      double dx = toTarget.getX() - fieldVelocity.vx * timeOfFlight;
      double dy = toTarget.getY() - fieldVelocity.vy * timeOfFlight;
      double virtualDistance = Math.hypot(dx, dy);
      // How far the guess is from what the table says for that distance.
      double error = timeOfFlight - TIME_OF_FLIGHT.get(virtualDistance);
      // How fast that error changes as the guess changes.
      double slope = 1 + (dx * fieldVelocity.vx + dy * fieldVelocity.vy) / (projectileSpeed * virtualDistance);
      timeOfFlight -= error / slope;
    }
    Translation2d virtualTarget =
        TARGET.minus(new Translation2d(fieldVelocity.vx * timeOfFlight, fieldVelocity.vy * timeOfFlight));
    return shotAt(robotPose, virtualTarget);
  }

  /** The shot that reaches a point on the field from where the robot is. */
  private static Shot shotAt(Pose2d robotPose, Translation2d aimPoint) {
    double distance = robotPose.getTranslation().getDistance(aimPoint);
    return new Shot(new Pose2d(aimPoint, Rotation2d.ZERO), RPM.of(FLYWHEEL_RPM.get(distance)));
  }
}
