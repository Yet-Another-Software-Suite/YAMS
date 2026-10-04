// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package yams.core.mechanisms.config;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.MetersPerSecond;

import java.util.concurrent.atomic.AtomicReference;
import org.junit.jupiter.api.Test;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.linalg.VecBuilder;
import org.wpilib.math.util.MathUtil;
import yams.core.mechanisms.config.enums.GyroAxis;

/**
 * Tests {@link SwerveDriveConfig#withGyroHeadingAxis(GyroAxis)}: the gyro's attitude is rotated
 * into the robot's frame, so the heading follows the robot turning about whichever gyro axis points
 * up, past a quarter and half rotation, and the tilt anti-tipping reads stays the robot's.
 */
public class SwerveDriveGyroAxisTest {
  private static final double kTolerance = 1e-6;

  private final AtomicReference<Rotation3d> gyro = new AtomicReference<>(new Rotation3d());

  private SwerveDriveConfig<?> config(GyroAxis axis) {
    return new yams.commands2.config.SwerveDriveConfig().withGyro(gyro::get).withGyroHeadingAxis(axis);
  }

  /** The gyro rotated about its own X, Y or Z axis by the given degrees. */
  private void rotateGyro(double x, double y, double z, double degrees) {
    gyro.set(new Rotation3d(VecBuilder.fill(x, y, z), Math.toRadians(degrees)));
  }

  /** The robot's heading from the config, in degrees. */
  private static double headingDegrees(SwerveDriveConfig<?> config) {
    return Math.toDegrees(config.getGyroRotation3d().getZ());
  }

  /**
   * Turns the gyro about the given axis through a full rotation and back, checking the heading,
   * which wraps at half a rotation each way.
   */
  private void assertHeadingFollows(GyroAxis axis, double x, double y, double z) {
    final SwerveDriveConfig<?> config = config(axis);
    final double[] turns = {0, 45, 89, 91, 135, 179, 181, 270, 359, 181, 90, 0, -90, -179};
    for (double turn : turns) {
      rotateGyro(x, y, z, turn);
      final double expected = Math.toDegrees(MathUtil.angleModulus(Math.toRadians(turn)));
      assertEquals(expected, headingDegrees(config), kTolerance, axis + " heading after turning " + turn);
    }
  }

  @Test
  void yawHeadingFollowsGyroZ() {
    assertHeadingFollows(GyroAxis.YAW, 0, 0, 1);
  }

  @Test
  void rollHeadingFollowsGyroX() {
    assertHeadingFollows(GyroAxis.ROLL, 1, 0, 0);
  }

  @Test
  void pitchHeadingFollowsGyroYPastAQuarterRotation() {
    // A Rotation3d's pitch only reaches a quarter rotation each way, so the heading has to come from
    // the robot frame yaw, not the gyro's pitch.
    assertHeadingFollows(GyroAxis.PITCH, 0, 1, 0);
  }

  @Test
  void inversionAndOffsetApplyToTheChosenAxis() {
    final SwerveDriveConfig<?> config = config(GyroAxis.ROLL).withGyroInverted(true).withGyroOffset(Degrees.of(10));
    rotateGyro(1, 0, 0, 30);
    assertEquals(-40, headingDegrees(config), kTolerance, "inverted, offset roll heading");
  }

  @Test
  void offsetChangesOnlyTheHeading() {
    final SwerveDriveConfig<?> config = config(GyroAxis.YAW).withGyroOffset(Degrees.of(90));
    gyro.set(new Rotation3d(Math.toRadians(5), Math.toRadians(-8), Math.toRadians(30)));
    final Rotation3d robot = config.getGyroRotation3d();
    assertEquals(-60, Math.toDegrees(robot.getZ()), kTolerance, "offset heading");
    assertEquals(5, Math.toDegrees(robot.getX()), kTolerance, "roll kept");
    assertEquals(-8, Math.toDegrees(robot.getY()), kTolerance, "pitch kept");
  }

  @Test
  void tiltIsTheRobotsForSideMountedGyros() {
    // ROLL mounting keeps the gyro's Y axis as the robot's: the robot pitching forward is a gyro
    // rotation about its Y axis.
    rotateGyro(0, 1, 0, 15);
    final Rotation3d rollMounted = config(GyroAxis.ROLL).getGyroRotation3d();
    assertEquals(15, Math.toDegrees(rollMounted.getY()), kTolerance, "ROLL mounted, robot pitch");
    assertEquals(0, Math.toDegrees(rollMounted.getX()), kTolerance, "ROLL mounted, robot roll");

    // PITCH mounting keeps the gyro's X axis as the robot's: the robot rolling is a gyro rotation
    // about its X axis.
    rotateGyro(1, 0, 0, 15);
    final Rotation3d pitchMounted = config(GyroAxis.PITCH).getGyroRotation3d();
    assertEquals(15, Math.toDegrees(pitchMounted.getX()), kTolerance, "PITCH mounted, robot roll");
    assertEquals(0, Math.toDegrees(pitchMounted.getY()), kTolerance, "PITCH mounted, robot pitch");
  }

  @Test
  void antiTippingCorrectsTheRobotsTiltWithASideMountedGyro() {
    final SwerveDriveConfig<?> config = config(GyroAxis.ROLL)
        .withAntiTipping(MetersPerSecond.of(4), Degrees.of(10), MetersPerSecond.of(1.5));
    // Robot tipping forward 15 degrees, seen by a ROLL mounted gyro as rotation about its Y axis.
    rotateGyro(0, 1, 0, 15);
    assertEquals(4 * Math.sin(Math.toRadians(15)), config.getAntiTippingCorrection().vx, kTolerance, "forward correction");
    assertEquals(0, config.getAntiTippingCorrection().vy, kTolerance, "no sideways correction");
  }
}
