// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package yams.core.mechanisms.config;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.MetersPerSecond;

import java.util.concurrent.atomic.AtomicReference;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.kinematics.ChassisVelocities;

/**
 * Tests {@link SwerveDriveConfig#withAntiTipping}: once the robot's pitch or roll, from the gyro's
 * attitude, is past the
 * threshold, a correction toward the side it is tipping to, {@code kP} times the sine of the tilt
 * and at most the maximum correction speed, is added to every robot relative chassis speed. With
 * WPILib's convention, positive pitch tips the robot forward and positive roll tips it right.
 */
public class SwerveDriveAntiTippingTest {
  private static final double kTolerance = 1e-9;
  /** Correction speed per sine of the tilt, in meters per second. */
  private static final double kP = 4;

  private final AtomicReference<Rotation3d> attitude = new AtomicReference<>(new Rotation3d());
  private SwerveDriveConfig<?> config;

  @BeforeEach
  void createConfig() {
    config = new yams.commands2.config.SwerveDriveConfig()
        .withGyro(attitude::get)
        .withAntiTipping(MetersPerSecond.of(kP), Degrees.of(10), MetersPerSecond.of(1.5));
  }

  /** The robot's attitude, from its roll, pitch and yaw in degrees. */
  private void tilt(double rollDegrees, double pitchDegrees, double yawDegrees) {
    attitude.set(new Rotation3d(Math.toRadians(rollDegrees), Math.toRadians(pitchDegrees), Math.toRadians(yawDegrees)));
  }

  private static void assertSpeeds(double vx, double vy, double omega, ChassisVelocities actual, String message) {
    assertEquals(vx, actual.vx, kTolerance, message + ": vx");
    assertEquals(vy, actual.vy, kTolerance, message + ": vy");
    assertEquals(omega, actual.omega, kTolerance, message + ": omega");
  }

  @Test
  void doesNotCorrectBelowTheThreshold() {
    tilt(5, -8, 0);
    assertSpeeds(0, 0, 0, config.getAntiTippingCorrection(), "below the threshold");
  }

  @Test
  void drivesForwardUnderAForwardTip() {
    tilt(0, 15, 0);
    assertSpeeds(kP * Math.sin(Math.toRadians(15)), 0, 0, config.getAntiTippingCorrection(), "tipping forward");
  }

  @Test
  void drivesRightUnderARightwardTip() {
    tilt(15, 0, 0);
    assertSpeeds(0, -kP * Math.sin(Math.toRadians(15)), 0, config.getAntiTippingCorrection(), "tipping right");
  }

  @Test
  void correctsRelativeToTheRobotWhateverItsHeading() {
    // The chassis speeds the correction is added to are robot relative, so the heading does not
    // change which way the robot drives to catch itself.
    tilt(0, 15, 90);
    assertSpeeds(kP * Math.sin(Math.toRadians(15)), 0, 0, config.getAntiTippingCorrection(), "tipping forward, facing 90 degrees");
  }

  @Test
  void correctsNoFasterThanTheMaximumCorrectionSpeed() {
    tilt(0, -40, 0);
    assertSpeeds(-1.5, 0, 0, config.getAntiTippingCorrection(), "tipping back past the maximum correction");
  }

  @Test
  void addsTheCorrectionToRobotRelativeChassisSpeeds() {
    tilt(0, 15, 0);
    final ChassisVelocities speeds = config.optimizeRobotRelativeChassisSpeeds(new ChassisVelocities(1, 0.5, 2));
    assertSpeeds(1 + kP * Math.sin(Math.toRadians(15)), 0.5, 2, speeds, "driving while tipping forward");
  }

  @Test
  void withoutAntiTippingChassisSpeedsAreUnchanged() {
    final SwerveDriveConfig<?> plain = new yams.commands2.config.SwerveDriveConfig().withGyro(attitude::get);
    tilt(0, 40, 0);
    assertSpeeds(1, 0.5, 2, plain.optimizeRobotRelativeChassisSpeeds(new ChassisVelocities(1, 0.5, 2)), "no anti-tipping");
    assertSpeeds(0, 0, 0, plain.getAntiTippingCorrection(), "no anti-tipping");
  }

  @Test
  void rejectsSettingsThatCannotCorrect() {
    final SwerveDriveConfig<?> plain = new yams.commands2.config.SwerveDriveConfig();
    assertThrows(IllegalArgumentException.class,
        () -> plain.withAntiTipping(MetersPerSecond.of(0), Degrees.of(10), MetersPerSecond.of(1)), "zero kP");
    assertThrows(IllegalArgumentException.class,
        () -> plain.withAntiTipping(MetersPerSecond.of(4), Degrees.of(0), MetersPerSecond.of(1)), "zero threshold");
    assertThrows(IllegalArgumentException.class,
        () -> plain.withAntiTipping(MetersPerSecond.of(4), Degrees.of(90), MetersPerSecond.of(1)), "90 degree threshold");
    assertThrows(IllegalArgumentException.class,
        () -> plain.withAntiTipping(MetersPerSecond.of(4), Degrees.of(10), MetersPerSecond.of(0)), "zero maximum speed");
    assertTrue(plain.getAntiTippingCorrection().vx == 0, "rejected settings are not applied");
  }

  @Test
  void headingStaysContinuousPastHalfARotation() {
    // A Rotation3d's yaw wraps at 180 degrees, but the heading turns on past it.
    final SwerveDriveConfig<?> plain = new yams.commands2.config.SwerveDriveConfig().withGyro(attitude::get);
    final double[] yaws = {170, 179, -179, -170, 179, 170};
    final double[] headings = {170, 179, 181, 190, 179, 170};
    for (int i = 0; i < yaws.length; i++) {
      tilt(0, 0, yaws[i]);
      assertEquals(headings[i], plain.getGyroAngle().in(Degrees), 1e-6, "heading after yaw " + yaws[i]);
    }
  }
}
