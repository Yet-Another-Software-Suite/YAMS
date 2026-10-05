// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package yams.core.mechanisms.config;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.DegreesPerSecond;
import static org.wpilib.units.Units.MetersPerSecond;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.kinematics.SwerveModuleVelocity;
import yams.core.mechanisms.swerve.SwerveModuleVelocityWithAzimuth;

/**
 * Tests {@link SwerveModuleConfig#getOptimizedState(SwerveModuleVelocity,
 * SwerveModuleVelocityWithAzimuth)} and
 * {@link SwerveModuleVelocityWithAzimuth#optimize(SwerveModuleVelocity, SwerveModuleVelocityWithAzimuth, Rotation2d)}:
 * optimization turns a wheel at most 90 degrees and reverses the drive instead, a wheel keeps the
 * orientation it was last sent to unless the other is clearly better, so a turning wheel is not
 * reversed back and forth by a target that keeps moving, and a wheel commanded too slowly to steer
 * by holds where it is.
 */
public class SwerveModuleConfigTest {
  private SwerveModuleConfig config;

  @BeforeEach
  void createConfig() {
    // Given the wheel's measured state, the optimizer needs no motor controllers.
    config =
        new SwerveModuleConfig(null, null)
            .withOptimization(true)
            .withVelocityDeadband(MetersPerSecond.of(0.1));
  }

  private static SwerveModuleVelocityWithAzimuth wheelAt(double degrees) {
    return new SwerveModuleVelocityWithAzimuth(
        0, Rotation2d.fromDegrees(degrees), DegreesPerSecond.of(0));
  }

  private SwerveModuleVelocity optimize(
      double velocity, double targetDegrees, double wheelDegrees) {
    return config.getOptimizedState(
        new SwerveModuleVelocity(velocity, Rotation2d.fromDegrees(targetDegrees)),
        wheelAt(wheelDegrees));
  }

  @Test
  void turnsAtMostAQuarterTurnAndReversesTheDrive() {
    final SwerveModuleVelocity state = optimize(1, 150, 0);
    assertEquals(-30, state.angle.getDegrees(), 1e-6, "turns 30 degrees the other way");
    assertEquals(-1, state.velocity, 1e-9, "and drives backwards");
  }

  @Test
  void reversesTheDriveForATargetBehindTheWheel() {
    optimize(1, 0, 0);
    final SwerveModuleVelocity state = optimize(1, 180, 0);
    assertEquals(0, state.angle.getDegrees(), 1e-6, "the wheel stays where it is");
    assertEquals(-1, state.velocity, 1e-9, "and drives the other way");
  }

  @Test
  void turningWheelIsNotReversedBackAndForth() {
    // The wheel is still near 0 degrees, turning toward a target near 90 degrees from it that moves
    // from loop to loop, as when a path starts off in a new direction. Chosen by the wheel's angle
    // alone, the target's two orientations would swap from loop to loop.
    Rotation2d last = optimize(1, 110, 0).angle;
    for (double target : new double[] {70, 112, 68, 110, 72, 88}) {
      final Rotation2d commanded = optimize(1, target, 0).angle;
      assertTrue(
          Math.abs(commanded.minus(last).getDegrees()) < 90,
          "commanded "
              + commanded.getDegrees()
              + " degrees right after "
              + last.getDegrees()
              + " for a target of "
              + target
              + " degrees");
      last = commanded;
    }
  }

  @Test
  void turnsAroundWhenTheOtherOrientationIsClearlyBetter() {
    assertEquals(
        -70,
        optimize(1, 110, 0).angle.getDegrees(),
        1e-6,
        "110 degrees is reached by turning back to -70 degrees");
    // The wheel has ended up at 90 degrees: -70 degrees is now 160 degrees away and 110 degrees 20.
    final SwerveModuleVelocity state = optimize(1, 110, 90);
    assertEquals(110, state.angle.getDegrees(), 1e-6, "turns to 110 degrees");
    assertEquals(1, state.velocity, 1e-9, "and drives forwards");
  }

  @Test
  void tooSlowToSteerByHoldsTheWheel() {
    final SwerveModuleVelocity held = optimize(0.05, 120, 30);
    assertEquals(30, held.angle.getDegrees(), 1e-6, "the wheel holds where it is");
    assertEquals(0, held.velocity, 1e-9, "and does not drive");
    // The next state is optimized from where the wheel was held: 140 degrees is 110 degrees from
    // it,
    // so the wheel turns 70 degrees the other way and drives backwards.
    final SwerveModuleVelocity next = optimize(1, 140, 30);
    assertEquals(-40, next.angle.getDegrees(), 1e-6, "turns to -40 degrees");
    assertEquals(-1, next.velocity, 1e-9, "and drives backwards");
  }

  @Test
  void withoutALastAngleOrHysteresisOptimizesLikeWpilib() {
    final SwerveModuleVelocity desired = new SwerveModuleVelocity(1, Rotation2d.fromDegrees(100));
    final SwerveModuleVelocity expected =
        new SwerveModuleVelocity(1, Rotation2d.fromDegrees(100)).optimize(new Rotation2d());
    assertEquals(
        expected,
        SwerveModuleVelocityWithAzimuth.optimize(desired, wheelAt(0), null),
        "no last angle");
    assertEquals(
        expected,
        SwerveModuleVelocityWithAzimuth.optimize(
            desired, wheelAt(0), Rotation2d.fromDegrees(100), Degrees.of(0)),
        "no hysteresis");
  }
}
