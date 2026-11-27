// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package first.robot;

import static org.junit.jupiter.api.Assertions.*;
import static org.wpilib.units.Units.MetersPerSecond;
import static org.wpilib.units.Units.RadiansPerSecond;

import java.util.SplittableRandom;
import org.junit.jupiter.api.Test;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.math.kinematics.SwerveDriveKinematics;
import org.wpilib.math.kinematics.SwerveModuleVelocity;

class SkidDetectorTest {
  private static final Translation2d[] SQUARE_06M = new Translation2d[] {
    new Translation2d(0.3, 0.3),
    new Translation2d(0.3, -0.3),
    new Translation2d(-0.3, 0.3),
    new Translation2d(-0.3, -0.3)
  };
  private static final long FUZZ_SEED = 20270278L;
  private static final int FUZZ_TRIALS = 50;

  @Test
  void detectsTwoLeftWheelsSlippingWithTrueGyroRate() {
    // 0.6 m square chassis driving straight at 1.0 m/s; both left wheels slip to 1.6 m/s.
    // Wheel-fit omega would hide this at 1.26, but true gyro omega (0 rad/s) gives 1.60.
    SkidDetector detector = new SkidDetector(SQUARE_06M, 1.35, MetersPerSecond.of(0.20));
    SwerveModuleVelocity[] states = new SwerveModuleVelocity[] {
      new SwerveModuleVelocity(1.6, Rotation2d.ZERO),
      new SwerveModuleVelocity(1.0, Rotation2d.ZERO),
      new SwerveModuleVelocity(1.6, Rotation2d.ZERO),
      new SwerveModuleVelocity(1.0, Rotation2d.ZERO)
    };

    SkidDetector.Result res = detector.detect(states, RadiansPerSecond.of(0.0));
    assertTrue(res.isSkidding());
    assertEquals(1.60, res.ratio(), 1e-6);
    assertTrue(res.isModuleSlipping(0));
    assertFalse(res.isModuleSlipping(1));
    assertTrue(res.isModuleSlipping(2));
    assertFalse(res.isModuleSlipping(3));
    assertEquals(2, res.slippingModuleCount());
  }

  @Test
  void isolatesSingleSlippingWheelWhileKeepingGrippingWheelsActive() {
    SkidDetector detector = new SkidDetector(SQUARE_06M, 1.35, MetersPerSecond.of(0.20));
    SwerveModuleVelocity[] states = new SwerveModuleVelocity[] {
      new SwerveModuleVelocity(2.2, Rotation2d.ZERO),
      new SwerveModuleVelocity(1.2, Rotation2d.ZERO),
      new SwerveModuleVelocity(1.2, Rotation2d.ZERO),
      new SwerveModuleVelocity(1.2, Rotation2d.ZERO)
    };

    SkidDetector.Result res = detector.detect(states, RadiansPerSecond.of(0.0));
    assertTrue(res.isSkidding());
    assertEquals(1, res.slippingModuleCount());
    assertTrue(res.isModuleSlipping(0));
    assertFalse(res.isModuleSlipping(1));
    assertFalse(res.isModuleSlipping(2));
    assertFalse(res.isModuleSlipping(3));
  }

  @Test
  void slowestModuleDeadbandPreventsLowSpeedFalseSkids() {
    SkidDetector detector = new SkidDetector(SQUARE_06M, 1.35, MetersPerSecond.of(0.20));
    // 0.12 vs 0.08 m/s would be a 1.5 ratio, but slowest module is below 0.20 m/s deadband.
    SwerveModuleVelocity[] states = new SwerveModuleVelocity[] {
      new SwerveModuleVelocity(0.12, Rotation2d.ZERO),
      new SwerveModuleVelocity(0.08, Rotation2d.ZERO),
      new SwerveModuleVelocity(0.11, Rotation2d.ZERO),
      new SwerveModuleVelocity(0.09, Rotation2d.ZERO)
    };

    SkidDetector.Result res = detector.detect(states, RadiansPerSecond.of(0.0));
    assertFalse(res.isSkidding());
    assertEquals(1.0, res.ratio(), 1e-6);
    assertEquals(0, res.slippingModuleCount());
  }

  @Test
  void fuzzPureRigidBodyMotionNeverFlagsFalseSkid() {
    SwerveDriveKinematics kinematics = new SwerveDriveKinematics(SQUARE_06M);
    SkidDetector detector = new SkidDetector(SQUARE_06M, 1.35, MetersPerSecond.of(0.20));
    SplittableRandom rng = new SplittableRandom(FUZZ_SEED);

    for (int i = 0; i < FUZZ_TRIALS; i++) {
      double vx = rng.nextDouble(-3.5, 3.5);
      double vy = rng.nextDouble(-3.5, 3.5);
      double omega = rng.nextDouble(-4.0, 4.0);
      SwerveModuleVelocity[] states =
          kinematics.toSwerveModuleVelocities(new ChassisVelocities(vx, vy, omega));
      SkidDetector.Result res = detector.detect(states, RadiansPerSecond.of(omega));
      assertFalse(res.isSkidding(), "Rigid-body motion flagged false skid on trial " + i);
      assertEquals(1.0, res.ratio(), 1e-5);
    }
  }
}
