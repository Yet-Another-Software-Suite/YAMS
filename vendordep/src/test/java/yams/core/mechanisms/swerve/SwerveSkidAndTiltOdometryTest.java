// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package yams.core.mechanisms.swerve;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.MetersPerSecond;

import java.util.SplittableRandom;
import org.junit.jupiter.api.Test;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.math.kinematics.SwerveDriveKinematics;
import org.wpilib.math.kinematics.SwerveModulePosition;
import org.wpilib.math.kinematics.SwerveModuleVelocity;
import yams.commands2.config.SwerveDriveConfig;

/**
 * Unit and seeded property-based tests for {@link SkidDetector}, {@link TiltOdometryScaler}, and
 * their fluent configuration builders on {@link yams.core.mechanisms.config.SwerveDriveConfig}.
 */
public class SwerveSkidAndTiltOdometryTest {
  private static final double kTolerance = 1e-6;

  private static final Translation2d[] kModuleLocations = {
    new Translation2d(0.30, 0.30),
    new Translation2d(0.30, -0.30),
    new Translation2d(-0.30, 0.30),
    new Translation2d(-0.30, -0.30)
  };

  private static final SwerveDriveKinematics kKinematics =
      new SwerveDriveKinematics(kModuleLocations);

  @Test
  void pureTranslationReportsUnitSkidRatio() {
    SkidDetector detector = new SkidDetector(1.35);
    SwerveModuleVelocity[] states =
        kKinematics.toSwerveModuleVelocities(new ChassisVelocities(3.0, -1.5, 0.0));
    SkidDetector.SkidState result = detector.calculate(states, kModuleLocations, 0.0);

    assertEquals(1.0, result.skidRatio(), kTolerance);
    assertFalse(result.skidding());
    assertEquals(Math.hypot(3.0, -1.5), result.minTranslationalSpeedMps(), kTolerance);
  }

  @Test
  void pureRotationTriggersDeadbandWithoutDivisionByZero() {
    SkidDetector detector = new SkidDetector(1.35);
    SwerveModuleVelocity[] states =
        kKinematics.toSwerveModuleVelocities(new ChassisVelocities(0.0, 0.0, 4.0));
    SkidDetector.SkidState result = detector.calculate(states, kModuleLocations, 4.0);

    assertEquals(1.0, result.skidRatio(), kTolerance);
    assertFalse(result.skidding());
    assertTrue(result.maxTranslationalSpeedMps() < 1e-6);
  }

  @Test
  void combinedTranslationAndRotationCancelsRotationalComponentExactly() {
    SkidDetector detector = new SkidDetector(1.35);
    ChassisVelocities cmd = new ChassisVelocities(2.4, 1.2, -2.5);
    SwerveModuleVelocity[] states = kKinematics.toSwerveModuleVelocities(cmd);
    SkidDetector.SkidState result = detector.calculate(states, kModuleLocations, cmd.omega);

    assertEquals(1.0, result.skidRatio(), kTolerance);
    assertFalse(result.skidding());
  }

  @Test
  void singleWheelSlipTriggersSkiddingWithExactRatio() {
    SkidDetector detector = new SkidDetector(1.35);
    SwerveModuleVelocity[] states =
        kKinematics.toSwerveModuleVelocities(new ChassisVelocities(2.0, 0.0, 0.0));
    // Front-left wheel slips at 2x speed while remaining 3 wheels read 2.0 m/s
    states[0] = new SwerveModuleVelocity(4.0, states[0].angle);

    SkidDetector.SkidState result = detector.calculate(states, kModuleLocations, 0.0);
    assertEquals(2.0, result.skidRatio(), kTolerance);
    assertTrue(result.skidding());
  }

  @Test
  void tiltScalerLinearRampAndYawInvariance() {
    TiltOdometryScaler scaler = new TiltOdometryScaler(Degrees.of(2.0), Degrees.of(25.0));

    // Flat and below 2 deg -> 1.0
    assertEquals(1.0, scaler.computeScale(new Rotation3d()), kTolerance);
    assertEquals(
        1.0, scaler.computeScale(new Rotation3d(0.0, Math.toRadians(2.0), 1.2)), kTolerance);

    // Midpoint (13.5 deg) -> 0.5 regardless of yaw
    assertEquals(0.5, scaler.computeScale(new Rotation3d(0.0, Math.toRadians(13.5), 0.0)), 1e-5);
    assertEquals(
        0.5,
        scaler.computeScale(new Rotation3d(Math.toRadians(-13.5), 0.0, Math.toRadians(145.0))),
        1e-5);

    // At and beyond 25 deg -> 0.0
    assertEquals(
        0.0, scaler.computeScale(new Rotation3d(0.0, Math.toRadians(25.0), 0.0)), kTolerance);
    assertEquals(
        0.0, scaler.computeScale(new Rotation3d(Math.toRadians(35.0), 0.0, -2.0)), kTolerance);

    // Vision max-tilt helper
    assertFalse(
        TiltOdometryScaler.isTiltExceeding(
            new Rotation3d(0.0, Math.toRadians(4.0), 0.0), Degrees.of(5.0)));
    assertTrue(
        TiltOdometryScaler.isTiltExceeding(
            new Rotation3d(Math.toRadians(4.0), Math.toRadians(4.0), 0.0), Degrees.of(5.0)));
  }

  @Test
  void tiltScalerScalesIncrementalDeltasAndResetsCleanly() {
    TiltOdometryScaler scaler = new TiltOdometryScaler(Degrees.of(2.0), Degrees.of(25.0));
    SwerveModulePosition[] p0 = positions(0.0);
    scaler.reset(p0);

    // 1. Flat drive +1.0 m -> full 1.0 m credit
    SwerveModulePosition[] s1 = scaler.scalePositions(new Rotation3d(), positions(1.0), false);
    assertEquals(1.0, s1[0].distance, kTolerance);

    // 2. Tilted 25 deg drive +0.5 m (raw = 1.5 m) -> 0.0 m credit (stays at 1.0 m)
    SwerveModulePosition[] s2 =
        scaler.scalePositions(
            new Rotation3d(0.0, Math.toRadians(25.0), 0.0), positions(1.5), false);
    assertEquals(1.0, s2[0].distance, kTolerance);
    assertEquals(0.0, scaler.getLastScale(), kTolerance);

    // 3. Return flat and drive +0.4 m (raw = 1.9 m) -> advances smoothly to 1.4 m (zero jump!)
    SwerveModulePosition[] s3 = scaler.scalePositions(new Rotation3d(), positions(1.9), false);
    assertEquals(1.4, s3[0].distance, kTolerance);

    // 4. Explicit skid rejection freezes delta even while flat
    SwerveModulePosition[] s4 = scaler.scalePositions(new Rotation3d(), positions(2.5), true);
    assertEquals(1.4, s4[0].distance, kTolerance);
    assertEquals(0.0, scaler.getLastScale(), kTolerance);

    // 5. Reset syncs scaled and raw baselines cleanly
    scaler.reset(positions(10.0));
    SwerveModulePosition[] s5 = scaler.scalePositions(new Rotation3d(), positions(10.2), false);
    assertEquals(10.2, s5[0].distance, kTolerance);
  }

  @Test
  void invalidConfigurationsThrowIllegalArgumentException() {
    assertThrows(IllegalArgumentException.class, () -> new SkidDetector(1.0));
    assertThrows(IllegalArgumentException.class, () -> new SkidDetector(0.8));
    assertThrows(
        IllegalArgumentException.class, () -> new SkidDetector(1.35, MetersPerSecond.of(-0.1)));
    assertThrows(
        IllegalArgumentException.class,
        () -> new TiltOdometryScaler(Degrees.of(10.0), Degrees.of(10.0)));
    assertThrows(
        IllegalArgumentException.class,
        () -> new TiltOdometryScaler(Degrees.of(-1.0), Degrees.of(20.0)));
    assertThrows(
        IllegalArgumentException.class,
        () -> new TiltOdometryScaler(Degrees.of(2.0), Degrees.of(95.0)));

    SwerveDriveConfig cfg = new SwerveDriveConfig();
    assertThrows(IllegalArgumentException.class, () -> cfg.withVisionMaxTilt(Degrees.of(0.0)));
    assertFalse(cfg.isSkidOdometryRejectionEnabled());
    cfg.withSkidDetection(1.4)
        .withTiltScaledOdometry(Degrees.of(2), Degrees.of(25))
        .withVisionMaxTilt(Degrees.of(5));
    assertTrue(cfg.getSkidDetector().isPresent());
    assertTrue(cfg.getTiltOdometryScaler().isPresent());
    assertTrue(cfg.getVisionMaxTilt().isPresent());
  }

  @Test
  void seededPropertyFuzzRigidBodyKinematicsNeverFalseTriggersSkid() {
    long seed = 2027060601L;
    SplittableRandom rng = new SplittableRandom(seed);
    SkidDetector detector = new SkidDetector(1.10, MetersPerSecond.of(0.10));

    for (int trial = 0; trial < 500; trial++) {
      ChassisVelocities cmd =
          new ChassisVelocities(
              rng.nextDouble(-5.0, 5.0), rng.nextDouble(-5.0, 5.0), rng.nextDouble(-8.0, 8.0));
      SwerveModuleVelocity[] states = kKinematics.toSwerveModuleVelocities(cmd);
      SkidDetector.SkidState res = detector.calculate(states, kModuleLocations, cmd.omega);
      assertEquals(
          1.0, res.skidRatio(), 1e-5, "False skid ratio on seed " + seed + " trial " + trial);
      assertFalse(res.skidding(), "False skid flag on seed " + seed + " trial " + trial);
    }
  }

  @Test
  void seededPropertyFuzzTiltScalerBoundedAndYawInvariant() {
    long seed = 2027060602L;
    SplittableRandom rng = new SplittableRandom(seed);
    TiltOdometryScaler scaler = new TiltOdometryScaler(Degrees.of(2.0), Degrees.of(25.0));

    for (int trial = 0; trial < 500; trial++) {
      double roll = rng.nextDouble(-Math.PI * 0.45, Math.PI * 0.45);
      double pitch = rng.nextDouble(-Math.PI * 0.45, Math.PI * 0.45);
      double yawA = rng.nextDouble(-Math.PI, Math.PI);
      double yawB = rng.nextDouble(-Math.PI, Math.PI);

      double scaleA = scaler.computeScale(new Rotation3d(roll, pitch, yawA));
      double scaleB = scaler.computeScale(new Rotation3d(roll, pitch, yawB));

      assertTrue(Double.isFinite(scaleA) && scaleA >= 0.0 && scaleA <= 1.0);
      assertEquals(
          scaleA, scaleB, 1e-12, "Scale depended on yaw on seed " + seed + " trial " + trial);
    }
  }

  private static SwerveModulePosition[] positions(double distanceMeters) {
    return new SwerveModulePosition[] {
      new SwerveModulePosition(distanceMeters, Rotation2d.ZERO),
      new SwerveModulePosition(distanceMeters, Rotation2d.ZERO),
      new SwerveModulePosition(distanceMeters, Rotation2d.ZERO),
      new SwerveModulePosition(distanceMeters, Rotation2d.ZERO)
    };
  }
}
