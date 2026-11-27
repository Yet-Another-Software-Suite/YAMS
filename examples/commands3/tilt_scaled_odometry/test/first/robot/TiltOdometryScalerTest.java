// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package first.robot;

import static org.junit.jupiter.api.Assertions.*;
import static org.wpilib.units.Units.Degrees;

import java.util.SplittableRandom;
import org.junit.jupiter.api.Test;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.kinematics.SwerveModulePosition;

class TiltOdometryScalerTest {
  private static final long FUZZ_SEED = 202702782L;
  private static final int FUZZ_TRIALS = 50;

  @Test
  void cosineProjectionPreservesFlatRampTravelWhileLinearRampDampsAggressively() {
    TiltOdometryScaler cosScaler =
        new TiltOdometryScaler(
            TiltOdometryScaler.ScalingMode.COSINE_PROJECTION,
            Degrees.of(2.0),
            Degrees.of(25.0),
            Degrees.of(10.0));
    TiltOdometryScaler linearScaler =
        new TiltOdometryScaler(
            TiltOdometryScaler.ScalingMode.LINEAR_TRUST_RAMP,
            Degrees.of(2.0),
            Degrees.of(25.0),
            Degrees.of(10.0));

    Rotation3d tilt10 = new Rotation3d(0.0, Math.toRadians(10.0), 0.0);
    Rotation3d tilt15 = new Rotation3d(0.0, Math.toRadians(15.0), 0.0);

    assertEquals(Math.cos(Math.toRadians(10.0)), cosScaler.computeScale(tilt10), 1e-6);
    assertEquals(Math.cos(Math.toRadians(15.0)), cosScaler.computeScale(tilt15), 1e-6);
    assertEquals(1.0 - 8.0 / 23.0, linearScaler.computeScale(tilt10), 1e-6);
    assertEquals(1.0 - 13.0 / 23.0, linearScaler.computeScale(tilt15), 1e-6);

    assertTrue(cosScaler.shouldAcceptVision(tilt10));
    assertFalse(cosScaler.shouldAcceptVision(tilt15));
  }

  @Test
  void scalesIncrementalWheelDeltasAndResetsCleanly() {
    TiltOdometryScaler scaler =
        new TiltOdometryScaler(
            TiltOdometryScaler.ScalingMode.COSINE_PROJECTION,
            Degrees.of(2.0),
            Degrees.of(25.0),
            Degrees.of(10.0));
    SwerveModulePosition[] p0 = new SwerveModulePosition[] {new SwerveModulePosition(0.0, Rotation2d.ZERO)};
    scaler.reset(p0);

    Rotation3d tilt60 = new Rotation3d(0.0, Math.toRadians(60.0), 0.0);
    SwerveModulePosition[] p1 =
        scaler.scalePositions(tilt60, new SwerveModulePosition[] {new SwerveModulePosition(2.0, Rotation2d.ZERO)});
    assertEquals(1.0, p1[0].distance, 1e-6, "2.0 m at 60 deg tilt projects to 1.0 m flat");

    scaler.reset(new SwerveModulePosition[] {new SwerveModulePosition(5.0, Rotation2d.ZERO)});
    SwerveModulePosition[] p2 =
        scaler.scalePositions(Rotation3d.ZERO, new SwerveModulePosition[] {new SwerveModulePosition(6.0, Rotation2d.ZERO)});
    assertEquals(6.0, p2[0].distance, 1e-6);
  }

  @Test
  void fuzzBothModesAlwaysStayWithinZeroToOneAndIgnoreYaw() {
    TiltOdometryScaler cosScaler =
        new TiltOdometryScaler(
            TiltOdometryScaler.ScalingMode.COSINE_PROJECTION,
            Degrees.of(2.0),
            Degrees.of(25.0),
            Degrees.of(10.0));
    TiltOdometryScaler linearScaler =
        new TiltOdometryScaler(
            TiltOdometryScaler.ScalingMode.LINEAR_TRUST_RAMP,
            Degrees.of(2.0),
            Degrees.of(25.0),
            Degrees.of(10.0));
    SplittableRandom rng = new SplittableRandom(FUZZ_SEED);

    for (int i = 0; i < FUZZ_TRIALS; i++) {
      double roll = rng.nextDouble(-0.7, 0.7);
      double pitch = rng.nextDouble(-0.7, 0.7);
      double yawA = rng.nextDouble(-Math.PI, Math.PI);
      double yawB = rng.nextDouble(-Math.PI, Math.PI);
      Rotation3d rotA = new Rotation3d(roll, pitch, yawA);
      Rotation3d rotB = new Rotation3d(roll, pitch, yawB);

      double cA = cosScaler.computeScale(rotA);
      double cB = cosScaler.computeScale(rotB);
      double lA = linearScaler.computeScale(rotA);
      assertTrue(cA >= 0.0 && cA <= 1.0);
      assertTrue(lA >= 0.0 && lA <= 1.0);
      assertEquals(cA, cB, 1e-9, "3D tilt scale must be invariant to yaw");
    }
  }
}
