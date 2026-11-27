// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package first.robot;

import static first.robot.Constants.CollisionConstants.MONITOR_CONFIG;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.wpilib.units.Units.Gs;
import static org.wpilib.units.Units.Seconds;

import java.util.SplittableRandom;
import org.junit.jupiter.api.Test;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Translation3d;

class CollisionJerkMonitorTest {
  @Test
  void detectsSideHitThatNaiveMagnitudeMisses() {
    CollisionJerkMonitor monitor = new CollisionJerkMonitor(MONITOR_CONFIG);
    Rotation3d flat = new Rotation3d();

    assertFalse(monitor.update(Seconds.of(0.00), flat, Gs.of(0.0), Gs.of(0.0), Gs.of(1.0)));

    // 0.9 g side hit: naive | ||a|| - 1 g | is only 0.345 g, but 3D gravity subtraction recovers
    // 0.90 g and 45 g/s jerk over 20 ms -> detected cleanly.
    assertTrue(monitor.update(Seconds.of(0.02), flat, Gs.of(0.0), Gs.of(0.90), Gs.of(1.0)));
    assertEquals(0.90, monitor.getLastLinearAccelGs(), 1e-6);
    assertEquals(45.0, monitor.getLastJerkGsPerSecond(), 1e-6);
    assertEquals(1, monitor.getCollisionCount());
  }

  @Test
  void ignoresSmoothHardDrivingAndTiltedRest() {
    CollisionJerkMonitor monitor = new CollisionJerkMonitor(MONITOR_CONFIG);
    Rotation3d flat = new Rotation3d();

    // Smooth 1.2 g swerve acceleration over 1.0 s (peak jerk ~3.8 g/s) must NOT trigger collision.
    for (int i = 0; i <= 50; i++) {
      double t = i * 0.02;
      double ax = 1.2 * Math.sin(Math.PI * t);
      assertFalse(monitor.update(Seconds.of(t), flat, Gs.of(ax), Gs.of(0.0), Gs.of(1.0)));
    }
    assertEquals(0, monitor.getCollisionCount());

    // Stationary on a 20-degree pitch ramp has zero 3D linear acceleration after gravity removal.
    Rotation3d tilted = new Rotation3d(0.0, Math.toRadians(20.0), 0.0);
    Translation3d gBody = new Translation3d(0.0, 0.0, 1.0).rotateBy(new Rotation3d(tilted.getQuaternion().inverse()));
    Translation3d linear =
        CollisionJerkMonitor.removeGravity(
            tilted, Gs.of(gBody.getX()), Gs.of(gBody.getY()), Gs.of(gBody.getZ()));
    assertEquals(0.0, linear.getNorm(), 1e-9);
  }

  @Test
  void preservesBumpedRecentlyLatchOnNonFiniteImuSamples() {
    CollisionJerkMonitor monitor = new CollisionJerkMonitor(MONITOR_CONFIG);
    Rotation3d flat = new Rotation3d();

    monitor.update(Seconds.of(0.00), flat, Gs.of(0.0), Gs.of(0.0), Gs.of(1.0));
    assertTrue(monitor.update(Seconds.of(0.02), flat, Gs.of(0.0), Gs.of(0.95), Gs.of(1.0)));

    // Non-finite samples must NOT clear bumpedRecently.
    assertTrue(
        monitor.update(Seconds.of(0.04), flat, Gs.of(Double.NaN), Gs.of(0.0), Gs.of(1.0)));
    assertTrue(
        monitor.update(
            Seconds.of(Double.NaN), flat, Gs.of(0.0), Gs.of(0.0), Gs.of(Double.POSITIVE_INFINITY)));
    assertTrue(monitor.isBumpedRecently());

    // After the 0.60 s latch window expires on a valid sample, bumpedRecently clears.
    assertFalse(monitor.update(Seconds.of(0.70), flat, Gs.of(0.0), Gs.of(0.0), Gs.of(1.0)));
    assertEquals(1, monitor.getCollisionCount());
  }

  @Test
  void fuzzRandomAttitudesAndAccelProfilesRemainFiniteAndMonotonic() {
    CollisionJerkMonitor monitor = new CollisionJerkMonitor(MONITOR_CONFIG);
    SplittableRandom rng = new SplittableRandom(20270504L);
    double t = 0.0;
    int prevCollisions = 0;

    for (int i = 0; i < 2000; i++) {
      t += 0.02;
      Rotation3d attitude =
          new Rotation3d(
              rng.nextDouble(-0.4, 0.4), rng.nextDouble(-0.4, 0.4), rng.nextDouble(-Math.PI, Math.PI));
      double ax = (i % 89 == 0) ? Double.NaN : rng.nextDouble(-1.5, 1.5);
      double ay = rng.nextDouble(-1.5, 1.5);
      double az = rng.nextDouble(0.2, 1.8);

      monitor.update(Seconds.of(t), attitude, Gs.of(ax), Gs.of(ay), Gs.of(az));
      assertTrue(Double.isFinite(monitor.getLastLinearAccelGs()));
      assertTrue(Double.isFinite(monitor.getLastJerkGsPerSecond()));
      assertTrue(monitor.getCollisionCount() >= prevCollisions);
      prevCollisions = monitor.getCollisionCount();
    }
  }
}
