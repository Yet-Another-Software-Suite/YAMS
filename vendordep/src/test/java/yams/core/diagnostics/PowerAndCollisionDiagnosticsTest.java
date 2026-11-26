// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package yams.core.diagnostics;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.Seconds;
import static org.wpilib.units.Units.Volts;

import java.util.SplittableRandom;
import org.junit.jupiter.api.Test;

class PowerAndCollisionDiagnosticsTest {

  @Test
  void powerMonitorIntegratesAmpHoursAndDetectsTotalAndChannelOvercurrent() {
    PowerHealthMonitor monitor =
        new PowerHealthMonitor()
            .withBatteryCapacityAmpHours(18.0)
            .withTotalCurrentAlert(Amps.of(120.0))
            .withChannelCurrentAlert(Amps.of(40.0));

    double[] channels = {12.0, 45.5, 18.0, 22.0};
    // Prime at t = 0 s, then draw 180 A for 36 s (1.8 Ah = 10% of 18 Ah)
    monitor.update(0.0, 12.6, 180.0, channels);
    PowerHealthMonitor.PowerStatus last = null;
    for (int i = 1; i <= 180; i++) {
      last = monitor.update(i * 0.2, 12.6, 180.0, channels);
    }

    assertEquals(1.8, last.consumedAmpHours(), 1e-6);
    assertEquals(90.0, last.estimatedBatteryPercent(), 1e-6);
    assertTrue(last.totalCurrentAlert());
    assertTrue(last.channelCurrentAlert());
    assertEquals(45.5, last.maxChannelCurrentAmps(), 1e-9);
    assertEquals(1, last.hottestChannelIndex());
    assertEquals(1.0, last.powerThrottleScale(), 1e-9);
  }

  @Test
  void powerMonitorAppliesLinearThrottleRampAndCountsBrownoutCycles() {
    PowerHealthMonitor monitor =
        new PowerHealthMonitor()
            .withThrottleVoltageRange(Volts.of(10.5), Volts.of(8.0), 0.5)
            .withBrownoutVoltage(Volts.of(6.8));

    assertEquals(1.0, monitor.update(0.00, 11.0, 20.0).powerThrottleScale(), 1e-9);
    assertEquals(0.75, monitor.update(0.02, 9.25, 40.0).powerThrottleScale(), 1e-9);
    assertEquals(0.5, monitor.update(0.04, 7.5, 90.0).powerThrottleScale(), 1e-9);

    // Three distinct brownout dips (< 6.8 V) with clean recovery in between
    PowerHealthMonitor.PowerStatus b1 = monitor.update(0.06, 6.4, 165.0);
    assertTrue(b1.brownoutActive());
    assertEquals(1, b1.brownoutCount());
    assertEquals(1, monitor.update(0.08, 6.3, 170.0).brownoutCount()); // same cycle

    assertFalse(monitor.update(0.10, 11.8, 25.0).brownoutActive());
    assertEquals(2, monitor.update(0.12, 6.5, 160.0).brownoutCount());
    monitor.update(0.14, 11.9, 25.0);
    assertEquals(3, monitor.update(0.16, 6.6, 155.0).brownoutCount());

    monitor.reset();
    PowerHealthMonitor.PowerStatus reset = monitor.update(0.20, 12.5, 10.0);
    assertEquals(0, reset.brownoutCount());
    assertEquals(0.0, reset.consumedAmpHours(), 1e-9);
  }

  @Test
  void powerMonitorDebouncesLowVoltageWarningAndEmergency() {
    PowerHealthMonitor monitor =
        new PowerHealthMonitor()
            .withLowVoltageThresholds(
                Volts.of(12.3), Volts.of(11.8), Seconds.of(1.0), Seconds.of(0.5));

    // Step in 0.2s increments so dt clamping (0.25s max) does not clip elapsed debounce time
    monitor.update(0.0, 11.6, 30.0);
    assertFalse(monitor.update(0.2, 11.6, 30.0).lowVoltageEmergency());
    assertFalse(monitor.update(0.4, 11.6, 30.0).lowVoltageEmergency());
    PowerHealthMonitor.PowerStatus at06 = monitor.update(0.6, 11.6, 30.0);
    assertFalse(at06.lowVoltageWarning());
    assertTrue(at06.lowVoltageEmergency());

    monitor.update(0.8, 11.6, 30.0);
    monitor.update(1.0, 11.6, 30.0);
    PowerHealthMonitor.PowerStatus at12 = monitor.update(1.2, 11.6, 30.0);
    assertTrue(at12.lowVoltageWarning());
    assertTrue(at12.lowVoltageEmergency());

    // Recovering above 12.3 V clears both immediately
    PowerHealthMonitor.PowerStatus recovered = monitor.update(1.4, 12.5, 15.0);
    assertFalse(recovered.lowVoltageWarning());
    assertFalse(recovered.lowVoltageEmergency());
  }

  @Test
  void collisionMonitorIsGravityInvariantUnderStaticTilt() {
    CollisionJerkMonitor monitor = new CollisionJerkMonitor();
    double tiltRad = Math.toRadians(25.0);
    double axG = Math.sin(tiltRad);
    double azG = Math.cos(tiltRad);

    CollisionJerkMonitor.ImpactStatus s0 = monitor.updateG(0.0, axG, 0.0, azG);
    CollisionJerkMonitor.ImpactStatus s1 = monitor.updateG(0.02, axG, 0.0, azG);

    assertEquals(0.0, s0.impactG(), 1e-12);
    assertEquals(0.0, s1.impactG(), 1e-12);
    assertEquals(0.0, s1.jerkGPerSec(), 1e-12);
    assertFalse(s1.impactDetected());
    assertFalse(s1.bumpedRecently());
  }

  @Test
  void collisionMonitorDetectsImpactAndJerkAndHoldsBumpedRecentlyLatch() {
    CollisionJerkMonitor monitor =
        new CollisionJerkMonitor()
            .withImpactThresholdG(0.5)
            .withJerkThresholdGPerSec(25.0)
            .withBumpedHoldDuration(Seconds.of(0.5));

    // Nominal stationary sample
    assertFalse(monitor.updateG(1.00, 0.0, 0.0, 1.0).impactDetected());

    // Hard 1.2 g lateral hit -> triggers impactG and jerk
    CollisionJerkMonitor.ImpactStatus hit = monitor.updateG(1.02, 1.2, 0.0, 1.0);
    assertTrue(hit.impactDetected());
    assertTrue(hit.bumpedRecently());
    assertTrue(hit.impactG() > 0.5);
    assertTrue(hit.jerkGPerSec() > 25.0);

    // 0.30 s later -> still latched as bumpedRecently
    CollisionJerkMonitor.ImpactStatus hold = monitor.updateG(1.32, 0.0, 0.0, 1.0);
    assertFalse(hold.impactDetected());
    assertTrue(hold.bumpedRecently());

    // 0.60 s later -> latch expires cleanly
    CollisionJerkMonitor.ImpactStatus cleared = monitor.updateG(1.62, 0.0, 0.0, 1.0);
    assertFalse(cleared.bumpedRecently());

    // m/s^2 overload works identically
    CollisionJerkMonitor.ImpactStatus mps2Hit =
        monitor.updateMetersPerSecondSquared(
            2.00, 12.0, 0.0, CollisionJerkMonitor.STANDARD_GRAVITY_MPS2);
    assertTrue(mps2Hit.impactDetected());

    monitor.reset();
    assertFalse(monitor.updateG(2.02, 0.0, 0.0, 1.0).bumpedRecently());
  }

  @Test
  void invalidConfigurationsThrowIllegalArgumentException() {
    var power = new PowerHealthMonitor();
    assertThrows(IllegalArgumentException.class, () -> power.withBatteryCapacityAmpHours(0.0));
    assertThrows(IllegalArgumentException.class, () -> power.withTotalCurrentAlert(Amps.of(-5.0)));
    assertThrows(IllegalArgumentException.class, () -> power.withChannelCurrentAlert(Amps.of(0.0)));
    assertThrows(
        IllegalArgumentException.class,
        () ->
            power.withLowVoltageThresholds(
                Volts.of(11.0), Volts.of(12.0), Seconds.of(1.0), Seconds.of(1.0)));
    assertThrows(
        IllegalArgumentException.class,
        () -> power.withThrottleVoltageRange(Volts.of(8.0), Volts.of(10.5), 0.5));
    assertThrows(
        IllegalArgumentException.class,
        () -> power.withThrottleVoltageRange(Volts.of(10.5), Volts.of(8.0), 0.0));

    var col = new CollisionJerkMonitor();
    assertThrows(IllegalArgumentException.class, () -> col.withImpactThresholdG(0.0));
    assertThrows(IllegalArgumentException.class, () -> col.withJerkThresholdGPerSec(-1.0));
    assertThrows(
        IllegalArgumentException.class, () -> col.withBumpedHoldDuration(Seconds.of(-0.1)));
    assertThrows(IllegalArgumentException.class, () -> col.withGravityMps2(0.0));
  }

  @Test
  void seededPropertyFuzzPowerMonitorMaintainsBoundsAndMonotonicAmpHours() {
    var rng = new SplittableRandom(20271125L);
    var monitor =
        new PowerHealthMonitor().withThrottleVoltageRange(Volts.of(10.5), Volts.of(8.0), 0.45);
    double prevAh = 0.0;
    double t = 0.0;

    for (int i = 0; i < 1_000; i++) {
      t += rng.nextDouble(0.005, 0.05);
      double v = rng.nextDouble(5.5, 13.2);
      double amps = rng.nextDouble(0.0, 220.0);
      var st = monitor.update(t, v, amps);
      assertTrue(Double.isFinite(st.consumedAmpHours()) && st.consumedAmpHours() >= prevAh);
      assertTrue(st.estimatedBatteryPercent() >= 0.0 && st.estimatedBatteryPercent() <= 100.0);
      assertTrue(st.powerThrottleScale() >= 0.45 - 1e-12 && st.powerThrottleScale() <= 1.0 + 1e-12);
      prevAh = st.consumedAmpHours();
    }
  }

  @Test
  void seededPropertyFuzzCollisionMonitorNeverFalseTriggersOnArbitraryStaticGravityOrientations() {
    var rng = new SplittableRandom(20271126L);
    var monitor = new CollisionJerkMonitor();

    for (int i = 0; i < 1_000; i++) {
      monitor.reset();
      double theta = rng.nextDouble(0.0, Math.PI);
      double phi = rng.nextDouble(0.0, 2.0 * Math.PI);
      double ax = Math.sin(theta) * Math.cos(phi);
      double ay = Math.sin(theta) * Math.sin(phi);
      double az = Math.cos(theta);

      var st = monitor.updateG(10.0, ax, ay, az);
      assertEquals(0.0, st.impactG(), 1e-12);
      assertFalse(st.impactDetected());
      assertFalse(st.bumpedRecently());
    }
  }
}
