// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package first.robot;

import static first.robot.Constants.PowerHealthConstants.MONITOR_CONFIG;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.Seconds;
import static org.wpilib.units.Units.Volts;

import first.robot.PowerHealthMonitor.PowerState;
import java.util.SplittableRandom;
import org.junit.jupiter.api.Test;

class PowerHealthMonitorTest {
  @Test
  void ignoresNonFiniteVoltageAndCurrentWithoutTrippingBrownout() {
    PowerHealthMonitor monitor = new PowerHealthMonitor(MONITOR_CONFIG);
    assertEquals(PowerState.NOMINAL, monitor.update(Seconds.of(0.0), Volts.of(12.3), Amps.of(20.0)));
    assertEquals(1.0, monitor.getThrottleScale(), 1e-9);

    // Non-finite voltage or timestamp must NOT become 0 V or trip a brownout.
    assertEquals(PowerState.NOMINAL, monitor.update(Seconds.of(0.02), Volts.of(Double.NaN), Amps.of(25.0)));
    assertEquals(
        PowerState.NOMINAL,
        monitor.update(Seconds.of(0.04), Volts.of(Double.POSITIVE_INFINITY), Amps.of(25.0)));
    assertEquals(
        PowerState.NOMINAL,
        monitor.update(Seconds.of(Double.NaN), Volts.of(5.0), Amps.of(25.0)));

    assertEquals(0, monitor.getBrownoutCount());
    assertEquals(12.3, monitor.getLastValidVoltage().in(Volts), 1e-9);
    assertEquals(1.0, monitor.getThrottleScale(), 1e-9);

    // Non-finite current must not corrupt amp-hour integration.
    monitor.update(Seconds.of(1.0), Volts.of(12.2), Amps.of(Double.NaN));
    assertEquals(0.0, monitor.getConsumedAmpHours(), 1e-9);
  }

  @Test
  void hysteresisPreventsDebounceResetAndBrownoutDoubleCounting() {
    PowerHealthMonitor monitor = new PowerHealthMonitor(MONITOR_CONFIG);
    monitor.update(Seconds.of(0.00), Volts.of(12.2), Amps.of(10.0));

    // Drop below 10.5 V for 0.15 s (not yet 0.25 s debounce).
    monitor.update(Seconds.of(0.15), Volts.of(10.2), Amps.of(60.0));
    assertEquals(PowerState.NOMINAL, monitor.getState());

    // Brief spike to 10.65 V (above 10.5 V, below 10.8 V hysteresis release) must NOT reset timer.
    monitor.update(Seconds.of(0.20), Volts.of(10.65), Amps.of(50.0));
    assertEquals(PowerState.NOMINAL, monitor.getState());

    // Another 0.12 s below 10.5 V completes 0.27 s >= 0.25 s and triggers WARNING.
    assertEquals(
        PowerState.LOW_VOLTAGE_WARNING,
        monitor.update(Seconds.of(0.32), Volts.of(10.3), Amps.of(65.0)));

    // Hovering around the 6.75 V brownout rail inside 0.30 V hysteresis counts exactly 1 brownout.
    assertEquals(
        PowerState.BROWNOUT, monitor.update(Seconds.of(0.40), Volts.of(6.60), Amps.of(120.0)));
    assertEquals(
        PowerState.BROWNOUT, monitor.update(Seconds.of(0.42), Volts.of(6.90), Amps.of(100.0)));
    assertEquals(
        PowerState.BROWNOUT, monitor.update(Seconds.of(0.44), Volts.of(6.55), Amps.of(120.0)));
    assertEquals(1, monitor.getBrownoutCount());

    // Only once voltage clears 6.75 + 0.30 = 7.05 V does brownout release.
    monitor.update(Seconds.of(0.50), Volts.of(11.5), Amps.of(15.0));
    assertEquals(PowerState.NOMINAL, monitor.getState());
    assertEquals(1, monitor.getBrownoutCount());
  }

  @Test
  void coulombCountsFromExplicitInitialSoCAndClampsToZero() {
    PowerHealthMonitor.Config midEventConfig =
        new PowerHealthMonitor.Config(
            Volts.of(10.5),
            Volts.of(9.5),
            Volts.of(7.0),
            Volts.of(0.30),
            Seconds.of(0.25),
            Seconds.of(0.15),
            Volts.of(11.0),
            Volts.of(9.0),
            0.35,
            18.0,
            0.80);
    PowerHealthMonitor monitor = new PowerHealthMonitor(midEventConfig);
    monitor.update(Seconds.of(0.0), Volts.of(12.2), Amps.of(0.0));
    assertEquals(0.80, monitor.getRemainingStateOfCharge(), 1e-9);

    // 180 A for 36 s = 1.8 Ah = 10% of an 18 Ah battery -> 0.80 - 0.10 = 0.70.
    monitor.update(Seconds.of(36.0), Volts.of(11.8), Amps.of(180.0));
    assertEquals(1.8, monitor.getConsumedAmpHours(), 1e-9);
    assertEquals(0.70, monitor.getRemainingStateOfCharge(), 1e-9);

    // Custom 7.0 V brownout rail trips at 6.95 V.
    assertEquals(
        PowerState.BROWNOUT, monitor.update(Seconds.of(36.02), Volts.of(6.95), Amps.of(180.0)));

    // Extreme draw clamps SoC cleanly at 0.0.
    monitor.update(Seconds.of(3636.02), Volts.of(11.5), Amps.of(100.0));
    assertEquals(0.0, monitor.getRemainingStateOfCharge(), 1e-9);
  }

  @Test
  void smoothThrottleScaleInterpolatesAndClamps() {
    PowerHealthMonitor monitor = new PowerHealthMonitor(MONITOR_CONFIG);
    monitor.update(Seconds.of(0.0), Volts.of(12.5), Amps.of(10.0));
    assertEquals(1.0, monitor.getThrottleScale(), 1e-9);

    monitor.update(Seconds.of(0.02), Volts.of(10.0), Amps.of(80.0));
    assertEquals(0.675, monitor.getThrottleScale(), 1e-9);

    monitor.update(Seconds.of(0.04), Volts.of(8.5), Amps.of(120.0));
    assertEquals(0.35, monitor.getThrottleScale(), 1e-9);
  }

  @Test
  void fuzzRandomVoltageAndCurrentProfilesStayBoundedAndMonotonic() {
    PowerHealthMonitor monitor = new PowerHealthMonitor(MONITOR_CONFIG);
    SplittableRandom rng = new SplittableRandom(20270503L);
    double t = 0.0;
    int prevBrownouts = 0;
    double prevSoC = 1.0;

    for (int i = 0; i < 2000; i++) {
      t += 0.02;
      double v = (i % 97 == 0) ? Double.NaN : 6.0 + rng.nextDouble() * 7.2;
      double a = (i % 113 == 0) ? Double.NaN : rng.nextDouble() * 160.0;
      PowerState state = monitor.update(Seconds.of(t), Volts.of(v), Amps.of(a));

      assertNotNull(state);
      assertNotNull(monitor.getSeveritySummary());
      assertTrue(
          monitor.getThrottleScale() >= 0.35 - 1e-9 && monitor.getThrottleScale() <= 1.0 + 1e-9);
      assertTrue(monitor.getBrownoutCount() >= prevBrownouts);
      assertTrue(monitor.getRemainingStateOfCharge() <= prevSoC + 1e-9);
      assertTrue(monitor.getRemainingStateOfCharge() >= 0.0);

      prevBrownouts = monitor.getBrownoutCount();
      prevSoC = monitor.getRemainingStateOfCharge();
    }
  }
}
