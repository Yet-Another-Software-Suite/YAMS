// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package yams.core.math;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.ValueSource;

/**
 * The {@link ADRCController} on its own, driving an ideal plant: a chain of {@code order}
 * integrators, {@code y^(order) = b u + d}, with a constant disturbance {@code d} it is not told
 * about.
 */
class ADRCControllerTest {
  private static final double kDt = 0.02;

  /** Plant simulation substeps per control period. */
  private static final int kSubsteps = 20;

  /**
   * A chain of integrators driven by {@code b u + d}.
   */
  private static final class Plant {
    private final double[] x;
    private final double b;
    private final double d;

    Plant(int order, double b, double d, double y0) {
      this.x = new double[order];
      this.x[0] = y0;
      this.b = b;
      this.d = d;
    }

    double output() {
      return x[0];
    }

    /** Hold {@code u} for one control period. */
    void step(double u) {
      final double h = kDt / kSubsteps;
      for (int s = 0; s < kSubsteps; s++) {
        final double top = b * u + d;
        for (int i = 0; i < x.length - 1; i++) {
          x[i] += x[i + 1] * h;
        }
        x[x.length - 1] += top * h;
      }
    }
  }

  private static ADRCController controller(int order, double settlingTime, double b0, double limit) {
    final ADRCController adrc = new ADRCController(order);
    adrc.initialize(settlingTime, 1.0, b0, -limit, limit, kDt, null, 0.0, 1.0, 0.0, 0.0, "none", null, false);
    return adrc;
  }

  /**
   * Run the loop, returning the largest error over the last second.
   */
  private static double run(ADRCController adrc, Plant plant, double setpoint, double seconds) {
    double worst = 0;
    final int steps = (int) Math.round(seconds / kDt);
    for (int i = 0; i < steps; i++) {
      plant.step(adrc.calculate(plant.output(), setpoint));
      if (i >= steps - (int) Math.round(1.0 / kDt)) {
        worst = Math.max(worst, Math.abs(plant.output() - setpoint));
      }
    }
    return worst;
  }

  @ParameterizedTest(name = "order {0}")
  @ValueSource(ints = {1, 2, 3, 4})
  void reachesItsSetpointAgainstAnUnknownDisturbance(int order) {
    final ADRCController adrc = controller(order, 1.0, 10, Double.POSITIVE_INFINITY);
    final Plant plant = new Plant(order, 10, -3, 0);
    final double worst = run(adrc, plant, 1.0, 6);
    assertTrue(worst < 1e-3, "order " + order + " should hold 1.0, but was " + worst + " from it");
    // The observer's last state is the disturbance it found.
    assertEquals(-3, adrc.getEstimatedDisturbance(), 0.05);
  }

  @ParameterizedTest(name = "order {0}")
  @ValueSource(ints = {1, 2})
  void reachesItsSetpointWithB0OffByAFactorOfTwo(int order) {
    for (double b0 : new double[] {5, 20}) {
      final ADRCController adrc = controller(order, 1.0, b0, Double.POSITIVE_INFINITY);
      final double worst = run(adrc, new Plant(order, 10, -3, 0), 1.0, 6);
      assertTrue(worst < 1e-3, "b0 " + b0 + ": should hold 1.0, but was " + worst + " from it");
    }
  }

  @Test
  void outputStaysWithinItsLimits() {
    final ADRCController adrc = controller(1, 0.2, 10, 2);
    final Plant plant = new Plant(1, 10, 0, 0);
    for (int i = 0; i < 100; i++) {
      final double u = adrc.calculate(plant.output(), 100);
      assertTrue(u >= -2 && u <= 2, "output " + u + " should stay within [-2, 2]");
      plant.step(u);
    }
    assertEquals(2, adrc.getLastOutput(), 1e-9);
  }

  /**
   * An actuator that applies less than it is asked for, as a motor controller does at its current
   * limit: told what was applied, the observer does not mistake the shortfall for a disturbance, and
   * the loop settles once the setpoint is within the actuator's reach.
   */
  @Test
  void settlesWhenTheActuatorAppliesLessThanItIsAskedFor() {
    final ADRCController adrc = controller(2, 0.5, 10, 12);
    final Plant plant = new Plant(2, 10, -3, 0);
    final double actuatorLimit = 3;
    double applied = 0;
    double worst = 0;
    for (int i = 0; i < 400; i++) {
      final double asked = adrc.calculate(plant.output(), 1.0, applied);
      applied = Math.max(-actuatorLimit, Math.min(actuatorLimit, asked));
      plant.step(applied);
      if (i >= 350) {
        worst = Math.max(worst, Math.abs(plant.output() - 1.0));
      }
    }
    assertTrue(worst < 1e-3, "should hold 1.0 through the actuator's limit, but was " + worst + " from it");
  }

  @Test
  void slewRateLimitsHowFastTheOutputChanges() {
    final ADRCController adrc = new ADRCController(1);
    adrc.initialize(0.2, 1.0, 10, -12, 12, kDt, null, 0.0, 1.0, 5.0, 0.0, "none", null, false);
    double last = 0;
    for (int i = 0; i < 20; i++) {
      final double u = adrc.calculate(0, 100);
      assertTrue(Math.abs(u - last) <= 5.0 * kDt + 1e-12, "changed by " + (u - last) + " in one step");
      last = u;
    }
    assertEquals(20 * 5.0 * kDt, last, 1e-9);
  }

  @Test
  void resetStartsTheObserverAtTheMeasurement() {
    final ADRCController adrc = controller(2, 0.5, 10, 12);
    run(adrc, new Plant(2, 10, -3, 0), 1.0, 2);
    adrc.reset(0.25);
    assertArrayEquals(new double[] {0.25, 0, 0}, adrc.getEstimatedStates(), 1e-12);
    assertEquals(0, adrc.getLastOutput());
  }

  @Test
  void retuningKeepsTheEstimatedDisturbance() {
    final ADRCController adrc = controller(1, 0.5, 10, 12);
    final Plant plant = new Plant(1, 10, -3, 0);
    run(adrc, plant, 1.0, 3);
    final double disturbance = adrc.getEstimatedDisturbance();
    adrc.updateTuning(0.25, 2.0);
    assertEquals(disturbance, adrc.getEstimatedDisturbance());
    assertEquals(0.25, adrc.getSettlingTime());
    assertEquals(2.0, adrc.getBandwidthScale());
    assertEquals(2.0 * 2 * 2.0 / 0.25, adrc.getBandwidth(), 1e-9);
    assertEquals(4 * adrc.getBandwidth(), adrc.getObserverBandwidth(), 1e-9);
    adrc.setB0(12);
    assertEquals(12, adrc.getB0());
    assertEquals(disturbance, adrc.getEstimatedDisturbance());
    assertTrue(run(adrc, plant, 1.0, 3) < 1e-3, "still holds its setpoint after retuning");
  }

  @Test
  void inputGainScaleDividesB0() {
    final ADRCController adrc = new ADRCController(1);
    adrc.initialize(0.2, 1.0, 10, -12, 12, kDt, null, 0.0, 2.0, 0.0, 0.0, "none", null, false);
    assertEquals(5, adrc.getB0());
  }

  @Test
  void inputDelayHoldsTheSetpointOnADelayedPlant() {
    final ADRCController adrc = new ADRCController(1);
    adrc.initialize(0.5, 1.0, 10, -12, 12, kDt, null, 0.0, 1.0, 0.0, 2 * kDt, "none", null, false);
    final Plant plant = new Plant(1, 10, -3, 0);
    final double[] pending = {0, 0};
    double worst = 0;
    for (int i = 0; i < 300; i++) {
      final double u = adrc.calculate(plant.output(), 1.0);
      plant.step(pending[1]);
      pending[1] = pending[0];
      pending[0] = u;
      if (i >= 250) {
        worst = Math.max(worst, Math.abs(plant.output() - 1.0));
      }
    }
    assertTrue(worst < 1e-3, "should hold 1.0 through a two step delay, but was " + worst + " from it");
  }

  @ParameterizedTest(name = "{0}")
  @ValueSource(strings = {"euler", "tod", "ld", "red", "intd"})
  void trackingDifferentiatorSmoothsTheSetpoint(String method) {
    final ADRCController adrc = new ADRCController(2);
    adrc.initialize(0.5, 1.0, 10, -12, 12, kDt, null, 0.0, 1.0, 0.0, 0.0, method, null, false);
    final Plant plant = new Plant(2, 10, -3, 0);
    final double worst = run(adrc, plant, 1.0, 15);
    assertTrue(worst < 1e-2, method + ": should hold 1.0, but was " + worst + " from it");
  }

  @Test
  void cascadedObserverHoldsTheSetpoint() {
    final ADRCController adrc = new ADRCController(1);
    adrc.initialize(0.5, 1.0, 10, -12, 12, kDt, null, 0.0, 1.0, 0.0, 0.0, "none", null, true);
    assertTrue(run(adrc, new Plant(1, 10, -3, 0), 1.0, 6) < 1e-3);
  }

  @Test
  void rejectsInvalidArguments() {
    assertThrows(IllegalArgumentException.class, () -> new ADRCController(0));
    assertThrows(IllegalArgumentException.class, () -> new ADRCController(5));
    final ADRCController adrc = new ADRCController(1);
    assertFalse(adrc.isInitialized());
    assertThrows(IllegalStateException.class, () -> adrc.calculate(0, 0));
    assertThrows(IllegalStateException.class, () -> adrc.reset(0));
    assertThrows(IllegalArgumentException.class,
        () -> adrc.initialize(0, 1, 10, -12, 12, kDt, null, 0, 1, 0, 0, "none", null, false));
    assertThrows(IllegalArgumentException.class,
        () -> adrc.initialize(0.2, 1, 0, -12, 12, kDt, null, 0, 1, 0, 0, "none", null, false));
    assertThrows(IllegalArgumentException.class,
        () -> adrc.initialize(0.2, 1, 10, 12, -12, kDt, null, 0, 1, 0, 0, "none", null, false));
    assertThrows(IllegalArgumentException.class,
        () -> adrc.initialize(0.2, 1, 10, -12, 12, kDt, new double[3], 0, 1, 0, 0, "none", null, false));
    assertThrows(IllegalArgumentException.class,
        () -> adrc.initialize(0.2, 1, 10, -12, 12, kDt, null, 0, 1, 0, 0, "unknown", null, false));
    adrc.initialize(0.2, 1, 10, -12, 12, kDt, null, 0, 1, 0, 0, "none", null, false);
    assertTrue(adrc.isInitialized());
    assertThrows(IllegalArgumentException.class, () -> adrc.setB0(0));
    assertThrows(IllegalArgumentException.class, () -> adrc.updateTuning(-1, 1));
    assertThrows(IllegalArgumentException.class, () -> adrc.reset(Double.NaN));
  }
}
