// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package yams.core.math;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotSame;
import static org.junit.jupiter.api.Assertions.assertSame;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.wpilib.units.Units.Seconds;

import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.EnumSource;
import org.junit.jupiter.params.provider.ValueSource;
import yams.core.math.ADRCConfig.SystemOrder;

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

  /** A config for a plant of order 1 through 4. */
  private static ADRCConfig config(int order) {
    final SystemOrder[] orders = {SystemOrder.FIRST, SystemOrder.SECOND, SystemOrder.THIRD, SystemOrder.FOURTH};
    return new ADRCConfig().withSystemOrder(orders[order - 1]);
  }

  /** An initialized controller for a config, with {@code b0} 10 unless it sets one. */
  private static ADRCController initialized(ADRCConfig config) {
    final ADRCController adrc = new ADRCController(config);
    adrc.initialize(10, Seconds.of(kDt));
    return adrc;
  }

  private static ADRCController controller(int order, double settlingTime, double b0) {
    final ADRCController adrc = new ADRCController(config(order).withSettlingTime(Seconds.of(settlingTime)));
    adrc.initialize(b0, Seconds.of(kDt));
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
    final ADRCController adrc = controller(order, 1.0, 10);
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
      final ADRCController adrc = controller(order, 1.0, b0);
      final double worst = run(adrc, new Plant(order, 10, -3, 0), 1.0, 6);
      assertTrue(worst < 1e-3, "b0 " + b0 + ": should hold 1.0, but was " + worst + " from it");
    }
  }

  /**
   * A setpoint moving at a steady rate, as a motion profile's does while it cruises. Given where it
   * is now and where it will be next, the controller tracks it; told only the position, it trails
   * it, by the rate divided by the bandwidth for a first order loop and twice that for a second
   * order loop.
   */
  @ParameterizedTest(name = "order {0}")
  @ValueSource(ints = {1, 2})
  void tracksAMovingSetpointGivenItsRate(int order) {
    final double rate = 0.5;
    final double seconds = 4;
    final double[] lag = new double[2];
    for (int withRate = 0; withRate < 2; withRate++) {
      final ADRCController adrc = controller(order, 0.5, 10);
      final Plant plant = new Plant(order, 10, 0, 0);
      final int steps = (int) Math.round(seconds / kDt);
      for (int i = 0; i < steps; i++) {
        final double now = rate * i * kDt;
        final double next = rate * (i + 1) * kDt;
        plant.step(withRate == 1
            ? adrc.calculate(plant.output(), new ADRCController.Reference(now, rate),
                new ADRCController.Reference(next, rate), Double.NaN)
            : adrc.calculate(plant.output(), now, Double.NaN));
      }
      // The plant has run one period past the last setpoint: compare it with where the setpoint is now.
      lag[withRate] = rate * steps * kDt - plant.output();
    }
    final double bandwidth = 2.0 * (order + 1) / 0.5;
    assertEquals(order * rate / bandwidth, lag[0], 0.02,
        "told only the position, order " + order + " should trail by about " + order + " rate/bandwidth");
    assertTrue(Math.abs(lag[1]) < 0.02 * lag[0],
        "given the rate, order " + order + " should track it, but trailed by " + lag[1] + " (vs " + lag[0] + ")");
  }

  /** A still reference is the plain setpoint, and a non-finite rate is taken as zero. */
  @ParameterizedTest(name = "order {0}")
  @ValueSource(ints = {1, 2})
  void aStillReferenceIsThePlainSetpoint(int order) {
    final ADRCController plain = controller(order, 0.5, 10);
    final ADRCController still = controller(order, 0.5, 10);
    final ADRCController nonFinite = controller(order, 0.5, 10);
    for (int i = 0; i < 10; i++) {
      final double expected = plain.calculate(0.1 * i, 1.0, Double.NaN);
      assertEquals(expected, still.calculate(0.1 * i, ADRCController.Reference.still(1.0),
          ADRCController.Reference.still(1.0), Double.NaN), 1e-12);
      assertEquals(expected, nonFinite.calculate(0.1 * i, new ADRCController.Reference(1.0, Double.NaN),
          new ADRCController.Reference(1.0, Double.POSITIVE_INFINITY), Double.NaN), 1e-12);
    }
  }

  /**
   * The general observer, which places its poles by Ackermann's formula and is used when the
   * back-EMF is modeled, gives the same gains as the closed forms for a chain of integrators.
   */
  @ParameterizedTest(name = "order {0}")
  @ValueSource(ints = {1, 2, 3, 4})
  void placingTheObserverPolesMatchesTheClosedForms(int order) {
    final ADRCController.Gains chain = ADRCController.computeGains(order, 20, 10, kDt);
    final org.ejml.simple.SimpleMatrix c = new org.ejml.simple.SimpleMatrix(1, order + 1);
    c.set(0, 0, 1.0);
    // The chain's discrete state matrix, recovered from (I - L C) Ad with L known.
    final org.ejml.simple.SimpleMatrix ad = org.ejml.simple.SimpleMatrix.identity(order + 1)
        .minus(chain.ld.mult(c)).solve(chain.ad);
    final org.ejml.simple.SimpleMatrix placed =
        ADRCController.placeObserverPoles(ad, c, Math.exp(-4 * 20 * kDt));
    for (int i = 0; i <= order; i++) {
      assertEquals(chain.ld.get(i), placed.get(i), 1e-9 * Math.max(1, Math.abs(chain.ld.get(i))),
          "gain " + i);
    }
  }

  /**
   * With the back-EMF in the observer's model, the observer still has every pole of its error
   * dynamics, {@code (I - L C) Ad}, at the one its bandwidth gives.
   */
  @Test
  void theBackEmfObserverPlacesItsPoles() {
    final double tau = 0.16;
    final ADRCController.Gains gains = ADRCController.computeGains(2, 20, 10, kDt, 1 / tau);
    assertEquals(1 / tau, gains.backEmfRate, 1e-12);
    final var eigen = gains.ad.eig();
    for (int i = 0; i < eigen.getNumberOfEigenvalues(); i++) {
      assertEquals(Math.exp(-80 * kDt), eigen.getEigenvalue(i).getMagnitude(), 1e-3);
    }
  }

  /**
   * An actuator that applies less than it is asked for, as a motor controller does at its current
   * limit: told what was applied, the observer does not mistake the shortfall for a disturbance, and
   * the loop settles once the setpoint is within the actuator's reach.
   */
  @Test
  void settlesWhenTheActuatorAppliesLessThanItIsAskedFor() {
    final ADRCController adrc = controller(2, 0.5, 10);
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
  void resetStartsTheObserverAtTheMeasurement() {
    final ADRCController adrc = controller(2, 0.5, 10);
    run(adrc, new Plant(2, 10, -3, 0), 1.0, 2);
    adrc.reset(0.25);
    assertArrayEquals(new double[] {0.25, 0, 0}, adrc.getEstimatedStates(), 1e-12);
    assertEquals(0, adrc.getLastOutput());
  }

  @Test
  void changingTheConfigRetunesAtOnceAndKeepsTheEstimatedDisturbance() {
    final ADRCController adrc = controller(1, 0.5, 10);
    final Plant plant = new Plant(1, 10, -3, 0);
    run(adrc, plant, 1.0, 3);
    final double disturbance = adrc.getEstimatedDisturbance();
    final ADRCController.Gains before = adrc.getConfig().getGains();
    adrc.getConfig().withSettlingTime(Seconds.of(0.25)).withBandwidthScale(2.0);
    assertNotSame(before, adrc.getConfig().getGains(), "the config worked the gains out again");
    assertEquals(disturbance, adrc.getEstimatedDisturbance());
    assertEquals(0.25, adrc.getSettlingTime().in(Seconds));
    assertEquals(2.0, adrc.getBandwidthScale());
    assertEquals(2.0 * 2 * 2.0 / 0.25, adrc.getBandwidth(), 1e-9);
    assertEquals(4 * adrc.getBandwidth(), adrc.getObserverBandwidth(), 1e-9);
    adrc.getConfig().withB0(12);
    assertEquals(12, adrc.getB0());
    assertEquals(disturbance, adrc.getEstimatedDisturbance());
    assertTrue(run(adrc, plant, 1.0, 3) < 1e-3, "still holds its setpoint after retuning");
  }

  @Test
  void aCopyOfABuiltConfigHasItsOwnGains() {
    final ADRCController adrc = controller(2, 0.5, 10);
    final ADRCConfig copy = adrc.getConfig().clone();
    final ADRCController.Gains gains = adrc.getConfig().getGains();
    final ADRCController.Gains copied = copy.getGains();
    assertNotSame(gains, copied);
    assertNotSame(gains.ad, copied.ad, "the matrices are copied, not shared");
    assertTrue(gains.ld.isIdentical(copied.ld, 0) && gains.ad.isIdentical(copied.ad, 0)
        && gains.bd.isIdentical(copied.bd, 0) && gains.k.isIdentical(copied.k, 0), "with the same values");
    assertEquals(gains.b0, copied.b0);
    assertEquals(adrc.getConfig().getBuiltPeriod(), copy.getBuiltPeriod());
    assertThrows(IllegalStateException.class, () -> copy.withSystemOrder(SystemOrder.FIRST), "the copy keeps the built order");
    copy.withSettlingTime(Seconds.of(1));
    assertNotSame(copied, copy.getGains(), "the copy works out its own gains");
    assertSame(gains, adrc.getConfig().getGains(), "retuning the copy leaves the original's gains");
    assertEquals(0.5, adrc.getSettlingTime().in(Seconds));
  }

  @Test
  void initializeReadsTheConfigOverTheGivenB0() {
    final ADRCController fromConfig = new ADRCController(config(1)
        .withSettlingTime(Seconds.of(0.4))
        .withBandwidthScale(2)
        .withB0(5));
    fromConfig.initialize(10, Seconds.of(kDt));
    assertEquals(0.4, fromConfig.getSettlingTime().in(Seconds));
    assertEquals(2, fromConfig.getBandwidthScale());
    assertEquals(5, fromConfig.getB0(), "the config's b0, not the given one");

    final ADRCController fromDefaults = initialized(config(1));
    assertEquals(ADRCConfig.DEFAULT_SETTLING_TIME, fromDefaults.getSettlingTime());
    assertEquals(ADRCConfig.DEFAULT_BANDWIDTH_SCALE, fromDefaults.getBandwidthScale());
    assertEquals(10, fromDefaults.getB0(), "the given b0, as the config leaves it unset");
  }

  @Test
  void inputDelayHoldsTheSetpointOnADelayedPlant() {
    final ADRCController adrc = initialized(config(1).withSettlingTime(Seconds.of(0.5)).withInputDelay(Seconds.of(2 * kDt)));
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
  @EnumSource(TrackingDifferentiator.Method.class)
  void trackingDifferentiatorSmoothsTheSetpoint(TrackingDifferentiator.Method method) {
    final ADRCController adrc = initialized(config(2).withSettlingTime(Seconds.of(0.5)).withTrackingDifferentiator(method));
    final Plant plant = new Plant(2, 10, -3, 0);
    final double worst = run(adrc, plant, 1.0, 15);
    assertTrue(worst < 1e-2, method + ": should hold 1.0, but was " + worst + " from it");
  }

  @Test
  void cascadedObserverHoldsTheSetpoint() {
    final ADRCController adrc = initialized(config(1).withSettlingTime(Seconds.of(0.5)).withCascaded(true));
    assertTrue(run(adrc, new Plant(1, 10, -3, 0), 1.0, 6) < 1e-3);
  }

  @Test
  void rejectsInvalidArguments() {
    assertThrows(IllegalArgumentException.class, () -> new ADRCController(new ADRCConfig()), "no system order");
    final ADRCController adrc = new ADRCController(config(1));
    assertFalse(adrc.isInitialized());
    assertThrows(IllegalStateException.class, () -> adrc.calculate(0, 0));
    assertThrows(IllegalStateException.class, () -> adrc.reset(0));
    assertThrows(IllegalArgumentException.class, () -> adrc.initialize(10, Seconds.of(0)), "no period");
    assertThrows(IllegalArgumentException.class, () -> adrc.initialize(0, Seconds.of(kDt)), "a zero b0");
    adrc.initialize(10, Seconds.of(kDt));
    assertTrue(adrc.isInitialized());
    assertThrows(IllegalArgumentException.class, () -> adrc.reset(new double[3], 0), "a state of the wrong length");
    assertThrows(IllegalStateException.class, () -> adrc.getConfig().withSystemOrder(SystemOrder.SECOND),
        "a built config keeps its order");
    adrc.getConfig().withSystemOrder(SystemOrder.FIRST);
    assertThrows(IllegalArgumentException.class, () -> adrc.reset(Double.NaN));
  }
}
