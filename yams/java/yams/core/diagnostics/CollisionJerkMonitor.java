// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package yams.core.diagnostics;

import static org.wpilib.units.Units.Seconds;

import java.util.Objects;
import org.wpilib.units.measure.Time;

/**
 * Gravity-invariant 3-axis IMU impact and jerk collision monitor.
 *
 * <p>Computes gravity-invariant impact magnitude {@code impactG = | ||a_g|| - 1.0 |} (which remains
 * {@code ~0.0 g} even while tilted on a bump or ramp because {@code ||g|| == 1 g} under any 3D
 * rotation) alongside 3-axis jerk {@code ||da_g / dt||} in {@code g/s}. Latches {@code
 * bumpedRecently} for a configurable hold window (default {@code 0.5 s}) after any impact so
 * downstream pose estimators or tactical controllers can widen tolerances or trigger recovery.
 *
 * <h2>Example</h2>
 *
 * <pre>{@code
 * CollisionJerkMonitor collisionMonitor = new CollisionJerkMonitor()
 *     .withImpactThresholdG(0.5)
 *     .withJerkThresholdGPerSec(25.0)
 *     .withBumpedHoldDuration(Seconds.of(0.5));
 *
 * // Inside robotPeriodic():
 * CollisionJerkMonitor.ImpactStatus status =
 *     collisionMonitor.updateG(Timer.getFPGATimestamp(), axG, ayG, azG);
 * if (status.bumpedRecently()) {
 *   // Widen tolerances or pause tight scoring until chassis settles
 * }
 * }</pre>
 */
public final class CollisionJerkMonitor {

  /** Standard Earth gravitational acceleration in m/s^2. */
  public static final double STANDARD_GRAVITY_MPS2 = 9.80665;

  /** Default gravity-invariant impact threshold in g. */
  public static final double DEFAULT_IMPACT_THRESHOLD_G = 0.5;

  /** Default 3-axis jerk threshold in g/s. */
  public static final double DEFAULT_JERK_THRESHOLD_G_PER_SEC = 25.0;

  /** Default bumped-recently latch duration. */
  public static final Time DEFAULT_BUMPED_HOLD_DURATION = Seconds.of(0.5);

  /** Resting acceleration magnitude in g under standard Earth gravity. */
  private static final double RESTING_GRAVITY_G = 1.0;

  /** Minimum sample interval in seconds required to compute finite-difference jerk safely. */
  private static final double MIN_JERK_DT_SECONDS = 1e-6;

  /** Maximum sample interval in seconds allowed for jerk differentiation before ignoring gaps. */
  private static final double MAX_JERK_DT_SECONDS = 0.5;

  /**
   * Immutable snapshot of the latest IMU impact and jerk evaluation.
   *
   * @param impactG Gravity-invariant impact magnitude {@code | ||a_g|| - 1.0 |} in g.
   * @param jerkGPerSec 3-axis jerk magnitude {@code ||da_g / dt||} in g/s.
   * @param impactDetected Whether the current sample exceeded the impact or jerk threshold.
   * @param bumpedRecently Whether an impact occurred within the configured hold window.
   * @param timeSinceLastImpactSec Elapsed seconds since the most recent impact, or {@link
   *     Double#POSITIVE_INFINITY} if none.
   */
  public record ImpactStatus(
      double impactG,
      double jerkGPerSec,
      boolean impactDetected,
      boolean bumpedRecently,
      double timeSinceLastImpactSec) {}

  /** Gravity-invariant impact threshold in g. */
  private double impactThresholdG = DEFAULT_IMPACT_THRESHOLD_G;

  /** 3-axis jerk threshold in g/s. */
  private double jerkThresholdGPerSec = DEFAULT_JERK_THRESHOLD_G_PER_SEC;

  /** Latch duration for {@code bumpedRecently} in seconds. */
  private double bumpedHoldDurationSec = DEFAULT_BUMPED_HOLD_DURATION.in(Seconds);

  /** Gravitational acceleration conversion constant in m/s^2. */
  private double gravityMps2 = STANDARD_GRAVITY_MPS2;

  /** Previous sample timestamp in seconds, or {@link Double#NaN} before the first sample. */
  private double lastTimestampSec = Double.NaN;

  /** Timestamp of the most recent impact in seconds, or {@link Double#NEGATIVE_INFINITY}. */
  private double lastImpactTimestampSec = Double.NEGATIVE_INFINITY;

  /** Previous X-axis acceleration in g. */
  private double prevAxG = 0.0;

  /** Previous Y-axis acceleration in g. */
  private double prevAyG = 0.0;

  /** Previous Z-axis acceleration in g. */
  private double prevAzG = RESTING_GRAVITY_G;

  /** Creates a collision monitor with competition defaults. */
  public CollisionJerkMonitor() {}

  /**
   * Sets the gravity-invariant impact magnitude threshold in g.
   *
   * @param thresholdG Positive impact threshold in g (default {@code 0.5}).
   * @return {@link CollisionJerkMonitor} for chaining.
   */
  public CollisionJerkMonitor withImpactThresholdG(double thresholdG) {
    if (!Double.isFinite(thresholdG) || thresholdG <= 0.0) {
      throw new IllegalArgumentException("impactThresholdG must be finite and positive");
    }
    this.impactThresholdG = thresholdG;
    return this;
  }

  /**
   * Sets the 3-axis jerk magnitude threshold in g/s.
   *
   * @param thresholdGPerSec Positive jerk threshold in g/s (default {@code 25.0}).
   * @return {@link CollisionJerkMonitor} for chaining.
   */
  public CollisionJerkMonitor withJerkThresholdGPerSec(double thresholdGPerSec) {
    if (!Double.isFinite(thresholdGPerSec) || thresholdGPerSec <= 0.0) {
      throw new IllegalArgumentException("jerkThresholdGPerSec must be finite and positive");
    }
    this.jerkThresholdGPerSec = thresholdGPerSec;
    return this;
  }

  /**
   * Sets how long {@code bumpedRecently} remains {@code true} after an impact.
   *
   * @param holdDuration Non-negative latch duration (default {@code 0.5 s}).
   * @return {@link CollisionJerkMonitor} for chaining.
   */
  public CollisionJerkMonitor withBumpedHoldDuration(Time holdDuration) {
    Objects.requireNonNull(holdDuration, "holdDuration must not be null");
    double sec = holdDuration.in(Seconds);
    if (!Double.isFinite(sec) || sec < 0.0) {
      throw new IllegalArgumentException("bumpedHoldDuration must be finite and non-negative");
    }
    this.bumpedHoldDurationSec = sec;
    return this;
  }

  /**
   * Sets the gravitational constant used by {@link #updateMetersPerSecondSquared}.
   *
   * @param gravityMps2 Positive gravitational acceleration in m/s^2 (default {@code 9.80665}).
   * @return {@link CollisionJerkMonitor} for chaining.
   */
  public CollisionJerkMonitor withGravityMps2(double gravityMps2) {
    if (!Double.isFinite(gravityMps2) || gravityMps2 <= 0.0) {
      throw new IllegalArgumentException("gravityMps2 must be finite and positive");
    }
    this.gravityMps2 = gravityMps2;
    return this;
  }

  /**
   * Updates the monitor with 3-axis IMU acceleration in m/s^2 (including gravity).
   *
   * @param timestampSeconds Monotonic timestamp in seconds.
   * @param axMps2 X-axis acceleration in m/s^2.
   * @param ayMps2 Y-axis acceleration in m/s^2.
   * @param azMps2 Z-axis acceleration in m/s^2.
   * @return Updated {@link ImpactStatus}.
   */
  public ImpactStatus updateMetersPerSecondSquared(
      double timestampSeconds, double axMps2, double ayMps2, double azMps2) {
    return updateG(
        timestampSeconds, axMps2 / gravityMps2, ayMps2 / gravityMps2, azMps2 / gravityMps2);
  }

  /**
   * Updates the monitor with 3-axis IMU acceleration in g (including 1 g gravity).
   *
   * @param timestampSeconds Monotonic timestamp in seconds.
   * @param axG X-axis acceleration in g.
   * @param ayG Y-axis acceleration in g.
   * @param azG Z-axis acceleration in g.
   * @return Updated {@link ImpactStatus}.
   */
  public ImpactStatus updateG(double timestampSeconds, double axG, double ayG, double azG) {
    if (!Double.isFinite(timestampSeconds)
        || !Double.isFinite(axG)
        || !Double.isFinite(ayG)
        || !Double.isFinite(azG)) {
      return new ImpactStatus(0.0, 0.0, false, false, Double.POSITIVE_INFINITY);
    }

    // Gravity-invariant norm: ||a_g|| == 1.0 at rest under any 3D pitch/roll/yaw tilt.
    double normG = Math.sqrt(axG * axG + ayG * ayG + azG * azG);
    double impactG = Math.abs(normG - RESTING_GRAVITY_G);

    double jerkGPerSec = 0.0;
    if (!Double.isNaN(lastTimestampSec)) {
      double dt = timestampSeconds - lastTimestampSec;
      if (dt > MIN_JERK_DT_SECONDS && dt <= MAX_JERK_DT_SECONDS) {
        double dax = axG - prevAxG;
        double day = ayG - prevAyG;
        double daz = azG - prevAzG;
        jerkGPerSec = Math.sqrt(dax * dax + day * day + daz * daz) / dt;
      }
    }

    lastTimestampSec = timestampSeconds;
    prevAxG = axG;
    prevAyG = ayG;
    prevAzG = azG;

    boolean impact = impactG >= impactThresholdG || jerkGPerSec >= jerkThresholdGPerSec;
    if (impact) {
      lastImpactTimestampSec = timestampSeconds;
    }

    double sinceLast =
        Double.isInfinite(lastImpactTimestampSec)
            ? Double.POSITIVE_INFINITY
            : Math.max(0.0, timestampSeconds - lastImpactTimestampSec);
    boolean bumpedRecently = sinceLast <= bumpedHoldDurationSec;
    return new ImpactStatus(impactG, jerkGPerSec, impact, bumpedRecently, sinceLast);
  }

  /** Resets previous IMU samples and impact latches. */
  public void reset() {
    lastTimestampSec = Double.NaN;
    lastImpactTimestampSec = Double.NEGATIVE_INFINITY;
    prevAxG = 0.0;
    prevAyG = 0.0;
    prevAzG = RESTING_GRAVITY_G;
  }
}
