// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package yams.core.mechanisms.swerve;

import static org.wpilib.units.Units.MetersPerSecond;

import java.util.Arrays;
import java.util.Objects;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.kinematics.SwerveModuleVelocity;
import org.wpilib.units.measure.LinearVelocity;

/**
 * Rigid-body swerve wheel-skid detector based on module translational-velocity consistency.
 *
 * <p>For a rigid swerve chassis rotating at gyro-measured angular velocity {@code omega}, each
 * module {@code i} at robot-frame location {@code (x_i, y_i)} has a known rotational velocity
 * component {@code v_rot_i = (-omega * y_i, omega * x_i)}. Subtracting {@code v_rot_i} from the
 * module's measured velocity vector isolates the module's translational velocity {@code v_trans_i}.
 * Without wheel slip, every module measures the same translational speed, so {@code skidRatio =
 * max(|v_trans_i|) / min(|v_trans_i|)} equals {@code 1.0}. When one or more wheels slip or drag,
 * {@code skidRatio} rises above {@code 1.0}.
 *
 * <h2>Example</h2>
 *
 * <pre>{@code
 * SwerveDriveConfig cfg =
 *     new SwerveDriveConfig()
 *         .withModules(fl, fr, bl, br)
 *         .withGyro(gyro::getRotation3d)
 *         .withSkidDetection(1.35);
 * }</pre>
 */
public class SkidDetector {
  /** Default skid ratio above which {@link SkidState#skidding()} becomes {@code true}. */
  public static final double DEFAULT_SKID_RATIO_THRESHOLD = 1.35;

  /**
   * Default minimum translational speed (and speed spread) deadband below which the skid ratio
   * stays at {@code 1.0} so stopped, pure-spinning, or low-speed noisy modules do not false-trigger
   * skid detection.
   */
  public static final LinearVelocity DEFAULT_MIN_TRANSLATIONAL_SPEED = MetersPerSecond.of(0.10);

  /** Ratio reported when stationary, below deadband, or without wheel slip. */
  private static final double NOMINAL_RATIO = 1.0;

  /** Minimum denominator in m/s to prevent division by zero when one wheel stalls. */
  private static final double MIN_DENOMINATOR_MPS = 1e-4;

  /** Bitmask indicating that no modules are flagged as slipping. */
  public static final int NO_SLIPPING_MODULES = 0;

  /** Minimum number of swerve modules required to compare translational speeds. */
  private static final int MIN_MODULES_FOR_SKID = 2;

  /**
   * Immutable snapshot of one skid evaluation.
   *
   * @param skidRatio Ratio {@code max(|v_trans_i|) / min(|v_trans_i|)} ({@code >= 1.0}; {@code 1.0}
   *     when stationary or without slip).
   * @param skidding {@code true} when {@code skidRatio} exceeds the configured threshold.
   * @param minTranslationalSpeedMps Smallest isolated module translational speed in m/s.
   * @param maxTranslationalSpeedMps Largest isolated module translational speed in m/s.
   * @param slippingModuleMask Bitmask where bit {@code i} ({@code 1 << i}) is set when module
   *     {@code i} is flagged as slipping. When a majority of modules agree, only the outlier
   *     module(s) are flagged so healthy wheels can continue advancing odometry; if a skid occurs
   *     with no clear majority (e.g. a 2-vs-2 split), all module bits are set.
   */
  public record SkidState(
      double skidRatio,
      boolean skidding,
      double minTranslationalSpeedMps,
      double maxTranslationalSpeedMps,
      int slippingModuleMask) {

    /** Nominal non-skidding state returned before first update or while stationary. */
    public static final SkidState NOMINAL =
        new SkidState(NOMINAL_RATIO, false, 0.0, 0.0, NO_SLIPPING_MODULES);

    /**
     * Check whether the given module index was flagged as slipping on this tick.
     *
     * @param moduleIndex Zero-based module index.
     * @return {@code true} if module {@code moduleIndex} is set in {@link #slippingModuleMask()}.
     */
    public boolean isModuleSlipping(int moduleIndex) {
      return moduleIndex >= 0
          && moduleIndex < Integer.SIZE
          && (slippingModuleMask & (1 << moduleIndex)) != 0;
    }
  }

  /** Skid ratio threshold above which wheel slip is flagged. */
  private final double m_skidRatioThreshold;

  /**
   * Minimum translational speed and speed-spread deadband in m/s required before computing the
   * ratio.
   */
  private final double m_minTranslationalSpeedMps;

  /**
   * Create a {@link SkidDetector} with the default minimum translational speed ({@code 0.10 m/s}).
   *
   * @param skidRatioThreshold Ratio threshold strictly greater than {@code 1.0} (e.g. {@code
   *     1.35}).
   */
  public SkidDetector(double skidRatioThreshold) {
    this(skidRatioThreshold, DEFAULT_MIN_TRANSLATIONAL_SPEED);
  }

  /**
   * Create a {@link SkidDetector}.
   *
   * @param skidRatioThreshold Ratio threshold strictly greater than {@code 1.0} (e.g. {@code
   *     1.35}).
   * @param minTranslationalSpeed Minimum translational speed and speed-spread deadband ({@code >=
   *     0}) below which {@code skidRatio} is reported as {@code 1.0}.
   */
  public SkidDetector(double skidRatioThreshold, LinearVelocity minTranslationalSpeed) {
    Objects.requireNonNull(minTranslationalSpeed, "minTranslationalSpeed");
    if (!Double.isFinite(skidRatioThreshold) || skidRatioThreshold <= NOMINAL_RATIO) {
      throw new IllegalArgumentException(
          "skidRatioThreshold must be finite and greater than 1.0, got: " + skidRatioThreshold);
    }
    double minSpeedMps = minTranslationalSpeed.in(MetersPerSecond);
    if (!Double.isFinite(minSpeedMps) || minSpeedMps < 0.0) {
      throw new IllegalArgumentException(
          "minTranslationalSpeed must be finite and non-negative, got: " + minSpeedMps);
    }
    m_skidRatioThreshold = skidRatioThreshold;
    m_minTranslationalSpeedMps = minSpeedMps;
  }

  /**
   * Get the configured skid ratio threshold.
   *
   * @return Skid ratio threshold ({@code > 1.0}).
   */
  public double getSkidRatioThreshold() {
    return m_skidRatioThreshold;
  }

  /**
   * Get the configured minimum translational speed deadband in meters per second.
   *
   * @return Minimum translational speed in m/s.
   */
  public double getMinTranslationalSpeedMps() {
    return m_minTranslationalSpeedMps;
  }

  /**
   * Evaluate wheel skid from measured module states, module robot-frame locations, and
   * gyro-measured chassis angular velocity.
   *
   * @param moduleStates Measured {@link SwerveModuleVelocity} for each module.
   * @param moduleLocations Robot-frame {@link Translation2d} location for each module (same order
   *     and length as {@code moduleStates}).
   * @param omegaRadPerSec Gyro-measured chassis angular velocity about +Z in rad/s (CCW positive).
   * @return {@link SkidState} for this tick.
   */
  public SkidState calculate(
      SwerveModuleVelocity[] moduleStates,
      Translation2d[] moduleLocations,
      double omegaRadPerSec) {
    Objects.requireNonNull(moduleStates, "moduleStates");
    Objects.requireNonNull(moduleLocations, "moduleLocations");
    if (moduleStates.length != moduleLocations.length) {
      throw new IllegalArgumentException(
          "moduleStates and moduleLocations must have the same length.");
    }
    int n = moduleStates.length;
    if (n < MIN_MODULES_FOR_SKID || !Double.isFinite(omegaRadPerSec)) {
      return SkidState.NOMINAL;
    }

    double[] transSpeeds = new double[n];
    double minTrans = Double.POSITIVE_INFINITY;
    double maxTrans = 0.0;

    for (int i = 0; i < n; i++) {
      SwerveModuleVelocity state = moduleStates[i];
      Translation2d loc = moduleLocations[i];
      double angleRad = state.angle.getRadians();
      double measVx = state.velocity * Math.cos(angleRad);
      double measVy = state.velocity * Math.sin(angleRad);

      // Subtract pure rigid-body rotational velocity: omega x r = (-omega * y, omega * x).
      double transVx = measVx - (-omegaRadPerSec * loc.getY());
      double transVy = measVy - (omegaRadPerSec * loc.getX());
      double transSpeed = Math.hypot(transVx, transVy);
      transSpeeds[i] = transSpeed;

      if (transSpeed < minTrans) {
        minTrans = transSpeed;
      }
      if (transSpeed > maxTrans) {
        maxTrans = transSpeed;
      }
    }

    // Guard against division by near-zero translational speeds and low-speed encoder noise:
    // require both the fastest module and the speed spread (max - min) to exceed the deadband.
    if (maxTrans < m_minTranslationalSpeedMps
        || (maxTrans - minTrans) < m_minTranslationalSpeedMps) {
      return new SkidState(NOMINAL_RATIO, false, minTrans, maxTrans, NO_SLIPPING_MODULES);
    }

    double denom = Math.max(minTrans, MIN_DENOMINATOR_MPS);
    double ratio = maxTrans / denom;
    boolean skidding = ratio > m_skidRatioThreshold;
    int slippingMask = skidding ? computeSlippingMask(transSpeeds) : NO_SLIPPING_MODULES;
    return new SkidState(ratio, skidding, minTrans, maxTrans, slippingMask);
  }

  /**
   * Identify which module(s) deviate from the chassis median translational speed by more than the
   * configured threshold. Falls back to flagging all modules when there is no majority consensus
   * (such as a 2-vs-2 split).
   */
  private int computeSlippingMask(double[] transSpeeds) {
    int n = transSpeeds.length;
    double[] sorted = transSpeeds.clone();
    Arrays.sort(sorted);
    double median =
        (n % 2 == 1)
            ? sorted[n / 2]
            : 0.5 * (sorted[(n / 2) - 1] + sorted[n / 2]);

    int mask = NO_SLIPPING_MODULES;
    int bitLimit = Math.min(n, Integer.SIZE);
    for (int i = 0; i < bitLimit; i++) {
      double speed = transSpeeds[i];
      double hi = Math.max(speed, median);
      double lo = Math.max(Math.min(speed, median), MIN_DENOMINATOR_MPS);
      if ((hi / lo) > m_skidRatioThreshold
          && Math.abs(speed - median) >= m_minTranslationalSpeedMps) {
        mask |= (1 << i);
      }
    }
    if (mask == NO_SLIPPING_MODULES) {
      for (int i = 0; i < bitLimit; i++) {
        mask |= (1 << i);
      }
    }
    return mask;
  }
}
