// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package yams.core.mechanisms.swerve;

import static org.wpilib.units.Units.MetersPerSecond;

import java.util.Objects;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.kinematics.SwerveModuleVelocity;
import org.wpilib.units.measure.LinearVelocity;

/**
 * Rigid-body swerve wheel-skid detector based on module translational-velocity consistency.
 *
 * <p>For a rigid swerve chassis rotating at angular velocity {@code omega}, each module {@code i}
 * at robot-frame location {@code (x_i, y_i)} has a known rotational velocity component {@code
 * v_rot_i = (-omega * y_i, omega * x_i)}. Subtracting {@code v_rot_i} from the module's measured
 * velocity vector isolates the module's translational velocity {@code v_trans_i}. Without wheel
 * slip, every module measures the same translational speed, so {@code skidRatio =
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
   * Default minimum translational speed below which the skid ratio stays at {@code 1.0} so stopped
   * or pure-spinning modules do not amplify encoder quantization noise.
   */
  public static final LinearVelocity DEFAULT_MIN_TRANSLATIONAL_SPEED = MetersPerSecond.of(0.10);

  /**
   * Immutable snapshot of one skid evaluation.
   *
   * @param skidRatio Ratio {@code max(|v_trans_i|) / min(|v_trans_i|)} ({@code >= 1.0}; {@code 1.0}
   *     when stationary or without slip).
   * @param skidding {@code true} when {@code skidRatio} exceeds the configured threshold.
   * @param minTranslationalSpeedMps Smallest isolated module translational speed in m/s.
   * @param maxTranslationalSpeedMps Largest isolated module translational speed in m/s.
   */
  public record SkidState(
      double skidRatio,
      boolean skidding,
      double minTranslationalSpeedMps,
      double maxTranslationalSpeedMps) {

    /** Nominal non-skidding state returned before first update or while stationary. */
    public static final SkidState NOMINAL = new SkidState(1.0, false, 0.0, 0.0);
  }

  /** Skid ratio threshold above which wheel slip is flagged. */
  private final double m_skidRatioThreshold;

  /** Minimum translational speed in m/s required before computing the ratio. */
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
   * @param minTranslationalSpeed Minimum translational speed deadband ({@code >= 0}) below which
   *     {@code skidRatio} is reported as {@code 1.0}.
   */
  public SkidDetector(double skidRatioThreshold, LinearVelocity minTranslationalSpeed) {
    Objects.requireNonNull(minTranslationalSpeed, "minTranslationalSpeed");
    if (!Double.isFinite(skidRatioThreshold) || skidRatioThreshold <= 1.0) {
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
   * Evaluate wheel skid from measured module states, module robot-frame locations, and chassis
   * angular velocity.
   *
   * @param moduleStates Measured {@link SwerveModuleVelocity} for each module.
   * @param moduleLocations Robot-frame {@link Translation2d} location for each module (same order
   *     and length as {@code moduleStates}).
   * @param omegaRadPerSec Chassis angular velocity about +Z in rad/s (CCW positive).
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
    if (moduleStates.length < 2 || !Double.isFinite(omegaRadPerSec)) {
      return SkidState.NOMINAL;
    }

    double minTrans = Double.POSITIVE_INFINITY;
    double maxTrans = 0.0;

    for (int i = 0; i < moduleStates.length; i++) {
      SwerveModuleVelocity state = moduleStates[i];
      Translation2d loc = moduleLocations[i];
      double angleRad = state.angle.getRadians();
      double measVx = state.velocity * Math.cos(angleRad);
      double measVy = state.velocity * Math.sin(angleRad);

      // Subtract pure rigid-body rotational velocity: omega x r = (-omega * y, omega * x).
      double transVx = measVx - (-omegaRadPerSec * loc.getY());
      double transVy = measVy - (omegaRadPerSec * loc.getX());
      double transSpeed = Math.hypot(transVx, transVy);

      if (transSpeed < minTrans) {
        minTrans = transSpeed;
      }
      if (transSpeed > maxTrans) {
        maxTrans = transSpeed;
      }
    }

    // Guard against division by near-zero translational speeds (stationary or pure spin).
    if (maxTrans < m_minTranslationalSpeedMps) {
      return new SkidState(1.0, false, minTrans, maxTrans);
    }
    double denom = Math.max(minTrans, 1e-4);
    double ratio = maxTrans / denom;
    return new SkidState(ratio, ratio > m_skidRatioThreshold, minTrans, maxTrans);
  }
}
