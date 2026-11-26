// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package yams.core.mechanisms.swerve;

import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Radians;

import java.util.Objects;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.kinematics.SwerveModulePosition;
import org.wpilib.units.measure.Angle;

/**
 * Scales incremental swerve wheel-odometry distance deltas by the robot's 3D tilt angle (and
 * optionally drops individual slipping modules) so driving over field bumps, ramps, or low-traction
 * patches does not corrupt 2D pose estimation with inflated wheel travel.
 *
 * <p>Uses the exact yaw-invariant 3D tilt angle from vertical {@code tilt = acos(cos(pitch) *
 * cos(roll))}. Two attenuation modes are available via {@link ScalingMode}:
 *
 * <ul>
 *   <li>{@link ScalingMode#LINEAR_RAMP} (default): a trust heuristic that scales wheel deltas by
 *       {@code 1.0} up to {@code fullTrustTilt} (default {@code 2 deg}) and ramps linearly down to
 *       {@code 0.0} at {@code zeroTrustTilt} (default {@code 25 deg}) to aggressively suppress
 *       unreliable wheel contact over steep bumps.
 *   <li>{@link ScalingMode#COSINE_PROJECTION}: geometric horizontal projection {@code cos(tilt)}
 *       above {@code fullTrustTilt}, dropping to {@code 0.0} at or above {@code zeroTrustTilt},
 *       matching horizontal ground travel on a rigid slope without wheel slip.
 * </ul>
 *
 * <h2>Example</h2>
 *
 * <pre>{@code
 * SwerveDriveConfig cfg =
 *     new SwerveDriveConfig()
 *         .withModules(fl, fr, bl, br)
 *         .withGyro(gyro::getRotation3d)
 *         .withTiltScaledOdometry(
 *             Degrees.of(2), Degrees.of(25), TiltOdometryScaler.ScalingMode.COSINE_PROJECTION);
 * }</pre>
 */
public class TiltOdometryScaler {
  /** Default tilt angle below which wheel odometry is trusted at 100% ({@code 1.0}). */
  public static final Angle DEFAULT_FULL_TRUST_TILT = Degrees.of(2.0);

  /** Default tilt angle at or above which wheel odometry deltas are attenuated to {@code 0.0}. */
  public static final Angle DEFAULT_ZERO_TRUST_TILT = Degrees.of(25.0);

  /** Full odometry trust scale factor ({@code 1.0}). */
  private static final double FULL_SCALE = 1.0;

  /** Zero odometry trust scale factor ({@code 0.0}). */
  private static final double ZERO_SCALE = 0.0;

  /** Maximum valid tilt threshold in radians (90 degrees). */
  private static final double MAX_TILT_RAD = Math.PI * 0.5;

  /**
   * How wheel-odometry distance deltas are attenuated as the robot tilts away from horizontal.
   */
  public enum ScalingMode {
    /**
     * Heuristic linear trust ramp from {@code 1.0} at {@code fullTrustTilt} down to {@code 0.0} at
     * {@code zeroTrustTilt}. This is a trust heuristic rather than a geometric projection: it
     * aggressively suppresses noisy wheel deltas on steep bumps or ramps where wheel contact is
     * unreliable.
     */
    LINEAR_RAMP,
    /**
     * Geometric horizontal projection {@code cos(tilt)} above {@code fullTrustTilt}, dropping to
     * {@code 0.0} at or above {@code zeroTrustTilt}. For a wheel rolling along a rigid slope
     * without slip, horizontal ground travel equals {@code wheelDelta * cos(tilt)}.
     */
    COSINE_PROJECTION
  }

  /**
   * Immutable configuration for constructing a stateful {@link TiltOdometryScaler} per {@link
   * SwerveDrive} instance without sharing mutable wheel-distance history through {@link
   * yams.core.mechanisms.config.SwerveDriveConfig}.
   *
   * @param fullTrustTilt Tilt angle at or below which wheel travel is trusted at {@code 1.0}
   *     ({@code >= 0 deg}).
   * @param zeroTrustTilt Tilt angle at or above which wheel travel is scaled to {@code 0.0} ({@code
   *     > fullTrustTilt} and {@code <= 90 deg}).
   * @param scalingMode Attenuation curve applied between {@code fullTrustTilt} and {@code
   *     zeroTrustTilt}.
   */
  public record Config(Angle fullTrustTilt, Angle zeroTrustTilt, ScalingMode scalingMode) {
    /** Validate thresholds and scaling mode on construction. */
    public Config {
      Objects.requireNonNull(fullTrustTilt, "fullTrustTilt");
      Objects.requireNonNull(zeroTrustTilt, "zeroTrustTilt");
      Objects.requireNonNull(scalingMode, "scalingMode");
      double fullRad = fullTrustTilt.in(Radians);
      double zeroRad = zeroTrustTilt.in(Radians);
      if (!Double.isFinite(fullRad)
          || !Double.isFinite(zeroRad)
          || fullRad < 0.0
          || zeroRad <= fullRad
          || zeroRad > MAX_TILT_RAD) {
        throw new IllegalArgumentException(
            "Require 0 <= fullTrustTilt < zeroTrustTilt <= 90 deg, got full="
                + fullTrustTilt.in(Degrees)
                + " deg, zero="
                + zeroTrustTilt.in(Degrees)
                + " deg.");
      }
    }
  }

  /** Tilt threshold in radians below which scale is {@code 1.0}. */
  private final double m_fullTrustTiltRad;

  /** Tilt threshold in radians at or above which scale is {@code 0.0}. */
  private final double m_zeroTrustTiltRad;

  /** Attenuation curve applied between full-trust and zero-trust tilt. */
  private final ScalingMode m_scalingMode;

  /** Whether tilt-based attenuation is enabled (false for skid-rejection-only passthrough). */
  private final boolean m_tiltScalingEnabled;

  /** Previous raw wheel distances in meters used to compute incremental deltas. */
  private double[] m_lastRawDistancesMeters;

  /** Accumulated tilt-scaled wheel distances in meters fed to the pose estimator. */
  private double[] m_scaledDistancesMeters;

  /** Last computed effective scale factor in {@code [0.0, 1.0]}. */
  private double m_lastScale = FULL_SCALE;

  /** Last computed 3D tilt angle in radians. */
  private double m_lastTiltRad = 0.0;

  /**
   * Create a {@link TiltOdometryScaler} with default thresholds ({@code 2 deg} to {@code 25 deg})
   * and {@link ScalingMode#LINEAR_RAMP}.
   */
  public TiltOdometryScaler() {
    this(DEFAULT_FULL_TRUST_TILT, DEFAULT_ZERO_TRUST_TILT, ScalingMode.LINEAR_RAMP);
  }

  /**
   * Create a {@link TiltOdometryScaler} using {@link ScalingMode#LINEAR_RAMP}.
   *
   * @param fullTrustTilt Tilt angle at or below which wheel travel is trusted at {@code 1.0}
   *     ({@code >= 0 deg}).
   * @param zeroTrustTilt Tilt angle at or above which wheel travel is scaled to {@code 0.0}
   *     ({@code > fullTrustTilt} and {@code <= 90 deg}).
   */
  public TiltOdometryScaler(Angle fullTrustTilt, Angle zeroTrustTilt) {
    this(fullTrustTilt, zeroTrustTilt, ScalingMode.LINEAR_RAMP);
  }

  /**
   * Create a {@link TiltOdometryScaler} with an explicit {@link ScalingMode}.
   *
   * @param fullTrustTilt Tilt angle at or below which wheel travel is trusted at {@code 1.0}
   *     ({@code >= 0 deg}).
   * @param zeroTrustTilt Tilt angle at or above which wheel travel is scaled to {@code 0.0}
   *     ({@code > fullTrustTilt} and {@code <= 90 deg}).
   * @param scalingMode Attenuation curve applied between {@code fullTrustTilt} and {@code
   *     zeroTrustTilt}.
   */
  public TiltOdometryScaler(Angle fullTrustTilt, Angle zeroTrustTilt, ScalingMode scalingMode) {
    this(new Config(fullTrustTilt, zeroTrustTilt, scalingMode));
  }

  /**
   * Create a {@link TiltOdometryScaler} from an immutable {@link Config}.
   *
   * @param config Immutable tilt-scaling configuration.
   */
  public TiltOdometryScaler(Config config) {
    Objects.requireNonNull(config, "config");
    m_fullTrustTiltRad = config.fullTrustTilt().in(Radians);
    m_zeroTrustTiltRad = config.zeroTrustTilt().in(Radians);
    m_scalingMode = config.scalingMode();
    m_tiltScalingEnabled = true;
  }

  private TiltOdometryScaler(boolean tiltScalingEnabled) {
    m_fullTrustTiltRad = 0.0;
    m_zeroTrustTiltRad = MAX_TILT_RAD;
    m_scalingMode = ScalingMode.LINEAR_RAMP;
    m_tiltScalingEnabled = tiltScalingEnabled;
  }

  /**
   * Create a passthrough {@link TiltOdometryScaler} that leaves wheel deltas unscaled by tilt
   * ({@code 1.0}) while still tracking raw wheel-distance deltas for per-module or full-chassis
   * skid suppression (such as skid odometry rejection without tilt attenuation).
   *
   * @return Passthrough {@link TiltOdometryScaler}.
   */
  public static TiltOdometryScaler passthrough() {
    return new TiltOdometryScaler(false);
  }

  /**
   * Compute the exact yaw-invariant 3D tilt angle from vertical for the given robot attitude.
   *
   * @param attitude Robot {@link Rotation3d} attitude (roll, pitch, yaw).
   * @return Non-negative tilt angle from vertical in {@code [0, pi]}.
   */
  public static Angle computeTiltAngle(Rotation3d attitude) {
    Objects.requireNonNull(attitude, "attitude");
    // Using cos(pitch) * cos(roll) gives the exact angle between robot +Z and world +Z,
    // independent of yaw.
    double cosTilt = Math.cos(attitude.getY()) * Math.cos(attitude.getX());
    cosTilt = Math.max(-FULL_SCALE, Math.min(FULL_SCALE, cosTilt));
    return Radians.of(Math.acos(cosTilt));
  }

  /**
   * Check whether the robot's 3D tilt angle exceeds {@code maxTilt}.
   *
   * @param attitude Robot {@link Rotation3d} attitude.
   * @param maxTilt Maximum permitted tilt angle.
   * @return {@code true} if the robot's 3D tilt strictly exceeds {@code maxTilt}.
   */
  public static boolean isTiltExceeding(Rotation3d attitude, Angle maxTilt) {
    Objects.requireNonNull(maxTilt, "maxTilt");
    return computeTiltAngle(attitude).in(Radians) > maxTilt.in(Radians);
  }

  /**
   * Get the configured {@link ScalingMode}.
   *
   * @return Active {@link ScalingMode}.
   */
  public ScalingMode getScalingMode() {
    return m_scalingMode;
  }

  /**
   * Compute the tilt odometry scale factor in {@code [0.0, 1.0]} for the given attitude without
   * mutating accumulated module state.
   *
   * @param attitude Robot {@link Rotation3d} attitude.
   * @return Scale factor in {@code [0.0, 1.0]}.
   */
  public double computeScale(Rotation3d attitude) {
    if (!m_tiltScalingEnabled) {
      return FULL_SCALE;
    }
    double tiltRad = computeTiltAngle(attitude).in(Radians);
    if (tiltRad <= m_fullTrustTiltRad) {
      return FULL_SCALE;
    }
    if (tiltRad >= m_zeroTrustTiltRad) {
      return ZERO_SCALE;
    }
    return switch (m_scalingMode) {
      case LINEAR_RAMP ->
          FULL_SCALE - (tiltRad - m_fullTrustTiltRad) / (m_zeroTrustTiltRad - m_fullTrustTiltRad);
      case COSINE_PROJECTION -> Math.max(ZERO_SCALE, Math.cos(tiltRad));
    };
  }

  /**
   * Reset internal wheel-distance history to match {@code currentRawPositions}. Call whenever
   * odometry is reset so subsequent ticks start with zero delta.
   *
   * @param currentRawPositions Current raw {@link SwerveModulePosition}s.
   */
  public void reset(SwerveModulePosition[] currentRawPositions) {
    Objects.requireNonNull(currentRawPositions, "currentRawPositions");
    int n = currentRawPositions.length;
    m_lastRawDistancesMeters = new double[n];
    m_scaledDistancesMeters = new double[n];
    for (int i = 0; i < n; i++) {
      double d = currentRawPositions[i].distance;
      m_lastRawDistancesMeters[i] = d;
      m_scaledDistancesMeters[i] = d;
    }
    m_lastScale = FULL_SCALE;
    m_lastTiltRad = 0.0;
  }

  /**
   * Apply tilt scaling to incremental wheel-distance deltas since the previous call and return the
   * effective {@link SwerveModulePosition}s to feed into {@code SwerveDrivePoseEstimator}.
   *
   * @param attitude Current robot {@link Rotation3d} attitude.
   * @param rawPositions Current raw {@link SwerveModulePosition}s from the modules.
   * @param rejectUpdate {@code true} to temporarily freeze all module deltas ({@code 0.0} scale).
   * @return Scaled {@link SwerveModulePosition}s.
   */
  public SwerveModulePosition[] scalePositions(
      Rotation3d attitude, SwerveModulePosition[] rawPositions, boolean rejectUpdate) {
    Objects.requireNonNull(rawPositions, "rawPositions");
    int mask = SkidDetector.NO_SLIPPING_MODULES;
    if (rejectUpdate) {
      int bitLimit = Math.min(rawPositions.length, Integer.SIZE);
      for (int i = 0; i < bitLimit; i++) {
        mask |= (1 << i);
      }
    }
    return scalePositions(attitude, rawPositions, mask);
  }

  /**
   * Apply tilt scaling to incremental wheel-distance deltas since the previous call while freezing
   * only the modules flagged in {@code slippingModuleMask}, so healthy wheels continue advancing
   * odometry during a single-wheel skid.
   *
   * @param attitude Current robot {@link Rotation3d} attitude.
   * @param rawPositions Current raw {@link SwerveModulePosition}s from the modules.
   * @param slippingModuleMask Bitmask where bit {@code i} ({@code 1 << i}) freezes module {@code
   *     i}'s delta on this tick (see {@link SkidDetector.SkidState#slippingModuleMask()}).
   * @return Scaled {@link SwerveModulePosition}s.
   */
  public SwerveModulePosition[] scalePositions(
      Rotation3d attitude, SwerveModulePosition[] rawPositions, int slippingModuleMask) {
    Objects.requireNonNull(attitude, "attitude");
    Objects.requireNonNull(rawPositions, "rawPositions");
    int n = rawPositions.length;
    if (m_lastRawDistancesMeters == null || m_lastRawDistancesMeters.length != n) {
      reset(rawPositions);
    }

    m_lastTiltRad = computeTiltAngle(attitude).in(Radians);
    double tiltScale = computeScale(attitude);

    SwerveModulePosition[] out = new SwerveModulePosition[n];
    double scaleSum = 0.0;
    for (int i = 0; i < n; i++) {
      boolean moduleSlipping =
          i < Integer.SIZE && (slippingModuleMask & (1 << i)) != 0;
      double moduleScale = moduleSlipping ? ZERO_SCALE : tiltScale;
      scaleSum += moduleScale;

      double rawDist = rawPositions[i].distance;
      double delta = rawDist - m_lastRawDistancesMeters[i];
      m_lastRawDistancesMeters[i] = rawDist;
      m_scaledDistancesMeters[i] += delta * moduleScale;
      out[i] = new SwerveModulePosition(m_scaledDistancesMeters[i], rawPositions[i].angle);
    }
    m_lastScale = (n > 0) ? (scaleSum / n) : tiltScale;
    return out;
  }

  /**
   * Get the most recently applied average effective scale factor across all modules in {@code [0.0,
   * 1.0]}. Equals the tilt scale when no modules are slipping, {@code 0.0} when all modules are
   * suppressed, and the mean per-module scale when only a subset of modules are slipping.
   *
   * @return Last applied odometry scale factor.
   */
  public double getLastScale() {
    return m_lastScale;
  }

  /**
   * Get the most recently computed 3D tilt angle.
   *
   * @return Last computed 3D tilt {@link Angle}.
   */
  public Angle getLastTiltAngle() {
    return Radians.of(m_lastTiltRad);
  }
}
