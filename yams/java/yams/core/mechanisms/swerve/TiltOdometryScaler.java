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
 * Scales incremental swerve wheel-odometry distance deltas by the robot's 3D tilt angle so driving
 * over field bumps or ramps does not corrupt 2D pose estimation with inflated wheel travel.
 *
 * <p>Uses the exact yaw-invariant 3D tilt angle from vertical {@code tilt = acos(cos(pitch) *
 * cos(roll))}. Wheel deltas are scaled by {@code 1.0} up to {@code fullTrustTilt} (default {@code
 * 2 deg}), ramping linearly down to {@code 0.0} at {@code zeroTrustTilt} (default {@code 25 deg}).
 *
 * <h2>Example</h2>
 *
 * <pre>{@code
 * SwerveDriveConfig cfg =
 *     new SwerveDriveConfig()
 *         .withModules(fl, fr, bl, br)
 *         .withGyro(gyro::getRotation3d)
 *         .withTiltScaledOdometry(Degrees.of(2), Degrees.of(25));
 * }</pre>
 */
public class TiltOdometryScaler {
  /** Default tilt angle below which wheel odometry is trusted at 100% ({@code 1.0}). */
  public static final Angle DEFAULT_FULL_TRUST_TILT = Degrees.of(2.0);

  /** Default tilt angle at or above which wheel odometry deltas are attenuated to {@code 0.0}. */
  public static final Angle DEFAULT_ZERO_TRUST_TILT = Degrees.of(25.0);

  /** Tilt threshold in radians below which scale is {@code 1.0}. */
  private final double m_fullTrustTiltRad;

  /** Tilt threshold in radians at or above which scale is {@code 0.0}. */
  private final double m_zeroTrustTiltRad;

  /** Previous raw wheel distances in meters used to compute incremental deltas. */
  private double[] m_lastRawDistancesMeters;

  /** Accumulated tilt-scaled wheel distances in meters fed to the pose estimator. */
  private double[] m_scaledDistancesMeters;

  /** Last computed scale factor in {@code [0.0, 1.0]}. */
  private double m_lastScale = 1.0;

  /** Last computed 3D tilt angle in radians. */
  private double m_lastTiltRad = 0.0;

  /**
   * Create a {@link TiltOdometryScaler} with default thresholds ({@code 2 deg} to {@code 25 deg}).
   */
  public TiltOdometryScaler() {
    this(DEFAULT_FULL_TRUST_TILT, DEFAULT_ZERO_TRUST_TILT);
  }

  /**
   * Create a {@link TiltOdometryScaler}.
   *
   * @param fullTrustTilt Tilt angle at or below which wheel travel is trusted at {@code 1.0}
   *     ({@code >= 0 deg}).
   * @param zeroTrustTilt Tilt angle at or above which wheel travel is scaled to {@code 0.0}
   *     ({@code > fullTrustTilt} and {@code <= 90 deg}).
   */
  public TiltOdometryScaler(Angle fullTrustTilt, Angle zeroTrustTilt) {
    Objects.requireNonNull(fullTrustTilt, "fullTrustTilt");
    Objects.requireNonNull(zeroTrustTilt, "zeroTrustTilt");
    double fullRad = fullTrustTilt.in(Radians);
    double zeroRad = zeroTrustTilt.in(Radians);
    if (!Double.isFinite(fullRad)
        || !Double.isFinite(zeroRad)
        || fullRad < 0.0
        || zeroRad <= fullRad
        || zeroRad > Math.PI * 0.5) {
      throw new IllegalArgumentException(
          "Require 0 <= fullTrustTilt < zeroTrustTilt <= 90 deg, got full="
              + fullTrustTilt.in(Degrees)
              + " deg, zero="
              + zeroTrustTilt.in(Degrees)
              + " deg.");
    }
    m_fullTrustTiltRad = fullRad;
    m_zeroTrustTiltRad = zeroRad;
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
    cosTilt = Math.max(-1.0, Math.min(1.0, cosTilt));
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
   * Compute the odometry scale factor in {@code [0.0, 1.0]} for the given attitude without mutating
   * accumulated module state.
   *
   * @param attitude Robot {@link Rotation3d} attitude.
   * @return Scale factor in {@code [0.0, 1.0]}.
   */
  public double computeScale(Rotation3d attitude) {
    double tiltRad = computeTiltAngle(attitude).in(Radians);
    if (tiltRad <= m_fullTrustTiltRad) {
      return 1.0;
    }
    if (tiltRad >= m_zeroTrustTiltRad) {
      return 0.0;
    }
    return 1.0 - (tiltRad - m_fullTrustTiltRad) / (m_zeroTrustTiltRad - m_fullTrustTiltRad);
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
    m_lastScale = 1.0;
    m_lastTiltRad = 0.0;
  }

  /**
   * Apply tilt scaling to incremental wheel-distance deltas since the previous call and return the
   * effective {@link SwerveModulePosition}s to feed into {@code SwerveDrivePoseEstimator}.
   *
   * @param attitude Current robot {@link Rotation3d} attitude.
   * @param rawPositions Current raw {@link SwerveModulePosition}s from the modules.
   * @param rejectUpdate {@code true} to temporarily apply a {@code 0.0} scale factor (for example
   *     when optional skid odometry rejection is active).
   * @return Scaled {@link SwerveModulePosition}s.
   */
  public SwerveModulePosition[] scalePositions(
      Rotation3d attitude, SwerveModulePosition[] rawPositions, boolean rejectUpdate) {
    Objects.requireNonNull(attitude, "attitude");
    Objects.requireNonNull(rawPositions, "rawPositions");
    int n = rawPositions.length;
    if (m_lastRawDistancesMeters == null || m_lastRawDistancesMeters.length != n) {
      reset(rawPositions);
    }

    m_lastTiltRad = computeTiltAngle(attitude).in(Radians);
    double scale = rejectUpdate ? 0.0 : computeScale(attitude);
    m_lastScale = scale;

    SwerveModulePosition[] out = new SwerveModulePosition[n];
    for (int i = 0; i < n; i++) {
      double rawDist = rawPositions[i].distance;
      double delta = rawDist - m_lastRawDistancesMeters[i];
      m_lastRawDistancesMeters[i] = rawDist;
      m_scaledDistancesMeters[i] += delta * scale;
      out[i] = new SwerveModulePosition(m_scaledDistancesMeters[i], rawPositions[i].angle);
    }
    return out;
  }

  /**
   * Get the most recently applied scale factor in {@code [0.0, 1.0]}.
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
