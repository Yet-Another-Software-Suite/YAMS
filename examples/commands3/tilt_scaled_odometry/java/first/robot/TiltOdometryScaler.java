// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package first.robot;

import static org.wpilib.units.Units.Radians;

import java.util.Objects;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.kinematics.SwerveModulePosition;
import org.wpilib.units.measure.Angle;

/**
 * Projects swerve wheel-distance deltas onto the flat field plane using 3D gyro attitude and
 * provides a 3D tilt gate for 2D vision pose updates.
 */
public final class TiltOdometryScaler {

  /** Mode selecting how 3D chassis tilt attenuates incremental wheel-distance deltas. */
  public enum ScalingMode {
    /** Exact horizontal ramp projection: {@code delta_flat = delta_wheel * cos(tilt)}. */
    COSINE_PROJECTION,
    /** Conservative linear trust heuristic from 1.0 at {@code fullTrustTilt} down to 0.0 at {@code zeroTrustTilt}. */
    LINEAR_TRUST_RAMP
  }

  private final ScalingMode mode;
  private final double fullTrustTiltRad;
  private final double zeroTrustTiltRad;
  private final double maxVisionTiltRad;

  private double[] lastRawDistancesMeters;
  private double[] scaledDistancesMeters;
  private double lastScale = 1.0;

  public TiltOdometryScaler(
      ScalingMode mode, Angle fullTrustTilt, Angle zeroTrustTilt, Angle maxVisionTilt) {
    this.mode = Objects.requireNonNull(mode, "mode");
    Objects.requireNonNull(fullTrustTilt, "fullTrustTilt");
    Objects.requireNonNull(zeroTrustTilt, "zeroTrustTilt");
    Objects.requireNonNull(maxVisionTilt, "maxVisionTilt");
    double fullRad = fullTrustTilt.in(Radians);
    double zeroRad = zeroTrustTilt.in(Radians);
    double visionRad = maxVisionTilt.in(Radians);
    if (!Double.isFinite(fullRad) || fullRad < 0.0) {
      throw new IllegalArgumentException("fullTrustTilt must be finite and >= 0.");
    }
    if (!Double.isFinite(zeroRad) || zeroRad <= fullRad) {
      throw new IllegalArgumentException("zeroTrustTilt must be finite and > fullTrustTilt.");
    }
    if (!Double.isFinite(visionRad) || visionRad <= 0.0) {
      throw new IllegalArgumentException("maxVisionTilt must be finite and > 0.");
    }
    this.fullTrustTiltRad = fullRad;
    this.zeroTrustTiltRad = zeroRad;
    this.maxVisionTiltRad = visionRad;
  }

  /** Computes the exact 3D tilt angle away from vertical: {@code acos(cos(roll) * cos(pitch))}. */
  public static Angle computeTiltAngle(Rotation3d attitude) {
    Objects.requireNonNull(attitude, "attitude");
    double cosTilt = Math.cos(attitude.getX()) * Math.cos(attitude.getY());
    double clamped = Math.max(-1.0, Math.min(1.0, cosTilt));
    return Radians.of(Math.acos(clamped));
  }

  /** Computes the wheel-delta scale factor in {@code [0.0, 1.0]} for {@code attitude}. */
  public double computeScale(Rotation3d attitude) {
    double tiltRad = computeTiltAngle(attitude).in(Radians);
    return switch (mode) {
      case COSINE_PROJECTION -> Math.max(0.0, Math.cos(tiltRad));
      case LINEAR_TRUST_RAMP -> {
        if (tiltRad <= fullTrustTiltRad) {
          yield 1.0;
        }
        if (tiltRad >= zeroTrustTiltRad) {
          yield 0.0;
        }
        yield 1.0 - (tiltRad - fullTrustTiltRad) / (zeroTrustTiltRad - fullTrustTiltRad);
      }
    };
  }

  /** Returns true if the robot's current 3D tilt is within {@code maxVisionTilt}. */
  public boolean shouldAcceptVision(Rotation3d attitude) {
    return computeTiltAngle(attitude).in(Radians) <= maxVisionTiltRad + 1e-9;
  }

  /** Resets the internal raw and scaled wheel baselines (call whenever odometry is reset). */
  public void reset(SwerveModulePosition[] rawPositions) {
    Objects.requireNonNull(rawPositions, "rawPositions");
    int n = rawPositions.length;
    lastRawDistancesMeters = new double[n];
    scaledDistancesMeters = new double[n];
    for (int i = 0; i < n; i++) {
      lastRawDistancesMeters[i] = rawPositions[i].distance;
      scaledDistancesMeters[i] = rawPositions[i].distance;
    }
    lastScale = 1.0;
  }

  /** Scales each module's incremental wheel delta by {@link #computeScale(Rotation3d)}. */
  public SwerveModulePosition[] scalePositions(
      Rotation3d attitude, SwerveModulePosition[] rawPositions) {
    Objects.requireNonNull(attitude, "attitude");
    Objects.requireNonNull(rawPositions, "rawPositions");
    int n = rawPositions.length;
    if (lastRawDistancesMeters == null || lastRawDistancesMeters.length != n) {
      reset(rawPositions);
    }
    double scale = computeScale(attitude);
    lastScale = scale;
    SwerveModulePosition[] out = new SwerveModulePosition[n];
    for (int i = 0; i < n; i++) {
      double delta = rawPositions[i].distance - lastRawDistancesMeters[i];
      lastRawDistancesMeters[i] = rawPositions[i].distance;
      scaledDistancesMeters[i] += delta * scale;
      out[i] = new SwerveModulePosition(scaledDistancesMeters[i], rawPositions[i].angle);
    }
    return out;
  }

  /** Returns the most recently computed scale factor in {@code [0.0, 1.0]}. */
  public double getLastScale() {
    return lastScale;
  }
}
