// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package first.robot;

import static org.wpilib.units.Units.MetersPerSecond;
import static org.wpilib.units.Units.RadiansPerSecond;

import java.util.Arrays;
import java.util.Objects;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.kinematics.SwerveModuleVelocity;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.LinearVelocity;

/**
 * Gyro-referenced swerve wheel-skid detector (Orbit 1690 method).
 *
 * <p>Subtracts pure rigid-body rotation ({@code omega x r_i}) from each module's measured velocity
 * vector using the gyro's true yaw rate rather than wheel-fitted kinematics, applies a
 * slowest-module deadband to reject low-speed encoder noise, and flags only the slipping module(s)
 * so gripping wheels can keep advancing odometry.
 */
public final class SkidDetector {
  private static final double MIN_DIVISOR_MPS = 1e-6;
  private static final double NO_SKID_RATIO = 1.0;

  /** Immutable detection result for a single robot loop. */
  public record Result(
      double ratio, boolean isSkidding, boolean[] slippingModules, int slippingModuleCount) {

    public Result {
      slippingModules = slippingModules.clone();
    }

    /** Returns a defensive copy of the per-module slip mask. */
    @Override
    public boolean[] slippingModules() {
      return slippingModules.clone();
    }

    /** Returns true if module {@code index} is flagged as slipping. */
    public boolean isModuleSlipping(int index) {
      return index >= 0 && index < slippingModules.length && slippingModules[index];
    }

    /** Convenience factory for a clean, non-skidding frame. */
    public static Result noSkid(int moduleCount) {
      return new Result(NO_SKID_RATIO, false, new boolean[moduleCount], 0);
    }
  }

  private final Translation2d[] moduleLocations;
  private final double skidRatioThreshold;
  private final double minTranslationalSpeedMps;

  public SkidDetector(
      Translation2d[] moduleLocations,
      double skidRatioThreshold,
      LinearVelocity minTranslationalSpeed) {
    Objects.requireNonNull(moduleLocations, "moduleLocations");
    Objects.requireNonNull(minTranslationalSpeed, "minTranslationalSpeed");
    if (moduleLocations.length < 2) {
      throw new IllegalArgumentException("At least 2 swerve modules are required.");
    }
    if (!Double.isFinite(skidRatioThreshold) || skidRatioThreshold <= 1.0) {
      throw new IllegalArgumentException("skidRatioThreshold must be finite and > 1.0.");
    }
    double minMps = minTranslationalSpeed.in(MetersPerSecond);
    if (!Double.isFinite(minMps) || minMps < 0.0) {
      throw new IllegalArgumentException("minTranslationalSpeed must be finite and >= 0.");
    }
    this.moduleLocations = moduleLocations.clone();
    this.skidRatioThreshold = skidRatioThreshold;
    this.minTranslationalSpeedMps = minMps;
  }

  /**
   * Evaluates whether any swerve module is slipping relative to the chassis rigid-body motion.
   *
   * @param moduleStates Measured module velocities and angles.
   * @param gyroYawRate True chassis yaw rate from the gyro (CCW positive).
   * @return {@link Result} containing the ratio, overall flag, and per-module slip mask.
   */
  public Result detect(SwerveModuleVelocity[] moduleStates, AngularVelocity gyroYawRate) {
    Objects.requireNonNull(moduleStates, "moduleStates");
    Objects.requireNonNull(gyroYawRate, "gyroYawRate");
    if (moduleStates.length != moduleLocations.length) {
      throw new IllegalArgumentException(
          "Expected " + moduleLocations.length + " module states, got " + moduleStates.length);
    }

    double omegaRadPerSec = gyroYawRate.in(RadiansPerSecond);
    if (!Double.isFinite(omegaRadPerSec)) {
      return Result.noSkid(moduleLocations.length);
    }

    double[] transSpeeds = new double[moduleLocations.length];
    double minSpeed = Double.POSITIVE_INFINITY;
    double maxSpeed = 0.0;

    for (int i = 0; i < moduleLocations.length; i++) {
      double speedMps = moduleStates[i].velocity;
      double angleRad = moduleStates[i].angle.getRadians();
      if (!Double.isFinite(speedMps) || !Double.isFinite(angleRad)) {
        return Result.noSkid(moduleLocations.length);
      }
      double measVx = speedMps * Math.cos(angleRad);
      double measVy = speedMps * Math.sin(angleRad);
      Translation2d loc = moduleLocations[i];
      // Subtract pure rigid-body rotation: omega x r = (-omega * y, +omega * x).
      double transVx = measVx - (-omegaRadPerSec * loc.getY());
      double transVy = measVy - (omegaRadPerSec * loc.getX());
      double transSpeed = Math.hypot(transVx, transVy);
      transSpeeds[i] = transSpeed;
      if (transSpeed < minSpeed) {
        minSpeed = transSpeed;
      }
      if (transSpeed > maxSpeed) {
        maxSpeed = transSpeed;
      }
    }

    // Apply deadband to the slowest module so low-speed encoder noise cannot spike max / min.
    if (minSpeed < minTranslationalSpeedMps || minSpeed < MIN_DIVISOR_MPS) {
      return Result.noSkid(moduleLocations.length);
    }

    double ratio = maxSpeed / minSpeed;
    boolean isSkidding = ratio > skidRatioThreshold;
    boolean[] slipping = new boolean[moduleLocations.length];
    int slippingCount = 0;

    if (isSkidding) {
      double medianSpeed = median(transSpeeds);
      for (int i = 0; i < transSpeeds.length; i++) {
        double hi = Math.max(transSpeeds[i], medianSpeed);
        double lo = Math.max(MIN_DIVISOR_MPS, Math.min(transSpeeds[i], medianSpeed));
        if ((hi / lo) > skidRatioThreshold) {
          slipping[i] = true;
          slippingCount++;
        }
      }
      if (slippingCount == 0) {
        // Symmetric 2-vs-2 slip (e.g., both left wheels): flag the faster slipping pair.
        for (int i = 0; i < transSpeeds.length; i++) {
          if ((transSpeeds[i] / minSpeed) > skidRatioThreshold) {
            slipping[i] = true;
            slippingCount++;
          }
        }
      }
    }

    return new Result(ratio, isSkidding, slipping, slippingCount);
  }

  private static double median(double[] values) {
    double[] copy = values.clone();
    Arrays.sort(copy);
    int mid = copy.length / 2;
    return (copy.length % 2 == 0) ? 0.5 * (copy[mid - 1] + copy[mid]) : copy[mid];
  }
}
