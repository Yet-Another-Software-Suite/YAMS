// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package first.robot;

import static org.wpilib.units.Units.Gs;
import static org.wpilib.units.Units.Seconds;

import java.util.Objects;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Translation3d;
import org.wpilib.units.measure.LinearAcceleration;
import org.wpilib.units.measure.Time;

/**
 * Detects robot collisions using 3D gyro-rotated gravity subtraction and 3D linear jerk
 * thresholding, with NaN-safe latch retention and configurable cooldown.
 */
public class CollisionJerkMonitor {
  private static final double ONE_G = 1.0;
  private static final double MIN_DT_SECONDS = 1e-4;

  /** Immutable configuration for {@link CollisionJerkMonitor}. */
  public record Config(
      double jerkThresholdGsPerSecond,
      double minImpactLinearAccelGs,
      Time bumpLatchDuration,
      Time collisionCooldown) {
    public Config {
      Objects.requireNonNull(bumpLatchDuration, "bumpLatchDuration must not be null");
      Objects.requireNonNull(collisionCooldown, "collisionCooldown must not be null");
      if (!Double.isFinite(jerkThresholdGsPerSecond) || jerkThresholdGsPerSecond <= 0.0) {
        throw new IllegalArgumentException("jerkThresholdGsPerSecond must be positive");
      }
      if (!Double.isFinite(minImpactLinearAccelGs) || minImpactLinearAccelGs < 0.0) {
        throw new IllegalArgumentException("minImpactLinearAccelGs must be non-negative");
      }
      if (bumpLatchDuration.in(Seconds) < 0.0 || collisionCooldown.in(Seconds) < 0.0) {
        throw new IllegalArgumentException("latch and cooldown durations must be non-negative");
      }
    }
  }

  private final double jerkThresholdGsPerSecond;
  private final double minImpactLinearAccelGs;
  private final double bumpLatchSeconds;
  private final double collisionCooldownSeconds;

  private boolean hasLastSample;
  private double lastTimestampSeconds;
  private Translation3d lastLinearAccelGs = new Translation3d();
  private double lastLinearAccelMagGs;
  private double lastJerkGsPerSecond;

  private boolean hasLastCollision;
  private double lastCollisionTimestampSeconds;
  private boolean bumpedRecently;
  private int collisionCount;

  public CollisionJerkMonitor(Config config) {
    Objects.requireNonNull(config, "config must not be null");
    this.jerkThresholdGsPerSecond = config.jerkThresholdGsPerSecond();
    this.minImpactLinearAccelGs = config.minImpactLinearAccelGs();
    this.bumpLatchSeconds = config.bumpLatchDuration().in(Seconds);
    this.collisionCooldownSeconds = config.collisionCooldown().in(Seconds);
  }

  /**
   * Subtracts the 3D gyro-rotated gravity vector (in g) from a body-frame IMU acceleration reading
   * to recover true 3D linear acceleration (in g).
   */
  public static Translation3d removeGravity(
      Rotation3d attitude,
      LinearAcceleration accelX,
      LinearAcceleration accelY,
      LinearAcceleration accelZ) {
    Objects.requireNonNull(attitude, "attitude must not be null");
    Objects.requireNonNull(accelX, "accelX must not be null");
    Objects.requireNonNull(accelY, "accelY must not be null");
    Objects.requireNonNull(accelZ, "accelZ must not be null");

    Translation3d gravityInBodyGs =
        new Translation3d(0.0, 0.0, ONE_G).rotateBy(new Rotation3d(attitude.getQuaternion().inverse()));
    return new Translation3d(
        accelX.in(Gs) - gravityInBodyGs.getX(),
        accelY.in(Gs) - gravityInBodyGs.getY(),
        accelZ.in(Gs) - gravityInBodyGs.getZ());
  }

  /**
   * Updates the collision detector with a timestamped 3D attitude and body-frame IMU acceleration.
   * Non-finite inputs are ignored while preserving any active {@code bumpedRecently} latch.
   *
   * @return {@code true} if the robot is currently within the {@code bumpedRecently} latch window.
   */
  public boolean update(
      Time timestamp,
      Rotation3d attitude,
      LinearAcceleration accelX,
      LinearAcceleration accelY,
      LinearAcceleration accelZ) {
    Objects.requireNonNull(timestamp, "timestamp must not be null");
    Objects.requireNonNull(attitude, "attitude must not be null");
    Objects.requireNonNull(accelX, "accelX must not be null");
    Objects.requireNonNull(accelY, "accelY must not be null");
    Objects.requireNonNull(accelZ, "accelZ must not be null");

    double nowSeconds = timestamp.in(Seconds);
    double ax = accelX.in(Gs);
    double ay = accelY.in(Gs);
    double az = accelZ.in(Gs);

    if (!Double.isFinite(nowSeconds)
        || !Double.isFinite(ax)
        || !Double.isFinite(ay)
        || !Double.isFinite(az)) {
      return bumpedRecently;
    }

    Translation3d linearAccelGs = removeGravity(attitude, accelX, accelY, accelZ);
    double linearMagGs = linearAccelGs.getNorm();

    if (hasLastSample) {
      double dtSeconds = nowSeconds - lastTimestampSeconds;
      if (dtSeconds >= MIN_DT_SECONDS) {
        double jerkGsPerSec = linearAccelGs.minus(lastLinearAccelGs).getNorm() / dtSeconds;
        lastJerkGsPerSecond = jerkGsPerSec;

        boolean cooldownElapsed =
            !hasLastCollision || (nowSeconds - lastCollisionTimestampSeconds) >= collisionCooldownSeconds;
        if (cooldownElapsed
            && jerkGsPerSec >= jerkThresholdGsPerSecond
            && linearMagGs >= minImpactLinearAccelGs) {
          hasLastCollision = true;
          lastCollisionTimestampSeconds = nowSeconds;
          collisionCount++;
        }
      }
    } else {
      lastJerkGsPerSecond = 0.0;
    }

    lastTimestampSeconds = nowSeconds;
    lastLinearAccelGs = linearAccelGs;
    lastLinearAccelMagGs = linearMagGs;
    hasLastSample = true;

    bumpedRecently =
        hasLastCollision && (nowSeconds - lastCollisionTimestampSeconds) <= bumpLatchSeconds;
    return bumpedRecently;
  }

  /** Resets sample history, collision counter, and bump latch. */
  public void reset() {
    hasLastSample = false;
    lastTimestampSeconds = 0.0;
    lastLinearAccelGs = new Translation3d();
    lastLinearAccelMagGs = 0.0;
    lastJerkGsPerSecond = 0.0;
    hasLastCollision = false;
    lastCollisionTimestampSeconds = 0.0;
    bumpedRecently = false;
    collisionCount = 0;
  }

  public boolean isBumpedRecently() {
    return bumpedRecently;
  }

  public int getCollisionCount() {
    return collisionCount;
  }

  public double getLastJerkGsPerSecond() {
    return lastJerkGsPerSecond;
  }

  public double getLastLinearAccelGs() {
    return lastLinearAccelMagGs;
  }

  public Translation3d getLastLinearAccelVectorGs() {
    return lastLinearAccelGs;
  }
}
