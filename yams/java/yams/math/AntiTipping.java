// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package yams.math;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/**
 * A proportional correction that drives the robot back under itself when it starts to tip. It is a
 * port of the {@code AntiTipping} controller WPILib ships from 2027 on, which this WPILib does not
 * have: <a
 * href="https://github.com/wpilibsuite/allwpilib/blob/main/wpimath/src/main/java/org/wpilib/math/controller/AntiTipping.java">
 * wpimath/src/main/java/org/wpilib/math/controller/AntiTipping.java</a> in wpilibsuite/allwpilib,
 * which is under the WPILib BSD license.
 *
 * <p>Give it the robot's attitude every loop and add the correction it returns to the drive's
 * speeds. The correction is zero until the robot is tilted past the threshold. Past it, the robot
 * is driven toward the side it is falling to at {@code kP * sin(tilt)}, up to the maximum
 * correction speed.
 *
 * <p>Tuning: set the threshold a few degrees above the largest tilt seen in hard, normal driving,
 * raise kP until the recovery is brisk without overshooting, and cap the correction at a fraction of
 * the drive's top speed so it never overpowers the driver.
 */
public class AntiTipping {
  /** Correction speed per unit of sin(tilt), in meters per second. */
  private double m_kP;

  /** Tilt past which the correction starts, in radians. */
  private double m_tippingThreshold;

  /** Largest correction speed, in meters per second. */
  private double m_maxCorrectionSpeed;

  /**
   * Create an anti-tipping correction.
   *
   * @param kP                 Correction speed per unit of sin(tilt), in meters per second.
   * @param tippingThreshold   Tilt past which the correction starts, in radians.
   * @param maxCorrectionSpeed Largest correction speed, in meters per second.
   */
  public AntiTipping(double kP, double tippingThreshold, double maxCorrectionSpeed) {
    m_kP = kP;
    m_tippingThreshold = tippingThreshold;
    m_maxCorrectionSpeed = maxCorrectionSpeed;
  }

  /**
   * Set the proportional gain.
   *
   * @param kP Correction speed per unit of sin(tilt), in meters per second.
   */
  public void setP(double kP) {
    m_kP = kP;
  }

  /**
   * Get the proportional gain.
   *
   * @return Correction speed per unit of sin(tilt), in meters per second.
   */
  public double getP() {
    return m_kP;
  }

  /**
   * Set the tilt past which the correction starts.
   *
   * @param threshold Threshold in radians.
   */
  public void setTippingThreshold(double threshold) {
    m_tippingThreshold = threshold;
  }

  /**
   * Get the tilt past which the correction starts.
   *
   * @return Threshold in radians.
   */
  public double getTippingThreshold() {
    return m_tippingThreshold;
  }

  /**
   * Set the largest correction speed.
   *
   * @param speed Speed in meters per second.
   */
  public void setMaxCorrectionSpeed(double speed) {
    m_maxCorrectionSpeed = speed;
  }

  /**
   * Get the largest correction speed.
   *
   * @return Speed in meters per second.
   */
  public double getMaxCorrectionSpeed() {
    return m_maxCorrectionSpeed;
  }

  /**
   * Calculate the correction for the robot's attitude.
   *
   * @param attitude The robot's attitude.
   * @return {@link ChassisSpeeds} to add to the drive's speeds, in the frame the attitude's yaw is
   *         measured in. Zero while the tilt is below the threshold.
   */
  public ChassisSpeeds calculate(Rotation3d attitude) {
    // The robot's up axis, scaled by the gain and tilted with the robot, leans over the side the
    // robot is falling to. Its shadow on the floor is the correction, of length kP * sin(tilt).
    Translation2d correction = new Translation3d(0.0, 0.0, m_kP).rotateBy(attitude).toTranslation2d();
    final double norm = correction.getNorm();
    final double tilt = Math.asin(Math.min(1.0, norm / m_kP));
    if (tilt < m_tippingThreshold) {
      return new ChassisSpeeds();
    }
    if (norm > m_maxCorrectionSpeed) {
      correction = correction.times(m_maxCorrectionSpeed / norm);
    }
    return new ChassisSpeeds(correction.getX(), correction.getY(), 0.0);
  }
}
