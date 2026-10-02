// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package yams.core.mechanisms.swerve;

import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.RadiansPerSecond;

import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.kinematics.SwerveModuleVelocity;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.LinearVelocity;

/**
 * A {@link SwerveModuleVelocity} that also holds how fast the wheel is turning: the angular velocity
 * of the azimuth.
 */
public class SwerveModuleVelocityWithAzimuth extends SwerveModuleVelocity {
  /** Angular velocity of the azimuth, counterclockwise positive. */
  public AngularVelocity azimuthVelocity;

  /**
   * Create a module state with the azimuth's angular velocity.
   *
   * @param velocity        Drive velocity of the wheel, in meters per second.
   * @param angle           Angle of the wheel.
   * @param azimuthVelocity Angular velocity of the azimuth, counterclockwise positive.
   */
  public SwerveModuleVelocityWithAzimuth(double velocity, Rotation2d angle, AngularVelocity azimuthVelocity) {
    super(velocity, angle);
    this.azimuthVelocity = azimuthVelocity;
  }

  /**
   * Create a module state with the azimuth's angular velocity.
   *
   * @param velocity        Drive velocity of the wheel.
   * @param angle           Angle of the wheel.
   * @param azimuthVelocity Angular velocity of the azimuth, counterclockwise positive.
   */
  public SwerveModuleVelocityWithAzimuth(LinearVelocity velocity, Rotation2d angle, AngularVelocity azimuthVelocity) {
    super(velocity, angle);
    this.azimuthVelocity = azimuthVelocity;
  }

  /**
   * Optimize a module state like {@link SwerveModuleVelocity#optimize(Rotation2d)}, keeping the
   * orientation the wheel was last sent to unless the other is better by more than 30 degrees.
   *
   * @param desired       Module state to optimize.
   * @param measured      The wheel's measured state.
   * @param lastCommanded Angle the wheel was last sent to, or null if it has not been sent anywhere.
   * @return The optimized module state.
   */
  public static SwerveModuleVelocity optimize(SwerveModuleVelocity desired, SwerveModuleVelocityWithAzimuth measured, Rotation2d lastCommanded) {
    return optimize(desired, measured, lastCommanded, Degrees.of(30));
  }

  /**
   * Optimize a module state like {@link SwerveModuleVelocity#optimize(Rotation2d)}, keeping the
   * orientation the wheel was last sent to unless the other is better by more than
   * {@code hysteresis}. A wheel still turning can be near 90 degrees from both orientations of a
   * target that keeps moving, such as a path starting off in a new direction; choosing by its angle
   * alone would reverse it back and forth.
   *
   * @param desired       Module state to optimize.
   * @param measured      The wheel's measured state.
   * @param lastCommanded Angle the wheel was last sent to, or null if it has not been sent anywhere.
   * @param hysteresis    How much nearer the wheel the other orientation must be to turn it around.
   * @return The optimized module state.
   */
  public static SwerveModuleVelocity optimize(SwerveModuleVelocity desired, SwerveModuleVelocityWithAzimuth measured, Rotation2d lastCommanded, Angle hysteresis) {
    final SwerveModuleVelocity state = desired.optimize(measured.angle);
    if (lastCommanded == null || Math.abs(state.angle.minus(lastCommanded).getDegrees()) <= 90) {
      return state;
    }
    final SwerveModuleVelocity lastOrientation = new SwerveModuleVelocity(-state.velocity, state.angle.rotateBy(Rotation2d.PI));
    return Math.abs(lastOrientation.angle.minus(measured.angle).getDegrees()) < 90 + hysteresis.in(Degrees) ? lastOrientation : state;
  }

  @Override
  public String toString() {
    return String.format("SwerveModuleVelocityWithAzimuth(Velocity: %.2f m/s, Angle: %s, Azimuth velocity: %.2f rad/s)",
        velocity, angle, azimuthVelocity.in(RadiansPerSecond));
  }
}
