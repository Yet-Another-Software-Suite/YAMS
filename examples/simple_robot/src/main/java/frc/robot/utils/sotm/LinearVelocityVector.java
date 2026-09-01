// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package frc.robot.utils.sotm;

import static org.wpilib.units.Units.RPM;
import static org.wpilib.units.Units.Seconds;

import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.kinematics.ChassisSpeeds;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Time;

/**
 * Linear velocity vector of the robot.
 */
public class LinearVelocityVector
{
  /**
   * Position of the robot (WPILib coordinate system).
   */
  public final Pose2d          position;
  /**
   * Target pose (WPILib coordinate system).
   */
  public final Pose2d          target;
  /**
   * Velocity of the robot.
   */
  public final ChassisSpeeds   velocity;
  /**
   * Flywheel velocity.
   */
  public final AngularVelocity flywheelVelocity;

  /**
   * Create a new linear velocity vector.
   *
   * @param position Position of the robot.
   * @param target   Target to shoot at.
   * @param velocity Velocity of the robot.
   */
  public LinearVelocityVector(Pose2d position, Pose2d target, ChassisSpeeds velocity, AngularVelocity flywheelVelocity)
  {
    this.position = position;
    this.target = target;
    this.velocity = velocity;
    this.flywheelVelocity = flywheelVelocity;
  }

  /**
   * Create a new linear velocity vector with a {@link ChassisSpeeds} of zero.
   *
   * @param position         Position of the robot.
   * @param target           Target to shoot at.
   * @param flywheelVelocity Velocity of the flywheel.
   */
  public LinearVelocityVector(Pose2d position, Pose2d target, AngularVelocity flywheelVelocity)
  {
    this.position = position;
    this.target = target;
    this.velocity = new ChassisSpeeds();
    this.flywheelVelocity = flywheelVelocity;
  }

  /**
   * Create a new linear velocity vector with a {@link AngularVelocity} of zero.
   *
   * @param position Position of the robot.
   * @param target   Target to shoot at.
   * @param velocity Velocity of the robot.
   */
  public LinearVelocityVector(Pose2d position, Pose2d target, ChassisSpeeds velocity)
  {
    this.position = position;
    this.target = target;
    this.velocity = velocity;
    this.flywheelVelocity = RPM.zero();
  }

  /**
   * Create a new linear velocity vector with a {@link ChassisSpeeds} and {@link AngularVelocity} of zero.
   *
   * @param position Position of the robot.
   * @param target   Target to shoot at.
   */
  public LinearVelocityVector(Pose2d position, Pose2d target)
  {
    this.position = position;
    this.target = target;
    this.velocity = new ChassisSpeeds();
    this.flywheelVelocity = RPM.zero();
  }

  /**
   * Get the linear velocity of the robot from {@link ChassisSpeeds} to {@link Translation2d}.
   *
   * @return {@link Translation2d} of the robot's linear velocity field oriented (WPILib coordinate system).
   */
  public Translation2d getLinearVelocity()
  {
    return new Translation2d(velocity.vxMetersPerSecond, velocity.vyMetersPerSecond);
  }

  /**
   * Estimate the robot's pose based on the linear velocity vector.
   *
   * @param latencyCompensation Latency compensation in time.
   * @return Estimated pose.
   */
  public Translation2d estimatePose(Time latencyCompensation)
  {
    return position.getTranslation().plus(getLinearVelocity().times(latencyCompensation.in(Seconds)));
  }

}
