// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package yams.core.mechanisms.config.enums;

import org.wpilib.math.geometry.Rotation3d;

/**
 * Axis of the gyro that points up through the robot, so the robot turning shows up as rotation
 * about it: {@link #YAW} for a gyro mounted flat, {@link #ROLL} or {@link #PITCH} for one mounted
 * on its side.
 */
public enum GyroAxis {
  /**
   * The gyro's X axis points up: the robot turning is the gyro's roll. The gyro's Y axis is taken to
   * point to the robot's left, like WPILib's robot frame.
   */
  ROLL(new Rotation3d(0, -Math.PI / 2, 0)),
  /**
   * The gyro's Y axis points up: the robot turning is the gyro's pitch. The gyro's X axis is taken
   * to point to the robot's front, like WPILib's robot frame.
   */
  PITCH(new Rotation3d(Math.PI / 2, 0, 0)),
  /** The gyro's Z axis points up: the robot turning is the gyro's yaw. The default. */
  YAW(new Rotation3d());

  /**
   * Rotation from the gyro's frame to the robot's.
   */
  private final Rotation3d gyroToRobot;

  GyroAxis(Rotation3d gyroToRobot) {
    this.gyroToRobot = gyroToRobot;
  }

  /**
   * Rotate the gyro's attitude into the robot's frame, so its yaw is the robot's heading and its
   * roll and pitch are the robot's tilt, whichever way the gyro is mounted. Reading the heading as
   * the robot frame yaw also keeps its full half rotation range each way, which a {@link Rotation3d}'s
   * pitch does not have.
   *
   * @param gyroAttitude Attitude reported by the gyro.
   * @return The robot's attitude.
   */
  public Rotation3d toRobotAttitude(Rotation3d gyroAttitude) {
    if (this == YAW) {
      return gyroAttitude;
    }
    // Change of frame: the same rotation, expressed about the robot's axes instead of the gyro's.
    final var q = gyroToRobot.getQuaternion();
    return new Rotation3d(q.times(gyroAttitude.getQuaternion()).times(q.inverse()));
  }
}
