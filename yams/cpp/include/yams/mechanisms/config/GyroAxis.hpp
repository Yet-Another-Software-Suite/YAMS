// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

#pragma once

#include <numbers>
#include <wpi/math/geometry/Quaternion.hpp>
#include <wpi/math/geometry/Rotation3d.hpp>
#include <wpi/units/angle.hpp>

namespace yams::mechanisms::config {

/**
 * The gyro axis that points up through the robot, for a gyro that is not mounted flat.
 *
 * A swerve drive needs the robot's heading, which is rotation about the axis pointing up through
 * the robot. A gyro mounted flat reports that as its yaw. A gyro mounted on its side reports it as
 * its roll or its pitch instead.
 */
enum class GyroAxis {
  /** The gyro's X axis points up. The gyro's Y axis is taken to point to the robot's left. */
  ROLL,
  /** The gyro's Y axis points up. The gyro's X axis is taken to point to the robot's front. */
  PITCH,
  /** The gyro's Z axis points up: the gyro is mounted flat. This is the default. */
  YAW
};

/**
 * Rotate a gyro's attitude into the robot's frame, so the yaw of the result is the robot's heading
 * and its roll and pitch are the robot's tilt, whichever way the gyro is mounted.
 *
 * @param axis The gyro axis that points up through the robot.
 * @param gyroAttitude The attitude the gyro reports.
 * @return The robot's attitude.
 */
inline wpi::math::Rotation3d ToRobotAttitude(GyroAxis axis,
                                             const wpi::math::Rotation3d& gyroAttitude) {
  if (axis == GyroAxis::YAW) {
    return gyroAttitude;
  }
  const wpi::units::radian_t zero{0};
  const wpi::units::radian_t quarterTurn{std::numbers::pi / 2};
  const wpi::math::Rotation3d gyroToRobot =
      axis == GyroAxis::ROLL ? wpi::math::Rotation3d{zero, -quarterTurn, zero}
                             : wpi::math::Rotation3d{quarterTurn, zero, zero};
  const auto q = gyroToRobot.GetQuaternion();
  return wpi::math::Rotation3d{q * gyroAttitude.GetQuaternion() * q.Inverse()};
}

}  // namespace yams::mechanisms::config
