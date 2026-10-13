// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

#pragma once

#include <optional>
#include <wpi/math/geometry/Rotation2d.hpp>
#include <wpi/math/kinematics/SwerveModuleVelocity.hpp>
#include <wpi/units/angle.hpp>
#include <wpi/units/angular_velocity.hpp>
#include <wpi/units/math.hpp>
#include <wpi/units/velocity.hpp>

namespace yams::mechanisms::swerve {

/**
 * A swerve module velocity that also carries how fast the azimuth is turning.
 */
struct SwerveModuleVelocityWithAzimuth : public wpi::math::SwerveModuleVelocity {
  /** How fast the azimuth is turning. */
  wpi::units::radians_per_second_t azimuthVelocity{0};

  SwerveModuleVelocityWithAzimuth() = default;

  /**
   * @param velocity Wheel velocity.
   * @param angle Azimuth angle.
   * @param azimuthVelocity How fast the azimuth is turning.
   */
  SwerveModuleVelocityWithAzimuth(wpi::units::meters_per_second_t velocity,
                                  wpi::math::Rotation2d angle,
                                  wpi::units::radians_per_second_t azimuthVelocity)
      : wpi::math::SwerveModuleVelocity{velocity, angle}, azimuthVelocity{azimuthVelocity} {}

  /**
   * Optimize a desired state against the measured one, keeping the orientation the wheel was last
   * sent to unless the other is better by more than the hysteresis. Without it a wheel that is
   * still turning reverses back and forth as the target moves across the 90 degree boundary.
   *
   * @param desired Desired state.
   * @param measured Measured state.
   * @param lastCommanded Angle the wheel was last sent to, if any.
   * @param hysteresis How much better the other orientation must be before the wheel reverses.
   * @return The optimized state.
   */
  static wpi::math::SwerveModuleVelocity Optimize(
      const wpi::math::SwerveModuleVelocity& desired,
      const SwerveModuleVelocityWithAzimuth& measured,
      const std::optional<wpi::math::Rotation2d>& lastCommanded,
      wpi::units::degree_t hysteresis = wpi::units::degree_t{30}) {
    wpi::math::SwerveModuleVelocity state = desired.Optimize(measured.angle);
    if (!lastCommanded || wpi::units::math::abs((state.angle - *lastCommanded).Degrees()) <=
                              wpi::units::degree_t{90}) {
      return state;
    }
    // The other orientation is the one the wheel was last sent to.
    wpi::math::SwerveModuleVelocity lastOrientation{
        -state.velocity, state.angle.RotateBy(wpi::math::Rotation2d{wpi::units::degree_t{180}})};
    return wpi::units::math::abs((lastOrientation.angle - measured.angle).Degrees()) <
                   wpi::units::degree_t{90} + hysteresis
               ? lastOrientation
               : state;
  }
};

}  // namespace yams::mechanisms::swerve
