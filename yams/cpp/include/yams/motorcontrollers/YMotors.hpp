// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

#pragma once

#include <numbers>
#include <wpi/math/system/DCMotor.hpp>
#include <wpi/units/angular_velocity.hpp>
#include <wpi/units/current.hpp>
#include <wpi/units/torque.hpp>
#include <wpi/units/voltage.hpp>

namespace yams::motorcontrollers {

/**
 * DCMotor models of motors WPILib does not have a model of, like the Java YMotors.
 *
 * This will be deleted once WPILib has a DCMotor of the FIRST A301. Use WPILib's then.
 */
struct YMotors {
  /**
   * The motor inside the FIRST A301, before its gearbox, at 12 V: 13,500 RPM free, and 0.078 N-m
   * stalled, twice the 5.27 N-m at peak power of FIRST's 135:1 gearbox. FIRST does not give its
   * currents, and the A301 is similar to a NEO 550, so it has the NEO 550's: 100 A stalled and
   * 1.4 A free.
   *
   * Every A301 has this motor. Its gearbox is part of the gearing given to the A301Wrapper's
   * config.
   *
   * @param numMotors How many A301 motors are in the gearbox.
   * @return The FIRST A301's motor DCMotor.
   */
  static constexpr wpi::math::DCMotor A301(int numMotors = 1) {
    return wpi::math::DCMotor{
        wpi::units::volt_t{12.0},
        wpi::units::newton_meter_t{2.0 * 5.27 / 135.0},
        wpi::units::ampere_t{100.0},
        wpi::units::ampere_t{1.4},
        wpi::units::radians_per_second_t{13500.0 * 2.0 * std::numbers::pi / 60.0},
        numMotors};
  }
};

}  // namespace yams::motorcontrollers
