// Copyright (c) 2026 YAMS Contributors
// SPDX-License-Identifier: LGPL-3.0-or-later

#pragma once

#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/length.h>
#include <units/time.h>
#include <units/velocity.h>
#include <units/voltage.h>

#include <optional>
#include <variant>

#include "LQRConfig.h"

namespace yams::math {

/**
 * Linear Quadratic Regulator controller wrapper.
 *
 * Wraps a WPILib LinearSystemLoop to provide a common Calculate() interface for
 * flywheel (velocity), arm, and elevator (position + velocity) plant types.
 */
class LQRController {
 public:
  /**
   * Construct an LQRController from an already-built loop.
   *
   * @param type   Plant type (FLYWHEEL, ARM, or ELEVATOR).
   * @param loop   Pre-built LinearSystemLoop variant.
   * @param period Loop period.
   */
  LQRController(LQRConfig::LQRType type, std::variant<LQRConfig::Loop1, LQRConfig::Loop2> loop,
                units::second_t period);

  /**
   * Construct an LQRController by building the loop from an LQRConfig.
   *
   * @param config Fully-configured LQRConfig.
   */
  explicit LQRController(LQRConfig config);

  /**
   * Replace the current LQR configuration and rebuild the internal loop.
   *
   * @param config New LQRConfig to apply.
   */
  void UpdateConfig(LQRConfig config);

  /**
   * Reset the angular position+velocity state of the controller (arm/elevator).
   *
   * @param angle    Current mechanism angle.
   * @param velocity Current mechanism velocity.
   */
  void Reset(units::radian_t angle, units::radians_per_second_t velocity);

  /**
   * Reset the linear position+velocity state of the controller (elevator).
   *
   * @param distance Current mechanism distance.
   * @param velocity Current mechanism linear velocity.
   */
  void Reset(units::meter_t distance, units::meters_per_second_t velocity);

  /**
   * Calculate the next voltage output for an angular positional mechanism (arm).
   *
   * @param measured  Current measured angle.
   * @param position  Target angle setpoint.
   * @param velocity  Target angular velocity setpoint.
   * @return Voltage to apply to the motor.
   */
  units::volt_t Calculate(units::radian_t measured, units::radian_t position,
                          units::radians_per_second_t velocity);

  /**
   * Calculate the next voltage output for a linear positional mechanism (elevator).
   *
   * @param measured  Current measured distance.
   * @param position  Target distance setpoint.
   * @param velocity  Target linear velocity setpoint.
   * @return Voltage to apply to the motor.
   */
  units::volt_t Calculate(units::meter_t measured, units::meter_t position,
                          units::meters_per_second_t velocity);

  /**
   * Calculate the next voltage output for an angular velocity mechanism (flywheel).
   *
   * @param measured  Current measured angular velocity.
   * @param velocity  Target angular velocity setpoint.
   * @return Voltage to apply to the motor.
   */
  units::volt_t Calculate(units::radians_per_second_t measured,
                          units::radians_per_second_t velocity);

  /**
   * Calculate the next voltage output for a linear velocity mechanism.
   *
   * @param measured  Current measured linear velocity.
   * @param velocity  Target linear velocity setpoint.
   * @return Voltage to apply to the motor.
   */
  units::volt_t Calculate(units::meters_per_second_t measured, units::meters_per_second_t velocity);

  /**
   * Get the configured LQR plant type.
   *
   * @return LQRType.
   */
  LQRConfig::LQRType GetType() const;

  /**
   * Get the optional LQRConfig used to construct this controller.
   *
   * @return Reference to the optional config.
   */
  const std::optional<LQRConfig>& GetConfig() const;

 private:
  std::optional<LQRConfig> m_config;
  LQRConfig::LQRType m_type;
  std::variant<LQRConfig::Loop1, LQRConfig::Loop2> m_loop;
  units::second_t m_period;
};

}  // namespace yams::math
