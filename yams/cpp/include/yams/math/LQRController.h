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

class LQRController {
 public:
  LQRController(LQRConfig::LQRType type, std::variant<LQRConfig::Loop1, LQRConfig::Loop2> loop,
                units::second_t period);
  explicit LQRController(LQRConfig config);

  void UpdateConfig(LQRConfig config);

  void Reset(units::radian_t angle, units::radians_per_second_t velocity);
  void Reset(units::meter_t distance, units::meters_per_second_t velocity);

  units::volt_t Calculate(units::radian_t measured, units::radian_t position,
                          units::radians_per_second_t velocity);
  units::volt_t Calculate(units::meter_t measured, units::meter_t position,
                          units::meters_per_second_t velocity);
  units::volt_t Calculate(units::radians_per_second_t measured,
                          units::radians_per_second_t velocity);
  units::volt_t Calculate(units::meters_per_second_t measured, units::meters_per_second_t velocity);

  LQRConfig::LQRType GetType() const;
  const std::optional<LQRConfig>& GetConfig() const;

 private:
  std::optional<LQRConfig> m_config;
  LQRConfig::LQRType m_type;
  std::variant<LQRConfig::Loop1, LQRConfig::Loop2> m_loop;
  units::second_t m_period;
};

}  // namespace yams::math
