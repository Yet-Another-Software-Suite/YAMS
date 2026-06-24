// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

#include "yams/mechanisms/config/FlyWheelConfig.hpp"

#include <optional>
#include <string>

namespace yams::mechanisms::config {

FlyWheelConfig& FlyWheelConfig::WithTelemetryName(const std::string& name) {
  m_telemetryName = name;
  return *this;
}

FlyWheelConfig& FlyWheelConfig::WithRollerDiameter(units::meter_t diameter) {
  m_rollerDiameter = diameter;
  return *this;
}

FlyWheelConfig& FlyWheelConfig::WithSimColor(const frc::Color8Bit& color) {
  m_simColor = color;
  return *this;
}

std::string FlyWheelConfig::GetTelemetryName() const { return m_telemetryName; }

std::optional<units::meter_t> FlyWheelConfig::GetRollerDiameter() const { return m_rollerDiameter; }

frc::Color8Bit FlyWheelConfig::GetSimColor() const { return m_simColor; }

}  // namespace yams::mechanisms::config
