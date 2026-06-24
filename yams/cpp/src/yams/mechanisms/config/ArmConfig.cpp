// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

#include "yams/mechanisms/config/ArmConfig.hpp"

#include <string>

namespace yams::mechanisms::config {

ArmConfig& ArmConfig::WithTelemetryName(const std::string& name) {
  m_telemetryName = name;
  return *this;
}

ArmConfig& ArmConfig::WithMinAngle(units::degree_t angle) {
  m_minAngle = angle;
  return *this;
}

ArmConfig& ArmConfig::WithMaxAngle(units::degree_t angle) {
  m_maxAngle = angle;
  return *this;
}

ArmConfig& ArmConfig::WithArmLength(units::meter_t length) {
  m_armLength = length;
  return *this;
}

ArmConfig& ArmConfig::WithSimColor(const frc::Color8Bit& color) {
  m_simColor = color;
  return *this;
}

std::string ArmConfig::GetTelemetryName() const { return m_telemetryName; }

std::optional<units::degree_t> ArmConfig::GetMinAngle() const { return m_minAngle; }

std::optional<units::degree_t> ArmConfig::GetMaxAngle() const { return m_maxAngle; }

std::optional<units::meter_t> ArmConfig::GetArmLength() const { return m_armLength; }

frc::Color8Bit ArmConfig::GetSimColor() const { return m_simColor; }

}  // namespace yams::mechanisms::config
