// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

#include "yams/mechanisms/config/PivotConfig.hpp"

#include <string>

namespace yams::mechanisms::config {

PivotConfig& PivotConfig::WithTelemetryName(const std::string& name) {
  m_telemetryName = name;
  return *this;
}

PivotConfig& PivotConfig::WithMinAngle(units::degree_t angle) {
  m_minAngle = angle;
  return *this;
}

PivotConfig& PivotConfig::WithMaxAngle(units::degree_t angle) {
  m_maxAngle = angle;
  return *this;
}

PivotConfig& PivotConfig::WithSimColor(const frc::Color8Bit& color) {
  m_simColor = color;
  return *this;
}

std::string PivotConfig::GetTelemetryName() const { return m_telemetryName; }

std::optional<units::degree_t> PivotConfig::GetMinAngle() const { return m_minAngle; }

std::optional<units::degree_t> PivotConfig::GetMaxAngle() const { return m_maxAngle; }

frc::Color8Bit PivotConfig::GetSimColor() const { return m_simColor; }

}  // namespace yams::mechanisms::config
