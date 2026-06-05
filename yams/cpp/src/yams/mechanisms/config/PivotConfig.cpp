// Copyright (c) 2026 YAMS Contributors
// SPDX-License-Identifier: LGPL-3.0-or-later

#include "yams/mechanisms/config/PivotConfig.hpp"

#include <string>

namespace yams::mechanisms::config {

PivotConfig& PivotConfig::WithMotorController(motorcontrollers::SmartMotorController* smc) {
  m_smc = smc;
  return *this;
}

PivotConfig& PivotConfig::WithSubsystem(frc2::SubsystemBase* subsystem) {
  m_subsystem = subsystem;
  return *this;
}

PivotConfig& PivotConfig::WithTelemetryName(const std::string& name) {
  m_telemetryName = name;
  return *this;
}

PivotConfig& PivotConfig::WithStartingAngle(units::degree_t angle) {
  m_startingAngle = angle;
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

motorcontrollers::SmartMotorController* PivotConfig::GetMotorController() const { return m_smc; }

frc2::SubsystemBase* PivotConfig::GetSubsystem() const { return m_subsystem; }

std::string PivotConfig::GetTelemetryName() const { return m_telemetryName; }

std::optional<units::degree_t> PivotConfig::GetStartingAngle() const { return m_startingAngle; }

std::optional<units::degree_t> PivotConfig::GetMinAngle() const { return m_minAngle; }

std::optional<units::degree_t> PivotConfig::GetMaxAngle() const { return m_maxAngle; }

}  // namespace yams::mechanisms::config
