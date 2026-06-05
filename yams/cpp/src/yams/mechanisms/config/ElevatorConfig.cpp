// Copyright (c) 2026 YAMS Contributors
// SPDX-License-Identifier: LGPL-3.0-or-later

#include "yams/mechanisms/config/ElevatorConfig.hpp"

#include <string>

namespace yams::mechanisms::config {

ElevatorConfig& ElevatorConfig::WithMotorController(motorcontrollers::SmartMotorController* smc) {
  m_smc = smc;
  return *this;
}

ElevatorConfig& ElevatorConfig::WithSubsystem(frc2::SubsystemBase* subsystem) {
  m_subsystem = subsystem;
  return *this;
}

ElevatorConfig& ElevatorConfig::WithTelemetryName(const std::string& name) {
  m_telemetryName = name;
  return *this;
}

ElevatorConfig& ElevatorConfig::WithStartingHeight(units::meter_t height) {
  m_startingHeight = height;
  return *this;
}

ElevatorConfig& ElevatorConfig::WithMinimumHeight(units::meter_t height) {
  m_minHeight = height;
  return *this;
}

ElevatorConfig& ElevatorConfig::WithMaximumHeight(units::meter_t height) {
  m_maxHeight = height;
  return *this;
}

motorcontrollers::SmartMotorController* ElevatorConfig::GetMotorController() const { return m_smc; }

frc2::SubsystemBase* ElevatorConfig::GetSubsystem() const { return m_subsystem; }

std::string ElevatorConfig::GetTelemetryName() const { return m_telemetryName; }

std::optional<units::meter_t> ElevatorConfig::GetStartingHeight() const { return m_startingHeight; }

std::optional<units::meter_t> ElevatorConfig::GetMinHeight() const { return m_minHeight; }

std::optional<units::meter_t> ElevatorConfig::GetMaxHeight() const { return m_maxHeight; }

}  // namespace yams::mechanisms::config
