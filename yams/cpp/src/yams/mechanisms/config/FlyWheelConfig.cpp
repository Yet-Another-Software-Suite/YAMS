// Copyright (c) 2026 YAMS Contributors
// SPDX-License-Identifier: LGPL-3.0-or-later

#include "yams/mechanisms/config/FlyWheelConfig.hpp"

#include <string>

namespace yams::mechanisms::config {

FlyWheelConfig& FlyWheelConfig::WithMotorController(motorcontrollers::SmartMotorController* smc) {
  m_smc = smc;
  return *this;
}

FlyWheelConfig& FlyWheelConfig::WithSubsystem(frc2::SubsystemBase* subsystem) {
  m_subsystem = subsystem;
  return *this;
}

FlyWheelConfig& FlyWheelConfig::WithTelemetryName(const std::string& name) {
  m_telemetryName = name;
  return *this;
}

FlyWheelConfig& FlyWheelConfig::WithRollerDiameter(units::meter_t diameter) {
  m_rollerDiameter = diameter;
  return *this;
}

motorcontrollers::SmartMotorController* FlyWheelConfig::GetMotorController() const { return m_smc; }

frc2::SubsystemBase* FlyWheelConfig::GetSubsystem() const { return m_subsystem; }

std::string FlyWheelConfig::GetTelemetryName() const { return m_telemetryName; }

std::optional<units::meter_t> FlyWheelConfig::GetRollerDiameter() const { return m_rollerDiameter; }

}  // namespace yams::mechanisms::config
