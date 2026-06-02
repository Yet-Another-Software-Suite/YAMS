// Copyright (c) 2026 YAMS Contributors
// SPDX-License-Identifier: LGPL-3.0-or-later

#include "yams/mechanisms/config/ArmConfig.h"

#include <string>

namespace yams::mechanisms::config {

ArmConfig& ArmConfig::WithMotorController(motorcontrollers::SmartMotorController* smc) {
  m_smc = smc;
  return *this;
}

ArmConfig& ArmConfig::WithSubsystem(frc2::SubsystemBase* subsystem) {
  m_subsystem = subsystem;
  return *this;
}

ArmConfig& ArmConfig::WithTelemetryName(const std::string& name) {
  m_telemetryName = name;
  return *this;
}

ArmConfig& ArmConfig::WithStartingAngle(units::degree_t angle) {
  m_startingAngle = angle;
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

motorcontrollers::SmartMotorController* ArmConfig::GetMotorController() const { return m_smc; }

frc2::SubsystemBase* ArmConfig::GetSubsystem() const { return m_subsystem; }

std::string ArmConfig::GetTelemetryName() const { return m_telemetryName; }

std::optional<units::degree_t> ArmConfig::GetStartingAngle() const { return m_startingAngle; }

std::optional<units::degree_t> ArmConfig::GetMinAngle() const { return m_minAngle; }

std::optional<units::degree_t> ArmConfig::GetMaxAngle() const { return m_maxAngle; }

std::optional<units::meter_t> ArmConfig::GetArmLength() const { return m_armLength; }

}  // namespace yams::mechanisms::config
