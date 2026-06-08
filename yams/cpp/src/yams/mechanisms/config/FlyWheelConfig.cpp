// Copyright (c) 2026 YAMS Contributors
// SPDX-License-Identifier: LGPL-3.0-or-later

#include "yams/mechanisms/config/FlyWheelConfig.hpp"

#include <optional>
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

FlyWheelConfig& FlyWheelConfig::WithSimColor(const frc::Color8Bit& color) {
  m_simColor = color;
  return *this;
}

FlyWheelConfig& FlyWheelConfig::WithLowerSoftLimit(units::degrees_per_second_t limit) {
  m_lowerSoftLimit = limit;
  return *this;
}

FlyWheelConfig& FlyWheelConfig::WithUpperSoftLimit(units::degrees_per_second_t limit) {
  m_upperSoftLimit = limit;
  return *this;
}

FlyWheelConfig& FlyWheelConfig::WithSoftLimits(units::degrees_per_second_t lower,
                                               units::degrees_per_second_t upper) {
  m_lowerSoftLimit = lower;
  m_upperSoftLimit = upper;
  return *this;
}

FlyWheelConfig& FlyWheelConfig::WithSpeedometerSimulation(units::degrees_per_second_t maxVelocity) {
  m_useSpeedometer = true;
  m_speedometerMaxVelocity = maxVelocity;
  return *this;
}

FlyWheelConfig& FlyWheelConfig::WithSpeedometerSimulation() {
  m_useSpeedometer = true;
  return *this;
}

motorcontrollers::SmartMotorController* FlyWheelConfig::GetMotorController() const { return m_smc; }

frc2::SubsystemBase* FlyWheelConfig::GetSubsystem() const { return m_subsystem; }

std::string FlyWheelConfig::GetTelemetryName() const { return m_telemetryName; }

std::optional<units::meter_t> FlyWheelConfig::GetRollerDiameter() const { return m_rollerDiameter; }

frc::Color8Bit FlyWheelConfig::GetSimColor() const { return m_simColor; }

bool FlyWheelConfig::IsUsingSpeedometerSimulation() const { return m_useSpeedometer; }

std::optional<units::degrees_per_second_t> FlyWheelConfig::GetSpeedometerMaxVelocity() const {
  return m_speedometerMaxVelocity;
}

std::optional<units::degrees_per_second_t> FlyWheelConfig::GetUpperSoftLimit() const {
  return m_upperSoftLimit;
}

std::optional<units::degrees_per_second_t> FlyWheelConfig::GetLowerSoftLimit() const {
  return m_lowerSoftLimit;
}

}  // namespace yams::mechanisms::config
