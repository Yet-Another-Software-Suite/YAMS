// Copyright (c) 2026 YAMS Contributors
// SPDX-License-Identifier: LGPL-3.0-or-later

#include "yams/mechanisms/config/ElevatorConfig.hpp"

#include <frc/util/Color8Bit.h>
#include <units/angle.h>
#include <units/length.h>
#include <units/mass.h>

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

ElevatorConfig& ElevatorConfig::WithCarriageMass(units::kilogram_t mass) {
  m_carriageMass = mass;
  return *this;
}

ElevatorConfig& ElevatorConfig::WithDrumRadius(units::meter_t radius) {
  m_drumRadius = radius;
  return *this;
}

ElevatorConfig& ElevatorConfig::WithIsHorizontal(bool horizontal) {
  m_isHorizontal = horizontal;
  return *this;
}

ElevatorConfig& ElevatorConfig::WithSimColor(const frc::Color8Bit& color) {
  m_simColor = color;
  return *this;
}

ElevatorConfig& ElevatorConfig::WithAngle(units::degree_t angle) {
  m_angle = angle;
  return *this;
}

motorcontrollers::SmartMotorController* ElevatorConfig::GetMotorController() const { return m_smc; }

frc2::SubsystemBase* ElevatorConfig::GetSubsystem() const { return m_subsystem; }

std::string ElevatorConfig::GetTelemetryName() const { return m_telemetryName; }

std::optional<units::meter_t> ElevatorConfig::GetStartingHeight() const { return m_startingHeight; }

std::optional<units::meter_t> ElevatorConfig::GetMinHeight() const { return m_minHeight; }

std::optional<units::meter_t> ElevatorConfig::GetMaxHeight() const { return m_maxHeight; }

std::optional<units::kilogram_t> ElevatorConfig::GetCarriageMass() const { return m_carriageMass; }

std::optional<units::meter_t> ElevatorConfig::GetDrumRadius() const { return m_drumRadius; }

bool ElevatorConfig::IsHorizontal() const { return m_isHorizontal; }

frc::Color8Bit ElevatorConfig::GetSimColor() const { return m_simColor; }

units::degree_t ElevatorConfig::GetAngle() const { return m_angle; }

}  // namespace yams::mechanisms::config
