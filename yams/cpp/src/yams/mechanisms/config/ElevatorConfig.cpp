// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

#include "yams/mechanisms/config/ElevatorConfig.hpp"

#include <wpi/util/Color8Bit.hpp>
#include <wpi/units/angle.hpp>
#include <wpi/units/length.hpp>
#include <wpi/units/mass.hpp>

#include <string>

namespace yams::mechanisms::config {

ElevatorConfig& ElevatorConfig::WithTelemetryName(const std::string& name) {
  m_telemetryName = name;
  return *this;
}

ElevatorConfig& ElevatorConfig::WithMinimumHeight(wpi::units::meter_t height) {
  m_minHeight = height;
  return *this;
}

ElevatorConfig& ElevatorConfig::WithMaximumHeight(wpi::units::meter_t height) {
  m_maxHeight = height;
  return *this;
}

ElevatorConfig& ElevatorConfig::WithCarriageMass(wpi::units::kilogram_t mass) {
  m_carriageMass = mass;
  return *this;
}

ElevatorConfig& ElevatorConfig::WithIsHorizontal(bool horizontal) {
  m_isHorizontal = horizontal;
  return *this;
}

ElevatorConfig& ElevatorConfig::WithSimColor(const wpi::util::Color8Bit& color) {
  m_simColor = color;
  return *this;
}

ElevatorConfig& ElevatorConfig::WithAngle(wpi::units::degree_t angle) {
  m_angle = angle;
  return *this;
}

std::string ElevatorConfig::GetTelemetryName() const { return m_telemetryName; }

std::optional<wpi::units::meter_t> ElevatorConfig::GetMinHeight() const { return m_minHeight; }

std::optional<wpi::units::meter_t> ElevatorConfig::GetMaxHeight() const { return m_maxHeight; }

std::optional<wpi::units::kilogram_t> ElevatorConfig::GetCarriageMass() const { return m_carriageMass; }

bool ElevatorConfig::IsHorizontal() const { return m_isHorizontal; }

wpi::util::Color8Bit ElevatorConfig::GetSimColor() const { return m_simColor; }

wpi::units::degree_t ElevatorConfig::GetAngle() const { return m_angle; }

}  // namespace yams::mechanisms::config
