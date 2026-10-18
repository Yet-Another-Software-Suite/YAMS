// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

#include "yams/mechanisms/config/ElevatorConfig.hpp"

#include <stdexcept>
#include <string>
#include <utility>
#include <vector>
#include <wpi/units/angle.hpp>
#include <wpi/units/length.hpp>
#include <wpi/units/mass.hpp>
#include <wpi/util/Color8Bit.hpp>

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

ElevatorConfig& ElevatorConfig::WithStages(int stages, std::vector<wpi::units::meter_t> stageHeights) {
  if (stages < 1) {
    throw std::invalid_argument("An elevator has at least 1 moving stage, its carriage, but was given " +
                                std::to_string(stages) + ".");
  }
  if (stageHeights.size() != static_cast<size_t>(stages)) {
    throw std::invalid_argument("Give one height for each of the " + std::to_string(stages) +
                                " moving stages, but " + std::to_string(stageHeights.size()) +
                                " were given.");
  }
  for (int stage = 0; stage < stages; stage++) {
    if (!(stageHeights[stage].value() > 0)) {
      throw std::invalid_argument("Stage " + std::to_string(stage + 1) + "'s height must be positive.");
    }
  }
  m_stages = stages;
  m_stageHeights = std::move(stageHeights);
  return *this;
}

std::string ElevatorConfig::GetTelemetryName() const { return m_telemetryName; }

std::optional<wpi::units::meter_t> ElevatorConfig::GetMinHeight() const { return m_minHeight; }

std::optional<wpi::units::meter_t> ElevatorConfig::GetMaxHeight() const { return m_maxHeight; }

std::optional<wpi::units::kilogram_t> ElevatorConfig::GetCarriageMass() const {
  return m_carriageMass;
}

bool ElevatorConfig::IsHorizontal() const { return m_isHorizontal; }

wpi::util::Color8Bit ElevatorConfig::GetSimColor() const { return m_simColor; }

wpi::units::degree_t ElevatorConfig::GetAngle() const { return m_angle; }

std::optional<int> ElevatorConfig::GetStages() const { return m_stages; }

const std::vector<wpi::units::meter_t>& ElevatorConfig::GetStageHeights() const {
  return m_stageHeights;
}

}  // namespace yams::mechanisms::config
