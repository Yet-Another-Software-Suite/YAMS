// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

#include "yams/mechanisms/config/SimSensorConfig.hpp"

#include <frc/DriverStation.h>
#include <units/time.h>

#include <functional>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "yams/motorcontrollers/simulation/Sensor.hpp"
#include "yams/motorcontrollers/simulation/SensorData.hpp"

namespace yams::mechanisms::config {

using SensorData = yams::motorcontrollers::simulation::SensorData;

SimSensorConfig::SimSensorConfig(std::string name) : m_name{std::move(name)} {}

// ---- Field registration -----------------------------------------------------

SimSensorConfig& SimSensorConfig::WithField(const std::string& name,
                                            std::function<double()> supplier, double defaultVal) {
  m_data.emplace_back(name, std::move(supplier), defaultVal);
  return *this;
}

SimSensorConfig& SimSensorConfig::WithField(const std::string& name, std::function<int()> supplier,
                                            int defaultVal) {
  m_data.emplace_back(name, std::move(supplier), defaultVal);
  return *this;
}

SimSensorConfig& SimSensorConfig::WithField(const std::string& name, std::function<bool()> supplier,
                                            bool defaultVal) {
  m_data.emplace_back(name, std::move(supplier), defaultVal);
  return *this;
}

SimSensorConfig& SimSensorConfig::WithField(const std::string& name,
                                            std::function<int64_t()> supplier, int64_t defaultVal) {
  m_data.emplace_back(name, std::move(supplier), defaultVal);
  return *this;
}

// ---- Match-time simulated value injection -----------------------------------

namespace {
std::function<bool()> MatchTimeTrigger(units::second_t start, units::second_t end) {
  return [start, end] {
    auto t = frc::DriverStation::GetMatchTime();
    return t >= start && t <= end;
  };
}
}  // namespace

SimSensorConfig& SimSensorConfig::WithSimulatedValue(const std::string& fieldName,
                                                     units::second_t start, units::second_t end,
                                                     double value) {
  return WithSimulatedValue(fieldName, MatchTimeTrigger(start, end), value);
}

SimSensorConfig& SimSensorConfig::WithSimulatedValue(const std::string& fieldName,
                                                     units::second_t start, units::second_t end,
                                                     int value) {
  return WithSimulatedValue(fieldName, MatchTimeTrigger(start, end), value);
}

SimSensorConfig& SimSensorConfig::WithSimulatedValue(const std::string& fieldName,
                                                     units::second_t start, units::second_t end,
                                                     int64_t value) {
  return WithSimulatedValue(fieldName, MatchTimeTrigger(start, end), value);
}

SimSensorConfig& SimSensorConfig::WithSimulatedValue(const std::string& fieldName,
                                                     units::second_t start, units::second_t end,
                                                     bool value) {
  return WithSimulatedValue(fieldName, MatchTimeTrigger(start, end), value);
}

// ---- Trigger-based simulated value injection --------------------------------

SimSensorConfig& SimSensorConfig::WithSimulatedValue(const std::string& fieldName,
                                                     std::function<bool()> trigger, double value) {
  for (auto& field : m_data)
    if (field.GetName() == fieldName) field.AddSimTrigger(SensorData::Convert(value), trigger);
  return *this;
}

SimSensorConfig& SimSensorConfig::WithSimulatedValue(const std::string& fieldName,
                                                     std::function<bool()> trigger, int value) {
  for (auto& field : m_data)
    if (field.GetName() == fieldName) field.AddSimTrigger(SensorData::Convert(value), trigger);
  return *this;
}

SimSensorConfig& SimSensorConfig::WithSimulatedValue(const std::string& fieldName,
                                                     std::function<bool()> trigger, int64_t value) {
  for (auto& field : m_data)
    if (field.GetName() == fieldName) field.AddSimTrigger(SensorData::Convert(value), trigger);
  return *this;
}

SimSensorConfig& SimSensorConfig::WithSimulatedValue(const std::string& fieldName,
                                                     std::function<bool()> trigger, bool value) {
  for (auto& field : m_data)
    if (field.GetName() == fieldName) field.AddSimTrigger(SensorData::Convert(value), trigger);
  return *this;
}

// ---- Accessors --------------------------------------------------------------

yams::motorcontrollers::simulation::Sensor& SimSensorConfig::GetSensor() {
  if (!m_sensor)
    m_sensor = std::make_unique<yams::motorcontrollers::simulation::Sensor>(m_name, m_data);
  return *m_sensor;
}

const std::string& SimSensorConfig::GetName() const { return m_name; }

std::vector<SensorData> SimSensorConfig::GetFields() const { return m_data; }

}  // namespace yams::mechanisms::config
