// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

#include "yams/motorcontrollers/simulation/Sensor.hpp"

#include <frc/RobotBase.h>
#include <hal/SimDevice.h>

#include <functional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "yams/mechanisms/config/SimSensorConfig.hpp"

namespace yams::motorcontrollers::simulation {

Sensor::Sensor(std::string sensorName, std::vector<SensorData> fields)
    : m_sensorName{std::move(sensorName)} {
  for (auto& field : fields) m_simData.emplace(field.GetName(), std::move(field));

  if (frc::RobotBase::IsSimulation()) {
    auto handle = HAL_CreateSimDevice(("Sensor[" + m_sensorName + "]").c_str());
    m_simDevice = handle;
    for (auto& [name, field] : m_simData) {
      field.CreateValue(handle, HAL_SimValueBidir);
    }
  }
}

Sensor::Sensor(const yams::mechanisms::config::SimSensorConfig& cfg)
    : Sensor{cfg.GetName(), cfg.GetFields()} {}

SensorData& Sensor::GetField(const std::string& name) {
  auto it = m_simData.find(name);
  if (it == m_simData.end())
    throw std::invalid_argument("Sensor[" + m_sensorName + "." + name + "] does not exist!");
  return it->second;
}

const SensorData& Sensor::GetField(const std::string& name) const {
  auto it = m_simData.find(name);
  if (it == m_simData.end())
    throw std::invalid_argument("Sensor[" + m_sensorName + "." + name + "] does not exist!");
  return it->second;
}

double Sensor::GetAsDouble(const std::string& name) const { return GetField(name).GetAsDouble(); }
int Sensor::GetAsInt(const std::string& name) const { return GetField(name).GetAsInt(); }
bool Sensor::GetAsBoolean(const std::string& name) const { return GetField(name).GetAsBoolean(); }
int64_t Sensor::GetAsLong(const std::string& name) const { return GetField(name).GetAsLong(); }

HAL_SimDeviceHandle Sensor::GetDevice() const { return m_simDevice.value_or(HAL_kInvalidHandle); }

void Sensor::AddSimTrigger(const std::string& field, HAL_Value value,
                           std::function<bool()> trigger) {
  GetField(field).AddSimTrigger(value, std::move(trigger));
}

}  // namespace yams::motorcontrollers::simulation
