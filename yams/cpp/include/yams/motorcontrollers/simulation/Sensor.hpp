// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

#pragma once

#include <hal/SimDevice.h>
#include <hal/Value.h>

#include <functional>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include "yams/motorcontrollers/simulation/SensorData.hpp"

namespace yams::mechanisms::config { class SimSensorConfig; }

namespace yams::motorcontrollers::simulation {

/**
 * Simulated sensor backed by a HAL SimDevice.
 *
 * All fields report the real supplier value on a physical robot.  In
 * simulation they surface in the Glass UI and can be overridden with
 * trigger-based values for automated testing scenarios.
 */
class Sensor {
 public:
  /**
   * Construct a sensor from a name and an explicit list of fields.
   *
   * @param sensorName  Name shown in the Glass simulation window.
   * @param fields      Fields to register with the SimDevice.
   */
  Sensor(std::string sensorName, std::vector<SensorData> fields);

  /** Construct from a SimSensorConfig builder. */
  explicit Sensor(const yams::mechanisms::config::SimSensorConfig& cfg);

  // ---- Field access -----------------------------------------------------------

  /** Get the SensorData for @p name (throws if not found). */
  SensorData& GetField(const std::string& name);
  const SensorData& GetField(const std::string& name) const;

  double GetAsDouble(const std::string& name) const;
  int GetAsInt(const std::string& name) const;
  bool GetAsBoolean(const std::string& name) const;
  int64_t GetAsLong(const std::string& name) const;

  /** The underlying SimDevice handle (invalid on a real robot). */
  HAL_SimDeviceHandle GetDevice() const;

  // ---- Trigger injection ------------------------------------------------------

  /** Set @p field to @p value whenever @p trigger returns true. */
  void AddSimTrigger(const std::string& field, HAL_Value value,
                     std::function<bool()> trigger);

 private:
  std::string m_sensorName;
  std::optional<HAL_SimDeviceHandle> m_simDevice;
  std::unordered_map<std::string, SensorData> m_simData;
};

}  // namespace yams::motorcontrollers::simulation
