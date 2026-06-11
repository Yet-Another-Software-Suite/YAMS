// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

#pragma once

#include <frc/DriverStation.h>
#include <units/time.h>

#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "yams/motorcontrollers/simulation/SensorData.hpp"

namespace yams::motorcontrollers::simulation {
class Sensor;
}

namespace yams::mechanisms::config {

/**
 * Builder for a simulated {@link yams::motorcontrollers::simulation::Sensor}.
 *
 * Mirrors the Java SensorConfig class.  Add typed fields with WithField(),
 * inject simulated values by match-time window or arbitrary trigger with
 * WithSimulatedValue(), then call GetSensor() to obtain the ready-to-use
 * Sensor instance (lazily constructed on first call).
 */
class SimSensorConfig {
 public:
  using SensorData = yams::motorcontrollers::simulation::SensorData;

  explicit SimSensorConfig(std::string name);

  // ---- Field registration -----------------------------------------------------

  SimSensorConfig& WithField(const std::string& name, std::function<double()> supplier,
                             double defaultVal);
  SimSensorConfig& WithField(const std::string& name, std::function<int()> supplier,
                             int defaultVal);
  SimSensorConfig& WithField(const std::string& name, std::function<bool()> supplier,
                             bool defaultVal);
  SimSensorConfig& WithField(const std::string& name, std::function<int64_t()> supplier,
                             int64_t defaultVal);

  // ---- Match-time simulated value injection -----------------------------------

  SimSensorConfig& WithSimulatedValue(const std::string& fieldName, units::second_t start,
                                      units::second_t end, double value);
  SimSensorConfig& WithSimulatedValue(const std::string& fieldName, units::second_t start,
                                      units::second_t end, int value);
  SimSensorConfig& WithSimulatedValue(const std::string& fieldName, units::second_t start,
                                      units::second_t end, int64_t value);
  SimSensorConfig& WithSimulatedValue(const std::string& fieldName, units::second_t start,
                                      units::second_t end, bool value);

  // ---- Trigger-based simulated value injection --------------------------------

  SimSensorConfig& WithSimulatedValue(const std::string& fieldName, std::function<bool()> trigger,
                                      double value);
  SimSensorConfig& WithSimulatedValue(const std::string& fieldName, std::function<bool()> trigger,
                                      int value);
  SimSensorConfig& WithSimulatedValue(const std::string& fieldName, std::function<bool()> trigger,
                                      int64_t value);
  SimSensorConfig& WithSimulatedValue(const std::string& fieldName, std::function<bool()> trigger,
                                      bool value);

  // ---- Accessors --------------------------------------------------------------

  /** Build (or return cached) Sensor instance. */
  yams::motorcontrollers::simulation::Sensor& GetSensor();

  const std::string& GetName() const;

  /** Direct access to the registered fields (used by Sensor constructor). */
  std::vector<SensorData> GetFields() const;

 private:
  std::string m_name;
  std::vector<SensorData> m_data;
  std::unique_ptr<yams::motorcontrollers::simulation::Sensor> m_sensor;
};

}  // namespace yams::mechanisms::config
