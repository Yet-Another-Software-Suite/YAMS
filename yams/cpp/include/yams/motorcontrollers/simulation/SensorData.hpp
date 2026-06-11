// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

#pragma once

#include <hal/SimDevice.h>
#include <hal/Value.h>

#include <functional>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace yams::motorcontrollers::simulation {

/**
 * Encapsulates a single named field of a simulated sensor.
 *
 * On a real robot the value is read from the supplied callable.  In simulation
 * the value is read from (and can be written to) a Glass SimValue, and
 * trigger-based overrides can inject specific values at controlled times.
 */
class SensorData {
 public:
  enum class HALValueType { kBoolean, kDouble, kEnum, kInt, kLong };

  // ---- Constructors -----------------------------------------------------------

  SensorData(std::string name, std::function<HAL_Value()> supplier, HAL_Value defaultValue,
             HALValueType type);
  SensorData(std::string name, std::function<double()> supplier, double defaultVal);
  SensorData(std::string name, std::function<int()> supplier, int defaultVal);
  SensorData(std::string name, std::function<bool()> supplier, bool defaultVal);
  SensorData(std::string name, std::function<int64_t()> supplier, int64_t defaultVal);

  // ---- Static converters ------------------------------------------------------

  static HAL_Value Convert(double value);
  static HAL_Value Convert(int value);
  static HAL_Value Convert(bool value);
  static HAL_Value Convert(int64_t value);

  static std::function<HAL_Value()> Convert(std::function<double()> supplier);
  static std::function<HAL_Value()> Convert(std::function<int()> supplier);
  static std::function<HAL_Value()> Convert(std::function<bool()> supplier);
  static std::function<HAL_Value()> Convert(std::function<int64_t()> supplier);

  // ---- Typed getters ----------------------------------------------------------

  double GetAsDouble() const;
  int GetAsInt() const;
  bool GetAsBoolean() const;
  int64_t GetAsLong() const;

  /** Get the current sensor value (real supplier on robot; Glass/trigger in sim). */
  HAL_Value GetValue() const;

  HAL_Value GetDefault() const;
  std::string GetName() const;
  HALValueType GetType() const;

  // ---- Setters ----------------------------------------------------------------

  void Set(HAL_Value val);
  void Set(double val);
  void Set(int val);
  void Set(int64_t val);
  void Set(bool val);

  // ---- Simulation triggers ----------------------------------------------------

  /** Override the sensor value whenever @p trigger returns true. */
  void AddSimTrigger(HAL_Value value, std::function<bool()> trigger);

  /**
   * Register this field with a SimDevice so it appears in Glass.
   * Called by Sensor during construction (simulation only).
   */
  HAL_SimValueHandle CreateValue(HAL_SimDeviceHandle device, int32_t direction);

 private:
  std::string m_name;
  std::function<HAL_Value()> m_supplier;
  HALValueType m_type;
  HAL_Value m_defaultValue;
  std::optional<std::vector<std::pair<std::function<bool()>, HAL_Value>>> m_triggerValues;
  HAL_SimValueHandle m_glassValue{HAL_kInvalidHandle};
  std::optional<HAL_Value> m_prev;
};

}  // namespace yams::motorcontrollers::simulation
