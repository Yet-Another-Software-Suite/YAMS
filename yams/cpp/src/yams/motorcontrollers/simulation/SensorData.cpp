// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

#include "yams/motorcontrollers/simulation/SensorData.hpp"

#include <frc/RobotBase.h>
#include <hal/SimDevice.h>
#include <hal/Value.h>

#include <functional>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace yams::motorcontrollers::simulation {

// ---- Constructors -----------------------------------------------------------

SensorData::SensorData(std::string name, std::function<HAL_Value()> supplier,
                       HAL_Value defaultValue, HALValueType type)
    : m_name{std::move(name)},
      m_supplier{std::move(supplier)},
      m_type{type},
      m_defaultValue{defaultValue} {}

SensorData::SensorData(std::string name, std::function<double()> supplier, double defaultVal)
    : SensorData{std::move(name), Convert(std::move(supplier)), Convert(defaultVal),
                 HALValueType::kDouble} {}

SensorData::SensorData(std::string name, std::function<int()> supplier, int defaultVal)
    : SensorData{std::move(name), Convert(std::move(supplier)), Convert(defaultVal),
                 HALValueType::kInt} {}

SensorData::SensorData(std::string name, std::function<bool()> supplier, bool defaultVal)
    : SensorData{std::move(name), Convert(std::move(supplier)), Convert(defaultVal),
                 HALValueType::kBoolean} {}

SensorData::SensorData(std::string name, std::function<int64_t()> supplier, int64_t defaultVal)
    : SensorData{std::move(name), Convert(std::move(supplier)), Convert(defaultVal),
                 HALValueType::kLong} {}

// ---- Static converters ------------------------------------------------------

HAL_Value SensorData::Convert(double value) { return HAL_MakeDouble(value); }
HAL_Value SensorData::Convert(int value) { return HAL_MakeInt(value); }
HAL_Value SensorData::Convert(bool value) { return HAL_MakeBoolean(static_cast<HAL_Bool>(value)); }
HAL_Value SensorData::Convert(int64_t value) { return HAL_MakeLong(value); }

std::function<HAL_Value()> SensorData::Convert(std::function<double()> supplier) {
  return [s = std::move(supplier)] { return Convert(s()); };
}
std::function<HAL_Value()> SensorData::Convert(std::function<int()> supplier) {
  return [s = std::move(supplier)] { return Convert(s()); };
}
std::function<HAL_Value()> SensorData::Convert(std::function<bool()> supplier) {
  return [s = std::move(supplier)] { return Convert(s()); };
}
std::function<HAL_Value()> SensorData::Convert(std::function<int64_t()> supplier) {
  return [s = std::move(supplier)] { return Convert(s()); };
}

// ---- Typed getters ----------------------------------------------------------

double SensorData::GetAsDouble() const {
  if (m_type != HALValueType::kDouble)
    throw std::logic_error(m_name + " HALValue is not a double!");
  return GetValue().data.v_double;
}

int SensorData::GetAsInt() const {
  if (m_type != HALValueType::kInt) throw std::logic_error(m_name + " HALValue is not an int!");
  return static_cast<int>(GetValue().data.v_int);
}

bool SensorData::GetAsBoolean() const {
  if (m_type != HALValueType::kBoolean)
    throw std::logic_error(m_name + " HALValue is not a boolean!");
  return GetValue().data.v_boolean != 0;
}

int64_t SensorData::GetAsLong() const {
  if (m_type != HALValueType::kLong) throw std::logic_error(m_name + " HALValue is not a long!");
  return GetValue().data.v_long;
}

HAL_Value SensorData::GetValue() const {
  if (frc::RobotBase::IsReal()) return m_supplier();

  if (m_triggerValues.has_value()) {
    for (auto& [trigger, value] : *m_triggerValues) {
      if (trigger()) {
        const_cast<SensorData*>(this)->Set(value);
        return value;
      }
    }
  }

  if (m_prev.has_value()) {
    const_cast<SensorData*>(this)->Set(*m_prev);
    const_cast<SensorData*>(this)->m_prev.reset();
  }

  if (m_glassValue != HAL_kInvalidHandle) return HAL_GetSimValue(m_glassValue);
  return m_supplier();
}

HAL_Value SensorData::GetDefault() const { return m_defaultValue; }
std::string SensorData::GetName() const { return m_name; }
SensorData::HALValueType SensorData::GetType() const { return m_type; }

// ---- Setters ----------------------------------------------------------------

void SensorData::Set(HAL_Value val) {
  if (!m_prev.has_value() && m_glassValue != HAL_kInvalidHandle)
    m_prev = HAL_GetSimValue(m_glassValue);
  if (m_glassValue != HAL_kInvalidHandle) HAL_SetSimValue(m_glassValue, val);
}

void SensorData::Set(double val) { Set(Convert(val)); }
void SensorData::Set(int val) { Set(Convert(val)); }
void SensorData::Set(int64_t val) { Set(Convert(val)); }
void SensorData::Set(bool val) { Set(Convert(val)); }

// ---- Simulation triggers ----------------------------------------------------

void SensorData::AddSimTrigger(HAL_Value value, std::function<bool()> trigger) {
  if (!m_triggerValues.has_value()) m_triggerValues.emplace();
  m_triggerValues->emplace_back(std::move(trigger), value);
}

HAL_SimValueHandle SensorData::CreateValue(HAL_SimDeviceHandle device, int32_t direction) {
  auto handle = HAL_CreateSimValue(device, m_name.c_str(), direction, m_defaultValue);
  if (direction == HAL_SimValueBidir || direction == HAL_SimValueInput) m_glassValue = handle;
  return handle;
}

}  // namespace yams::motorcontrollers::simulation
