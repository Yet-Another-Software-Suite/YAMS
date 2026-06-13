// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

#include "yams/telemetry/SmartMotorControllerTelemetry.hpp"

#include <frc/DataLogManager.h>
#include <units/angle.h>
#include <units/angular_acceleration.h>
#include <units/angular_velocity.h>
#include <units/current.h>
#include <units/length.h>
#include <units/time.h>
#include <units/velocity.h>
#include <units/voltage.h>
#include <wpi/json.h>

#include <memory>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>

#include "yams/exceptions.hpp"
#include "yams/motorcontrollers/SmartMotorController.hpp"
#include "yams/motorcontrollers/SmartMotorControllerConfig.hpp"

namespace yams::telemetry {

using namespace yams::motorcontrollers;

// ============================================================================
// DoubleTelemetry
// ============================================================================

DoubleTelemetry::DoubleTelemetry(std::string key, double defaultVal, DoubleTelemetryField field,
                                 bool tunable, std::string unit)
    : m_field{field},
      m_key{std::move(key)},
      m_tunable{tunable},
      m_unit{std::move(unit)},
      m_defaultValue{defaultVal},
      m_cachedValue{defaultVal} {}

void DoubleTelemetry::SetDefaultValue(double value) { m_cachedValue = m_defaultValue = value; }

void DoubleTelemetry::SetupNetworkTables(std::shared_ptr<nt::NetworkTable> dataTable,
                                         std::shared_ptr<nt::NetworkTable> tuningTable) {
  m_dataTable = dataTable;
  m_tuningTable = tuningTable;
  if (!m_enabled) return;

  if (tuningTable && m_tunable) {
    auto topic = tuningTable->GetDoubleTopic(m_key);
    if (m_unit != "none") {
      topic.SetProperties(wpi::json{{"units", m_unit}});
    }
    m_subPublisher = topic.Publish();
    m_subPublisher->SetDefault(m_defaultValue);
    m_subscriber = topic.Subscribe(m_defaultValue);
  } else if (dataTable) {
    auto topic = dataTable->GetDoubleTopic(m_key);
    if (m_unit != "none") {
      topic.SetProperties(wpi::json{{"units", m_unit}});
    }
    m_publisher = topic.Publish();
    m_publisher->SetDefault(m_defaultValue);
  }
}

void DoubleTelemetry::SetupNetworkTable(std::shared_ptr<nt::NetworkTable> dataTable) {
  SetupNetworkTables(dataTable, nullptr);
}

void DoubleTelemetry::SetupDataLog(const std::string& prefix) {
  if (m_tunable) return;
  std::string path = prefix;
  if (!path.empty() && path.back() != '/') path += '/';
  path += m_unit + '/' + m_key;
  m_dataLogEntry = wpi::log::DoubleLogEntry{frc::DataLogManager::GetLog(), path};
}

void DoubleTelemetry::TransformUnit(const SmartMotorControllerConfig& cfg) {
  bool linear = cfg.GetLinearClosedLoopControllerUse();
  if (m_unit == "tunable_position") {
    m_unit = linear ? "meter" : "degrees";
  } else if (m_unit == "position") {
    m_unit = linear ? "meter" : "rotations";
  } else if (m_unit == "tunable_velocity") {
    m_unit = linear ? "meter_per_second" : "rotations_per_minute";
  } else if (m_unit == "velocity") {
    m_unit = linear ? "meter_per_second" : "rotation_per_second";
  } else if (m_unit == "tunable_acceleration") {
    m_unit = linear ? "meter_per_second_per_second" : "rotations_per_minute_per_second";
  } else if (m_unit == "acceleration") {
    m_unit = linear ? "meter_per_second_per_second" : "rotation_per_second_per_second";
  }
}

bool DoubleTelemetry::Set(double value) {
  if (!m_enabled) return false;
  if (m_dataLogEntry) {
    m_dataLogEntry->Append(value);
  }
  if (m_subscriber) {
    double tuningValue = m_subscriber->Get(m_defaultValue);
    if (tuningValue != value) return false;
  }
  if (m_publisher) {
    m_publisher->Set(value);
  }
  return true;
}

double DoubleTelemetry::Get() const {
  if (!m_enabled) return m_defaultValue;
  if (m_subscriber) {
    return m_subscriber->Get(m_defaultValue);
  }
  throw std::runtime_error("Tuning table not configured for " + m_key + "!");
}

bool DoubleTelemetry::IsTunable() {
  if (m_subscriber && m_tunable && m_enabled) {
    double current = m_subscriber->Get(m_defaultValue);
    if (current != m_cachedValue) {
      m_cachedValue = current;
      return true;
    }
  }
  return false;
}

void DoubleTelemetry::Close() {
  m_subscriber.reset();
  m_subPublisher.reset();
  m_publisher.reset();
  if (m_dataTable) {
    m_dataTable->GetEntry(m_key).Unpublish();
  }
  if (m_tuningTable) {
    m_tuningTable->GetEntry(m_key).Unpublish();
  }
}

// ============================================================================
// BooleanTelemetry
// ============================================================================

BooleanTelemetry::BooleanTelemetry(std::string key, bool defaultVal, BooleanTelemetryField field,
                                   bool tunable)
    : m_field{field},
      m_key{std::move(key)},
      m_tunable{tunable},
      m_defaultValue{defaultVal},
      m_cachedValue{defaultVal} {}

void BooleanTelemetry::SetDefaultValue(bool value) {
  m_defaultValue = value;
  m_cachedValue = value;
}

void BooleanTelemetry::SetupNetworkTables(std::shared_ptr<nt::NetworkTable> dataTable,
                                          std::shared_ptr<nt::NetworkTable> tuningTable) {
  m_dataTable = dataTable;
  m_tuningTable = tuningTable;
  if (tuningTable && m_tunable) {
    auto topic = tuningTable->GetBooleanTopic(m_key);
    m_pubSub = topic.Publish();
    m_pubSub->SetDefault(m_defaultValue);
    m_subscriber = topic.Subscribe(m_defaultValue);
  } else if (dataTable) {
    auto topic = dataTable->GetBooleanTopic(m_key);
    m_publisher = topic.Publish();
    m_publisher->SetDefault(m_defaultValue);
  }
}

void BooleanTelemetry::SetupNetworkTable(std::shared_ptr<nt::NetworkTable> dataTable) {
  SetupNetworkTables(dataTable, nullptr);
}

void BooleanTelemetry::SetupDataLog(const std::string& prefix) {
  if (m_tunable) return;
  std::string path = prefix;
  if (!path.empty() && path.back() != '/') path += '/';
  path += m_key;
  m_dataLogEntry = wpi::log::BooleanLogEntry{frc::DataLogManager::GetLog(), path};
}

bool BooleanTelemetry::Set(bool value) {
  if (!m_enabled) return false;
  if (m_dataLogEntry) {
    m_dataLogEntry->Append(value);
  }
  if (m_subscriber) {
    bool tuningValue = m_subscriber->Get(m_defaultValue);
    if (tuningValue != value) return false;
  }
  if (m_publisher) {
    m_publisher->Set(value);
  }
  return true;
}

bool BooleanTelemetry::Get() const {
  if (m_subscriber) {
    return m_subscriber->Get(m_defaultValue);
  }
  throw std::runtime_error("Tuning table not configured for " + m_key + "!");
}

bool BooleanTelemetry::IsTunable() {
  if (m_subscriber && m_tunable && m_enabled) {
    bool current = m_subscriber->Get(m_defaultValue);
    if (current != m_cachedValue) {
      m_cachedValue = current;
      return true;
    }
  }
  return false;
}

void BooleanTelemetry::Close() {
  m_subscriber.reset();
  m_pubSub.reset();
  m_publisher.reset();
  if (m_dataTable) {
    m_dataTable->GetEntry(m_key).Unpublish();
  }
  if (m_tuningTable) {
    m_tuningTable->GetEntry(m_key).Unpublish();
  }
}

// ============================================================================
// SmartMotorControllerTelemetry
// ============================================================================

void SmartMotorControllerTelemetry::SetupTelemetry(
    SmartMotorController& smc, std::shared_ptr<nt::NetworkTable> publishTable,
    std::shared_ptr<nt::NetworkTable> tuningTable,
    std::unordered_map<DoubleTelemetryField, DoubleTelemetry>& doubleFields,
    std::unordered_map<BooleanTelemetryField, BooleanTelemetry>& boolFields, bool nt4Enabled,
    std::optional<std::string> dataLogName) {
  if (m_dataTable && m_dataTable == publishTable) return;  // already set up

  m_dataTable = publishTable;
  m_tuningTable = tuningTable;
  m_doubleFields = &doubleFields;
  m_boolFields = &boolFields;

  SmartMotorControllerConfig& cfg = smc.GetConfig();

  for (auto& [field, dt] : doubleFields) {
    if (nt4Enabled) {
      dt.TransformUnit(cfg);
      dt.SetupNetworkTables(publishTable, tuningTable);
    }
    if (dataLogName) {
      dt.TransformUnit(cfg);
      dt.SetupDataLog(*dataLogName);
    }
  }
  for (auto& [field, bt] : boolFields) {
    if (nt4Enabled) {
      bt.SetupNetworkTables(publishTable, tuningTable);
    }
    if (dataLogName) {
      bt.SetupDataLog(*dataLogName);
    }
  }
}

void SmartMotorControllerTelemetry::Publish(SmartMotorController& smc) {
  if (!m_doubleFields || !m_boolFields) return;

  SmartMotorControllerConfig& cfg = smc.GetConfig();

  // Boolean fields
  for (auto& [field, bt] : *m_boolFields) {
    if (!bt.IsEnabled()) continue;
    switch (bt.GetField()) {
      case BooleanTelemetryField::MechanismUpperLimit:
        if (auto lim = cfg.GetMechanismUpperLimit()) bt.Set(smc.GetMechanismPosition() >= *lim);
        break;
      case BooleanTelemetryField::MechanismLowerLimit:
        if (auto lim = cfg.GetMechanismLowerLimit()) bt.Set(smc.GetMechanismPosition() <= *lim);
        break;
      case BooleanTelemetryField::TemperatureLimit:
        if (auto cutoff = cfg.GetTemperatureCutoff()) bt.Set(smc.GetTemperature() >= *cutoff);
        break;
      case BooleanTelemetryField::VelocityControl:
        bt.Set(smc.GetMechanismSetpointVelocity().has_value());
        break;
      case BooleanTelemetryField::ElevatorFeedForward:
        bt.Set(cfg.GetElevatorFeedforward(smc.GetClosedLoopControllerSlot()).has_value());
        break;
      case BooleanTelemetryField::ArmFeedForward:
        bt.Set(cfg.GetArmFeedforward(smc.GetClosedLoopControllerSlot()).has_value());
        break;
      case BooleanTelemetryField::SimpleMotorFeedForward:
        bt.Set(cfg.GetSimpleFeedforward(smc.GetClosedLoopControllerSlot()).has_value());
        break;
      case BooleanTelemetryField::MotionProfile:
        bt.Set(cfg.HasExponentialProfile() || cfg.HasTrapezoidProfile());
        break;
      default:
        break;
    }
  }

  // Double fields
  for (auto& [field, dt] : *m_doubleFields) {
    if (!dt.IsEnabled()) continue;
    bool linear = cfg.GetLinearClosedLoopControllerUse();
    switch (dt.GetField()) {
      case DoubleTelemetryField::SetpointPosition:
        if (auto sp = smc.GetMechanismPositionSetpoint()) {
          if (linear)
            dt.Set(cfg.ConvertFromMechanism(*sp).value());
          else
            dt.Set(sp->value());  // turn_t → rotations
        }
        break;
      case DoubleTelemetryField::SetpointVelocity:
        if (auto sv = smc.GetMechanismSetpointVelocity()) {
          if (linear)
            dt.Set(cfg.ConvertFromMechanism(*sv).value());
          else
            dt.Set(sv->value());  // turns_per_second_t → rotations/s
        }
        break;
      case DoubleTelemetryField::OutputVoltage:
        dt.Set(smc.GetVoltage().value());
        break;
      case DoubleTelemetryField::StatorCurrent:
        dt.Set(smc.GetStatorCurrent().value());
        break;
      case DoubleTelemetryField::SupplyCurrent:
        if (auto sc = smc.GetSupplyCurrent()) dt.Set(sc->value());
        break;
      case DoubleTelemetryField::MotorTemperature:
        // Celsius → Fahrenheit
        dt.Set(smc.GetTemperature().value() * 9.0 / 5.0 + 32.0);
        break;
      case DoubleTelemetryField::MeasurementPosition:
        if (cfg.GetMechanismCircumference()) dt.Set(smc.GetMeasurementPosition().value());
        break;
      case DoubleTelemetryField::MeasurementVelocity:
        if (cfg.GetMechanismCircumference()) dt.Set(smc.GetMeasurementVelocity().value());
        break;
      case DoubleTelemetryField::MeasurementAcceleration:
        if (cfg.GetMechanismCircumference()) dt.Set(smc.GetMeasurementAcceleration().value());
        break;
      case DoubleTelemetryField::MechanismPosition:
        dt.Set(smc.GetMechanismPosition().value());
        break;
      case DoubleTelemetryField::MechanismVelocity:
        dt.Set(smc.GetMechanismVelocity().value());
        break;
      case DoubleTelemetryField::MechanismAcceleration:
        dt.Set(smc.GetMechanismAcceleration().value());
        break;
      case DoubleTelemetryField::RotorPosition:
        dt.Set(smc.GetRotorPosition().value());
        break;
      case DoubleTelemetryField::RotorVelocity:
        dt.Set(smc.GetRotorVelocity().value());
        break;
      case DoubleTelemetryField::ExternalEncoderPosition:
        dt.Set(smc.GetExternalEncoderPosition().value_or(units::degree_t{0}).value() / 360.0);
        break;
      case DoubleTelemetryField::ExternalEncoderVelocity:
        dt.Set(smc.GetExternalEncoderVelocity().value_or(units::degrees_per_second_t{0}).value() /
               360.0);
        break;
      case DoubleTelemetryField::ActiveClosedLoopControllerSlot:
        dt.Set(static_cast<double>(static_cast<int>(smc.GetClosedLoopControllerSlot())));
        break;
      default:
        break;
    }
  }
}

void SmartMotorControllerTelemetry::ApplyTuningValues(SmartMotorController& smc) {
  if (!m_doubleFields || !m_boolFields) return;

  SmartMotorControllerConfig& cfg = smc.GetConfig();
  if (cfg.GetMotorControllerMode() != SmartMotorControllerConfig::ControlMode::CLOSED_LOOP) {
    throw yams::exceptions::SmartMotorControllerConfigurationException(
        "Live tuning does not work in OPEN_LOOP", "Cannot apply setpoints for Live Tuning.",
        ".withControlMode(ControlMode.CLOSED_LOOP)");
  }

  // Boolean tuning
  for (auto& [field, bt] : *m_boolFields) {
    if (!bt.IsTunable()) continue;
    switch (bt.GetField()) {
      case BooleanTelemetryField::MotorInversion:
        smc.SetMotorInverted(bt.Get());
        break;
      case BooleanTelemetryField::EncoderInversion:
        smc.SetEncoderInverted(bt.Get());
        break;
      default:
        break;
    }
  }

  bool linear = cfg.GetLinearClosedLoopControllerUse();

  // Double tuning
  for (auto& [field, dt] : *m_doubleFields) {
    if (!dt.IsTunable()) continue;
    switch (dt.GetField()) {
      case DoubleTelemetryField::TunableClosedLoopControllerSlot: {
        int idx = static_cast<int>(dt.Get());
        if (idx >= 0 && idx <= 3) {
          auto slot = static_cast<SmartMotorControllerConfig::ClosedLoopControllerSlot>(idx);
          if (smc.GetClosedLoopControllerSlot() != slot) smc.SetClosedLoopSlot(slot);
        }
        break;
      }
      case DoubleTelemetryField::TunableSetpointPosition:
        if (linear)
          smc.SetPosition(units::meter_t{dt.Get()});
        else
          smc.SetPosition(units::degree_t{dt.Get()});
        break;
      case DoubleTelemetryField::TunableSetpointVelocity:
        if (dt.Get() == 0.0) break;
        if (linear)
          smc.SetVelocity(units::meters_per_second_t{dt.Get()});
        else
          smc.SetVelocity(units::degrees_per_second_t{dt.Get() * 6.0});  // RPM → deg/s
        break;
      case DoubleTelemetryField::kP:
        smc.SetKp(dt.Get());
        break;
      case DoubleTelemetryField::kI:
        smc.SetKi(dt.Get());
        break;
      case DoubleTelemetryField::kD:
        smc.SetKd(dt.Get());
        break;
      case DoubleTelemetryField::kS:
        smc.SetKs(dt.Get());
        break;
      case DoubleTelemetryField::kV:
        smc.SetKv(dt.Get());
        break;
      case DoubleTelemetryField::kA:
        smc.SetKa(dt.Get());
        break;
      case DoubleTelemetryField::kG:
        smc.SetKg(dt.Get());
        break;
      case DoubleTelemetryField::ClosedloopRampRate:
        smc.SetClosedLoopRampRate(units::second_t{dt.Get()});
        break;
      case DoubleTelemetryField::OpenloopRampRate:
        smc.SetOpenLoopRampRate(units::second_t{dt.Get()});
        break;
      case DoubleTelemetryField::SupplyCurrentLimit:
        smc.SetSupplyCurrentLimit(units::ampere_t{dt.Get()});
        break;
      case DoubleTelemetryField::StatorCurrentLimit:
        smc.SetStatorCurrentLimit(units::ampere_t{dt.Get()});
        break;
      case DoubleTelemetryField::MeasurementUpperLimit:
        smc.SetMeasurementUpperLimit(units::meter_t{dt.Get()});
        break;
      case DoubleTelemetryField::MeasurementLowerLimit:
        smc.SetMeasurementLowerLimit(units::meter_t{dt.Get()});
        break;
      case DoubleTelemetryField::MechanismUpperLimit:
        smc.SetMechanismUpperLimit(units::degree_t{dt.Get()});
        break;
      case DoubleTelemetryField::MechanismLowerLimit:
        smc.SetMechanismLowerLimit(units::degree_t{dt.Get()});
        break;
      case DoubleTelemetryField::TrapezoidalProfileMaxAcceleration:
        if (linear)
          smc.SetMotionProfileMaxAcceleration(units::meters_per_second_squared_t{dt.Get()});
        else
          smc.SetMotionProfileMaxAcceleration(units::degrees_per_second_squared_t{dt.Get() * 6.0});
        break;
      case DoubleTelemetryField::TrapezoidalProfileMaxVelocity:
        if (linear)
          smc.SetMotionProfileMaxVelocity(units::meters_per_second_t{dt.Get()});
        else
          smc.SetMotionProfileMaxVelocity(units::degrees_per_second_t{dt.Get() * 6.0});
        break;
      case DoubleTelemetryField::TrapezoidalProfileMaxJerk:
        smc.SetMotionProfileMaxJerk(
            units::unit_t<
                units::compound_unit<units::angular_acceleration::degrees_per_second_squared,
                                     units::inverse<units::seconds>>>{dt.Get() * 6.0});
        break;
      case DoubleTelemetryField::ExponentialProfileKA:
        smc.SetExponentialProfile(std::nullopt, dt.Get(), std::nullopt);
        break;
      case DoubleTelemetryField::ExponentialProfileKV:
        smc.SetExponentialProfile(dt.Get(), std::nullopt, std::nullopt);
        break;
      case DoubleTelemetryField::ExponentialProfileMaxInput:
        smc.SetExponentialProfile(std::nullopt, std::nullopt, units::volt_t{dt.Get()});
        break;
      default:
        break;
    }
  }
}

bool SmartMotorControllerTelemetry::TuningEnabled() const {
  if (!m_doubleFields) return false;
  auto itPos = m_doubleFields->find(DoubleTelemetryField::TunableSetpointPosition);
  auto itVel = m_doubleFields->find(DoubleTelemetryField::TunableSetpointVelocity);
  bool posEnabled = (itPos != m_doubleFields->end()) && itPos->second.IsEnabled();
  bool velEnabled = (itVel != m_doubleFields->end()) && itVel->second.IsEnabled();
  return posEnabled || velEnabled;
}

void SmartMotorControllerTelemetry::Close() {
  if (m_doubleFields) {
    for (auto& [field, dt] : *m_doubleFields) dt.Close();
  }
  if (m_boolFields) {
    for (auto& [field, bt] : *m_boolFields) bt.Close();
  }
}

}  // namespace yams::telemetry
