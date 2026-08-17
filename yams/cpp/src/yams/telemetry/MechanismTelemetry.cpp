// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

#include "yams/telemetry/MechanismTelemetry.hpp"

#include <frc/Timer.h>
#include <networktables/NetworkTableInstance.h>
#include <wpi/json.h>

#include <memory>
#include <string>

#include "yams/motorcontrollers/SmartMotorController.hpp"

namespace yams::telemetry {

void MechanismTelemetry::SetupLoopTime() {
  auto topic = m_networkTable->GetDoubleTopic("loopTime");
  topic.SetProperties(wpi::json{{"unit", "second"}});
  m_loopTimePublisher = topic.Publish();
}

void MechanismTelemetry::SetupTelemetry(const std::string& mechanismName,
                                        motorcontrollers::SmartMotorController& motorController) {
  auto inst = nt::NetworkTableInstance::GetDefault();
  m_tuningNetworkTable = inst.GetTable("Tuning")->GetSubTable(mechanismName);
  m_networkTable = inst.GetTable("Mechanisms")->GetSubTable(mechanismName);
  motorController.SetupTelemetry(m_networkTable, m_tuningNetworkTable);
  SetupLoopTime();
}

void MechanismTelemetry::SetupTelemetry(const std::string& mechanismName) {
  auto inst = nt::NetworkTableInstance::GetDefault();
  m_tuningNetworkTable = inst.GetTable("Tuning")->GetSubTable(mechanismName);
  m_networkTable = inst.GetTable("Mechanisms")->GetSubTable(mechanismName);
  SetupLoopTime();
}

void MechanismTelemetry::SetupTelemetry(const std::string& mechanismName,
                                        const std::string& dataLogName) {
  SetupTelemetry(mechanismName);
  m_dataLogName = dataLogName;
}

void MechanismTelemetry::AddMotorController(
    const std::string& subTableName, motorcontrollers::SmartMotorController& motorController) {
  motorController.SetupTelemetry(m_networkTable->GetSubTable(subTableName),
                                 m_tuningNetworkTable->GetSubTable(subTableName));
}

std::function<void(double)> MechanismTelemetry::PublishDouble(const std::string& key,
                                                               const std::string& unit) {
  auto topic = m_networkTable->GetDoubleTopic(key);
  if (!unit.empty()) {
    topic.SetProperties(wpi::json{{"units", unit}});
  }
  auto publisher = std::make_shared<nt::DoublePublisher>(topic.Publish());
  std::shared_ptr<wpi::log::DoubleLogEntry> logEntry;
  if (m_dataLogName) {
    logEntry =
        std::make_shared<wpi::log::DoubleLogEntry>(frc::DataLogManager::GetLog(), *m_dataLogName + "/" + key);
  }
  return [publisher, logEntry](double value) {
    publisher->Set(value);
    if (logEntry) logEntry->Append(value);
  };
}

void MechanismTelemetry::UpdateLoopTime() {
  if (!m_loopTimePublisher) return;
  double now = frc::Timer::GetFPGATimestamp().value();
  if (m_prevTimestamp != 0.0) {
    m_loopTimePublisher->Set(now - m_prevTimestamp);
  }
  m_prevTimestamp = now;
}

std::shared_ptr<nt::NetworkTable> MechanismTelemetry::GetDataTable() const {
  return m_networkTable;
}

std::shared_ptr<nt::NetworkTable> MechanismTelemetry::GetTuningTable() const {
  return m_tuningNetworkTable;
}

}  // namespace yams::telemetry
