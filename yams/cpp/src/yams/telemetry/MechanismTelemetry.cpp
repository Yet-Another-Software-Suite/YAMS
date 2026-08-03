// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

#include "yams/telemetry/MechanismTelemetry.hpp"

#include <wpi/system/Timer.hpp>
#include <wpi/nt/NetworkTableInstance.hpp>
#include <wpi/util/json.hpp>

#include <memory>
#include <string>

#include "yams/motorcontrollers/SmartMotorController.hpp"

namespace yams::telemetry {

void MechanismTelemetry::SetupLoopTime() {
  auto topic = m_networkTable->GetDoubleTopic("loopTime");
  topic.SetProperties(wpi::util::json::object("unit", "second"));
  m_loopTimePublisher = topic.Publish();
}

void MechanismTelemetry::SetupTelemetry(const std::string& mechanismName,
                                        motorcontrollers::SmartMotorController& motorController) {
  auto inst = wpi::nt::NetworkTableInstance::GetDefault();
  m_tuningNetworkTable = inst.GetTable("Tuning")->GetSubTable(mechanismName);
  m_networkTable = inst.GetTable("Mechanisms")->GetSubTable(mechanismName);
  motorController.SetupTelemetry(m_networkTable, m_tuningNetworkTable);
  SetupLoopTime();
}

void MechanismTelemetry::SetupTelemetry(const std::string& mechanismName) {
  auto inst = wpi::nt::NetworkTableInstance::GetDefault();
  m_tuningNetworkTable = inst.GetTable("Tuning")->GetSubTable(mechanismName);
  m_networkTable = inst.GetTable("Mechanisms")->GetSubTable(mechanismName);
  SetupLoopTime();
}

void MechanismTelemetry::UpdateLoopTime() {
  if (!m_loopTimePublisher) return;
  double now = wpi::Timer::GetTimestamp().value();
  if (m_prevTimestamp != 0.0) {
    m_loopTimePublisher->Set(now - m_prevTimestamp);
  }
  m_prevTimestamp = now;
}

std::shared_ptr<wpi::nt::NetworkTable> MechanismTelemetry::GetDataTable() const {
  return m_networkTable;
}

std::shared_ptr<wpi::nt::NetworkTable> MechanismTelemetry::GetTuningTable() const {
  return m_tuningNetworkTable;
}

}  // namespace yams::telemetry
