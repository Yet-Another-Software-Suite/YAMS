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
