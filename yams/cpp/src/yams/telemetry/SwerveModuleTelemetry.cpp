// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

#include "yams/telemetry/SwerveModuleTelemetry.hpp"

#include <networktables/NetworkTableInstance.h>
#include <units/angle.h>

#include <utility>

#include "yams/mechanisms/swerve/SwerveModule.hpp"
#include "yams/motorcontrollers/SmartMotorController.hpp"

namespace yams::telemetry {

SwerveModuleTelemetry::SwerveModuleTelemetry(SwerveModuleTelemetryConfig config)
    : m_config{std::move(config)} {}

void SwerveModuleTelemetry::SetupTelemetry(const std::string& mechName,
                                           mechanisms::swerve::SwerveModule& module) {
  auto inst = nt::NetworkTableInstance::GetDefault();
  m_dataTable = inst.GetTable("Mechanisms")
                    ->GetSubTable(mechName)
                    ->GetSubTable("modules")
                    ->GetSubTable(module.GetName());
  m_tuningTable = inst.GetTable("Tuning")
                      ->GetSubTable(mechName)
                      ->GetSubTable("modules")
                      ->GetSubTable(module.GetName());

  bool nt4Enabled = m_config.GetNT4Enabled();
  auto dataLogName = m_config.GetDataLogName();

  for (auto& [field, dt] : m_config.GetDoubleFields()) {
    if (!dt.IsEnabled()) continue;
    if (nt4Enabled) dt.SetupNetworkTables(m_dataTable, m_tuningTable);
    if (dataLogName) dt.SetupDataLog(*dataLogName);
  }
  for (auto& [field, stt] : m_config.GetStructFields()) {
    if (!stt.IsEnabled()) continue;
    if (nt4Enabled) stt.SetupNetworkTables(m_dataTable, m_tuningTable);
    if (dataLogName) stt.SetupDataLog(*dataLogName);
  }

  module.GetDriveMotorController()->SetupTelemetry(m_dataTable, m_tuningTable);
  module.GetAzimuthMotorController()->SetupTelemetry(m_dataTable, m_tuningTable);
}

void SwerveModuleTelemetry::Publish(mechanisms::swerve::SwerveModule& module) {
  for (auto& [field, dt] : m_config.GetDoubleFields()) {
    if (!dt.IsEnabled()) continue;
    switch (field) {
      case DoubleTelemetryField::AbsoluteEncoder:
        dt.Set(module.GetRawAbsoluteEncoderAngle().value());
        break;
      default:
        break;
    }
  }
  for (auto& [field, stt] : m_config.GetStructFields()) {
    if (!stt.IsEnabled()) continue;
    switch (field) {
      case StructTelemetryField::State:
        stt.Set(module.GetState());
        break;
      default:
        break;
    }
  }
}

void SwerveModuleTelemetry::Close() {
  for (auto& [field, dt] : m_config.GetDoubleFields()) dt.Close();
  for (auto& [field, stt] : m_config.GetStructFields()) stt.Close();
}

}  // namespace yams::telemetry
