// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

#include "yams/telemetry/SwerveModuleTelemetryConfig.hpp"

#include <utility>

namespace yams::telemetry {

SwerveModuleTelemetryConfig::SwerveModuleTelemetryConfig() {
  m_doubleFields.emplace(
      DoubleTelemetryField::AbsoluteEncoder,
      DoubleTelemetry<DoubleTelemetryField>{"encoder", 0.0, DoubleTelemetryField::AbsoluteEncoder,
                                            false, "degrees"});
  m_structFields.emplace(
      StructTelemetryField::State,
      StructTelemetry<frc::SwerveModuleState, StructTelemetryField>{
          "state", frc::SwerveModuleState{}, StructTelemetryField::State, false});
}

SwerveModuleTelemetryConfig& SwerveModuleTelemetryConfig::WithDataLogName(
    const std::string& dataLogName) {
  m_dataLogName = dataLogName;
  return *this;
}

SwerveModuleTelemetryConfig& SwerveModuleTelemetryConfig::WithNetworkTables(bool enabled) {
  m_nt4Telemetry = enabled;
  return *this;
}

SwerveModuleTelemetryConfig& SwerveModuleTelemetryConfig::WithoutNetworkTables() {
  m_nt4Telemetry = false;
  return *this;
}

SwerveModuleTelemetryConfig& SwerveModuleTelemetryConfig::WithTelemetryVerbosity(
    TelemetryVerbosity verbosity) {
  switch (verbosity) {
    case TelemetryVerbosity::HIGH:
    case TelemetryVerbosity::MEDIUM:
    case TelemetryVerbosity::LOW:
      m_doubleFields.at(DoubleTelemetryField::AbsoluteEncoder).Enable();
      m_structFields.at(StructTelemetryField::State).Enable();
      break;
    case TelemetryVerbosity::NONE:
      break;
  }
  return *this;
}

SwerveModuleTelemetryConfig& SwerveModuleTelemetryConfig::WithAbsoluteEncoder() {
  m_doubleFields.at(DoubleTelemetryField::AbsoluteEncoder).Enable();
  return *this;
}

SwerveModuleTelemetryConfig& SwerveModuleTelemetryConfig::WithState() {
  m_structFields.at(StructTelemetryField::State).Enable();
  return *this;
}

std::optional<std::string> SwerveModuleTelemetryConfig::GetDataLogName() const {
  return m_dataLogName;
}

bool SwerveModuleTelemetryConfig::GetNT4Enabled() const { return m_nt4Telemetry; }

std::unordered_map<SwerveModuleTelemetryConfig::DoubleTelemetryField,
                   DoubleTelemetry<SwerveModuleTelemetryConfig::DoubleTelemetryField>>&
SwerveModuleTelemetryConfig::GetDoubleFields() {
  return m_doubleFields;
}

std::unordered_map<
    SwerveModuleTelemetryConfig::StructTelemetryField,
    StructTelemetry<frc::SwerveModuleState, SwerveModuleTelemetryConfig::StructTelemetryField>>&
SwerveModuleTelemetryConfig::GetStructFields() {
  return m_structFields;
}

SwerveModuleTelemetryConfig& SwerveModuleTelemetryConfig::WithCustom(DoubleTelemetryField field,
                                                                     bool value) {
  auto& dt = m_doubleFields.at(field);
  value ? dt.Enable() : dt.Disable();
  return *this;
}

SwerveModuleTelemetryConfig& SwerveModuleTelemetryConfig::WithCustom(StructTelemetryField field,
                                                                     bool value) {
  auto& stt = m_structFields.at(field);
  value ? stt.Enable() : stt.Disable();
  return *this;
}

SwerveModuleTelemetryConfig& SwerveModuleTelemetryConfig::WithCustom(
    const std::vector<DoubleTelemetryField>& fields, bool value) {
  for (auto field : fields) WithCustom(field, value);
  return *this;
}

SwerveModuleTelemetryConfig& SwerveModuleTelemetryConfig::WithCustom(
    const std::vector<StructTelemetryField>& fields, bool value) {
  for (auto field : fields) WithCustom(field, value);
  return *this;
}

}  // namespace yams::telemetry
