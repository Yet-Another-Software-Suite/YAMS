// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

#include "yams/telemetry/SwerveDriveTelemetryConfig.hpp"

#include <stdexcept>
#include <tuple>
#include <utility>

namespace yams::telemetry {

SwerveDriveTelemetryConfig::SwerveDriveTelemetryConfig() {
  m_poseFields.emplace(
      StructTelemetryField::TargetPose,
      StructTelemetry<frc::Pose2d, StructTelemetryField>{
          "tuning/driveToPose", frc::Pose2d{}, StructTelemetryField::TargetPose, true});
  m_poseFields.emplace(
      StructTelemetryField::Pose,
      StructTelemetry<frc::Pose2d, StructTelemetryField>{
          "pose", frc::Pose2d{}, StructTelemetryField::Pose, false});

  m_chassisSpeedsFields.emplace(
      StructTelemetryField::DesiredRobotRelativeChassisSpeeds,
      StructTelemetry<frc::ChassisSpeeds, StructTelemetryField>{
          "chassis/desired", frc::ChassisSpeeds{},
          StructTelemetryField::DesiredRobotRelativeChassisSpeeds, false});
  m_chassisSpeedsFields.emplace(
      StructTelemetryField::CurrentRobotRelativeChassisSpeeds,
      StructTelemetry<frc::ChassisSpeeds, StructTelemetryField>{
          "chassis/current", frc::ChassisSpeeds{},
          StructTelemetryField::CurrentRobotRelativeChassisSpeeds, false});
  m_chassisSpeedsFields.emplace(
      StructTelemetryField::FieldRelativeChassisSpeeds,
      StructTelemetry<frc::ChassisSpeeds, StructTelemetryField>{
          "chassis/field", frc::ChassisSpeeds{},
          StructTelemetryField::FieldRelativeChassisSpeeds, false});

  m_moduleStatesFields.emplace(
      StructArrayTelemetryField::DesiredModuleStates,
      StructArrayTelemetry<frc::SwerveModuleState, StructArrayTelemetryField>{
          "states/desired", std::vector<frc::SwerveModuleState>{},
          StructArrayTelemetryField::DesiredModuleStates, false});
  m_moduleStatesFields.emplace(
      StructArrayTelemetryField::CurrentModuleStates,
      StructArrayTelemetry<frc::SwerveModuleState, StructArrayTelemetryField>{
          "states/current", std::vector<frc::SwerveModuleState>{},
          StructArrayTelemetryField::CurrentModuleStates, false});

  for (auto [field, key, unit] :
       {std::tuple{DoubleTelemetryField::TranslationP, "autoalign/translation/p", "meters"},
        std::tuple{DoubleTelemetryField::TranslationI, "autoalign/translation/i", "meters"},
        std::tuple{DoubleTelemetryField::TranslationD, "autoalign/translation/d", "meters"},
        std::tuple{DoubleTelemetryField::RotationP, "autoalign/rotation/p", "radians"},
        std::tuple{DoubleTelemetryField::RotationI, "autoalign/rotation/i", "radians"},
        std::tuple{DoubleTelemetryField::RotationD, "autoalign/rotation/d", "radians"}}) {
    m_doubleFields.emplace(field, DoubleTelemetry<DoubleTelemetryField>{key, 0.0, field, true,
                                                                        unit});
  }
  m_doubleFields.emplace(DoubleTelemetryField::Gyro,
                         DoubleTelemetry<DoubleTelemetryField>{"gyro", 0.0,
                                                               DoubleTelemetryField::Gyro, false,
                                                               "degrees"});
}

SwerveDriveTelemetryConfig::SwerveDriveTelemetryConfig(TelemetryVerbosity verbosity)
    : SwerveDriveTelemetryConfig() {
  WithTelemetryVerbosity(verbosity);
}

SwerveDriveTelemetryConfig& SwerveDriveTelemetryConfig::WithDataLogName(
    const std::string& dataLogName) {
  m_dataLogName = dataLogName;
  return *this;
}

SwerveDriveTelemetryConfig& SwerveDriveTelemetryConfig::WithNetworkTables(bool enabled) {
  m_nt4Telemetry = enabled;
  return *this;
}

SwerveDriveTelemetryConfig& SwerveDriveTelemetryConfig::WithoutNetworkTables() {
  m_nt4Telemetry = false;
  return *this;
}

SwerveDriveTelemetryConfig& SwerveDriveTelemetryConfig::WithTelemetryVerbosity(
    TelemetryVerbosity verbosity) {
  switch (verbosity) {
    case TelemetryVerbosity::HIGH:
      m_chassisSpeedsFields.at(StructTelemetryField::DesiredRobotRelativeChassisSpeeds).Enable();
      m_moduleStatesFields.at(StructArrayTelemetryField::DesiredModuleStates).Enable();
      m_doubleFields.at(DoubleTelemetryField::TranslationP).Enable();
      m_doubleFields.at(DoubleTelemetryField::TranslationI).Enable();
      m_doubleFields.at(DoubleTelemetryField::TranslationD).Enable();
      m_doubleFields.at(DoubleTelemetryField::RotationP).Enable();
      m_doubleFields.at(DoubleTelemetryField::RotationI).Enable();
      m_doubleFields.at(DoubleTelemetryField::RotationD).Enable();
      [[fallthrough]];
    case TelemetryVerbosity::MEDIUM:
      m_chassisSpeedsFields.at(StructTelemetryField::CurrentRobotRelativeChassisSpeeds).Enable();
      m_chassisSpeedsFields.at(StructTelemetryField::FieldRelativeChassisSpeeds).Enable();
      m_moduleStatesFields.at(StructArrayTelemetryField::CurrentModuleStates).Enable();
      [[fallthrough]];
    case TelemetryVerbosity::LOW:
      m_poseFields.at(StructTelemetryField::Pose).Enable();
      m_doubleFields.at(DoubleTelemetryField::Gyro).Enable();
      break;
    case TelemetryVerbosity::NONE:
      break;
  }
  return *this;
}

SwerveDriveTelemetryConfig& SwerveDriveTelemetryConfig::WithPose() {
  m_poseFields.at(StructTelemetryField::Pose).Enable();
  return *this;
}

SwerveDriveTelemetryConfig& SwerveDriveTelemetryConfig::WithGyro() {
  m_doubleFields.at(DoubleTelemetryField::Gyro).Enable();
  return *this;
}

SwerveDriveTelemetryConfig& SwerveDriveTelemetryConfig::WithDesiredRobotRelativeChassisSpeeds() {
  m_chassisSpeedsFields.at(StructTelemetryField::DesiredRobotRelativeChassisSpeeds).Enable();
  return *this;
}

SwerveDriveTelemetryConfig& SwerveDriveTelemetryConfig::WithCurrentRobotRelativeChassisSpeeds() {
  m_chassisSpeedsFields.at(StructTelemetryField::CurrentRobotRelativeChassisSpeeds).Enable();
  return *this;
}

SwerveDriveTelemetryConfig& SwerveDriveTelemetryConfig::WithFieldRelativeChassisSpeeds() {
  m_chassisSpeedsFields.at(StructTelemetryField::FieldRelativeChassisSpeeds).Enable();
  return *this;
}

SwerveDriveTelemetryConfig& SwerveDriveTelemetryConfig::WithDesiredModuleStates() {
  m_moduleStatesFields.at(StructArrayTelemetryField::DesiredModuleStates).Enable();
  return *this;
}

SwerveDriveTelemetryConfig& SwerveDriveTelemetryConfig::WithCurrentModuleStates() {
  m_moduleStatesFields.at(StructArrayTelemetryField::CurrentModuleStates).Enable();
  return *this;
}

std::optional<std::string> SwerveDriveTelemetryConfig::GetDataLogName() const {
  return m_dataLogName;
}

bool SwerveDriveTelemetryConfig::GetNT4Enabled() const { return m_nt4Telemetry; }

std::unordered_map<SwerveDriveTelemetryConfig::DoubleTelemetryField,
                   DoubleTelemetry<SwerveDriveTelemetryConfig::DoubleTelemetryField>>&
SwerveDriveTelemetryConfig::GetDoubleFields() {
  return m_doubleFields;
}

std::unordered_map<
    SwerveDriveTelemetryConfig::StructTelemetryField,
    StructTelemetry<frc::Pose2d, SwerveDriveTelemetryConfig::StructTelemetryField>>&
SwerveDriveTelemetryConfig::GetPoseFields() {
  return m_poseFields;
}

std::unordered_map<
    SwerveDriveTelemetryConfig::StructTelemetryField,
    StructTelemetry<frc::ChassisSpeeds, SwerveDriveTelemetryConfig::StructTelemetryField>>&
SwerveDriveTelemetryConfig::GetChassisSpeedsFields() {
  return m_chassisSpeedsFields;
}

std::unordered_map<
    SwerveDriveTelemetryConfig::StructArrayTelemetryField,
    StructArrayTelemetry<frc::SwerveModuleState,
                         SwerveDriveTelemetryConfig::StructArrayTelemetryField>>&
SwerveDriveTelemetryConfig::GetModuleStatesFields() {
  return m_moduleStatesFields;
}

SwerveDriveTelemetryConfig& SwerveDriveTelemetryConfig::WithCustom(DoubleTelemetryField field,
                                                                   bool value) {
  auto& dt = m_doubleFields.at(field);
  value ? dt.Enable() : dt.Disable();
  return *this;
}

SwerveDriveTelemetryConfig& SwerveDriveTelemetryConfig::WithCustom(StructTelemetryField field,
                                                                   bool value) {
  auto it = m_poseFields.find(field);
  if (it != m_poseFields.end()) {
    value ? it->second.Enable() : it->second.Disable();
    return *this;
  }
  auto& stt = m_chassisSpeedsFields.at(field);
  value ? stt.Enable() : stt.Disable();
  return *this;
}

SwerveDriveTelemetryConfig& SwerveDriveTelemetryConfig::WithCustom(
    StructArrayTelemetryField field, bool value) {
  auto& stat = m_moduleStatesFields.at(field);
  value ? stat.Enable() : stat.Disable();
  return *this;
}

SwerveDriveTelemetryConfig& SwerveDriveTelemetryConfig::WithCustom(
    const std::vector<DoubleTelemetryField>& fields, bool value) {
  for (auto field : fields) WithCustom(field, value);
  return *this;
}

SwerveDriveTelemetryConfig& SwerveDriveTelemetryConfig::WithCustom(
    const std::vector<StructTelemetryField>& fields, bool value) {
  for (auto field : fields) WithCustom(field, value);
  return *this;
}

SwerveDriveTelemetryConfig& SwerveDriveTelemetryConfig::WithCustom(
    const std::vector<StructArrayTelemetryField>& fields, bool value) {
  for (auto field : fields) WithCustom(field, value);
  return *this;
}

}  // namespace yams::telemetry
