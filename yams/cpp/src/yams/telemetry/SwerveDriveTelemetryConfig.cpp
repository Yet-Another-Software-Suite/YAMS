// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

#include "yams/telemetry/SwerveDriveTelemetryConfig.hpp"

#include <stdexcept>
#include <tuple>
#include <utility>

namespace yams::telemetry {

SwerveDriveTelemetryConfig::SwerveDriveTelemetryConfig() {
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
  for (auto [field, key, unit, defaultVal] :
       {std::tuple{DoubleTelemetryField::AutoAlignPoseX, "autoalign/setpoint/x", "meters", 3.0},
        std::tuple{DoubleTelemetryField::AutoAlignPoseY, "autoalign/setpoint/y", "meters", 3.0},
        std::tuple{DoubleTelemetryField::AutoAlignPoseRotation, "autoalign/setpoint/rot",
                   "degrees", 0.0}}) {
    m_doubleFields.emplace(field, DoubleTelemetry<DoubleTelemetryField>{key, defaultVal, field,
                                                                        true, unit});
  }
  for (auto [field, key] :
       {std::tuple{DoubleTelemetryField::ModulesDriveP, "modules/drive/feedback/p"},
        std::tuple{DoubleTelemetryField::ModulesDriveI, "modules/drive/feedback/i"},
        std::tuple{DoubleTelemetryField::ModulesDriveD, "modules/drive/feedback/d"},
        std::tuple{DoubleTelemetryField::ModulesDriveKs, "modules/drive/feedforward/s"},
        std::tuple{DoubleTelemetryField::ModulesDriveKv, "modules/drive/feedforward/v"},
        std::tuple{DoubleTelemetryField::ModulesDriveKa, "modules/drive/feedforward/a"},
        std::tuple{DoubleTelemetryField::ModulesAzimuthP, "modules/azimuth/feedback/p"},
        std::tuple{DoubleTelemetryField::ModulesAzimuthI, "modules/azimuth/feedback/i"},
        std::tuple{DoubleTelemetryField::ModulesAzimuthD, "modules/azimuth/feedback/d"},
        std::tuple{DoubleTelemetryField::ModulesAzimuthKs, "modules/azimuth/feedforward/s"},
        std::tuple{DoubleTelemetryField::ModulesAzimuthKv, "modules/azimuth/feedforward/v"},
        std::tuple{DoubleTelemetryField::ModulesAzimuthKa, "modules/azimuth/feedforward/a"}}) {
    m_doubleFields.emplace(field, DoubleTelemetry<DoubleTelemetryField>{key, 0.0, field, true,
                                                                        "none"});
  }
  m_doubleFields.emplace(
      DoubleTelemetryField::ModulesDriveVelocity,
      DoubleTelemetry<DoubleTelemetryField>{"modules/drive/velocity", 0.0,
                                            DoubleTelemetryField::ModulesDriveVelocity, true,
                                            "meters_per_second"});
  m_doubleFields.emplace(
      DoubleTelemetryField::ModulesAzimuthAngle,
      DoubleTelemetry<DoubleTelemetryField>{"modules/azimuth/angle", 0.0,
                                            DoubleTelemetryField::ModulesAzimuthAngle, true,
                                            "degrees"});
  m_doubleFields.emplace(DoubleTelemetryField::Gyro,
                         DoubleTelemetry<DoubleTelemetryField>{"gyro", 0.0,
                                                               DoubleTelemetryField::Gyro, false,
                                                               "degrees"});

  m_boolFields.emplace(
      BooleanTelemetryField::AutoAlignEnabled,
      BooleanTelemetry<BooleanTelemetryField>{"autoalign/enabled", false,
                                              BooleanTelemetryField::AutoAlignEnabled, true});
  m_boolFields.emplace(
      BooleanTelemetryField::ModulesDriveTuningEnabled,
      BooleanTelemetry<BooleanTelemetryField>{"modules/drive/enabled", false,
                                              BooleanTelemetryField::ModulesDriveTuningEnabled,
                                              true});
  m_boolFields.emplace(
      BooleanTelemetryField::ModulesDriveInPlace,
      BooleanTelemetry<BooleanTelemetryField>{"modules/drive/inplace", false,
                                              BooleanTelemetryField::ModulesDriveInPlace, true});
  m_boolFields.emplace(
      BooleanTelemetryField::ModulesAzimuthTuningEnabled,
      BooleanTelemetry<BooleanTelemetryField>{"modules/azimuth/enabled", false,
                                              BooleanTelemetryField::ModulesAzimuthTuningEnabled,
                                              true});
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
      m_doubleFields.at(DoubleTelemetryField::AutoAlignPoseX).Enable();
      m_doubleFields.at(DoubleTelemetryField::AutoAlignPoseY).Enable();
      m_doubleFields.at(DoubleTelemetryField::AutoAlignPoseRotation).Enable();
      m_boolFields.at(BooleanTelemetryField::AutoAlignEnabled).Enable();
      m_doubleFields.at(DoubleTelemetryField::ModulesDriveP).Enable();
      m_doubleFields.at(DoubleTelemetryField::ModulesDriveI).Enable();
      m_doubleFields.at(DoubleTelemetryField::ModulesDriveD).Enable();
      m_doubleFields.at(DoubleTelemetryField::ModulesDriveKs).Enable();
      m_doubleFields.at(DoubleTelemetryField::ModulesDriveKv).Enable();
      m_doubleFields.at(DoubleTelemetryField::ModulesDriveKa).Enable();
      m_doubleFields.at(DoubleTelemetryField::ModulesDriveVelocity).Enable();
      m_doubleFields.at(DoubleTelemetryField::ModulesAzimuthP).Enable();
      m_doubleFields.at(DoubleTelemetryField::ModulesAzimuthI).Enable();
      m_doubleFields.at(DoubleTelemetryField::ModulesAzimuthD).Enable();
      m_doubleFields.at(DoubleTelemetryField::ModulesAzimuthKs).Enable();
      m_doubleFields.at(DoubleTelemetryField::ModulesAzimuthKv).Enable();
      m_doubleFields.at(DoubleTelemetryField::ModulesAzimuthKa).Enable();
      m_doubleFields.at(DoubleTelemetryField::ModulesAzimuthAngle).Enable();
      m_boolFields.at(BooleanTelemetryField::ModulesDriveTuningEnabled).Enable();
      m_boolFields.at(BooleanTelemetryField::ModulesAzimuthTuningEnabled).Enable();
      m_boolFields.at(BooleanTelemetryField::ModulesDriveInPlace).Enable();
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

std::unordered_map<SwerveDriveTelemetryConfig::BooleanTelemetryField,
                   BooleanTelemetry<SwerveDriveTelemetryConfig::BooleanTelemetryField>>&
SwerveDriveTelemetryConfig::GetBoolFields() {
  return m_boolFields;
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

SwerveDriveTelemetryConfig& SwerveDriveTelemetryConfig::WithCustom(BooleanTelemetryField field,
                                                                   bool value) {
  auto& bt = m_boolFields.at(field);
  value ? bt.Enable() : bt.Disable();
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

SwerveDriveTelemetryConfig& SwerveDriveTelemetryConfig::WithCustom(
    const std::vector<BooleanTelemetryField>& fields, bool value) {
  for (auto field : fields) WithCustom(field, value);
  return *this;
}

}  // namespace yams::telemetry
