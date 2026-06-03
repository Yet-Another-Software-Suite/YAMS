// Copyright (c) 2026 YAMS Contributors
// SPDX-License-Identifier: LGPL-3.0-or-later

#include "yams/mechanisms/swerve/SwerveModule.h"

#include <frc/RobotBase.h>
#include <frc/geometry/Rotation2d.h>
#include <networktables/NetworkTableInstance.h>
#include <units/angle.h>

#include <stdexcept>
#include <string>

namespace yams::mechanisms::swerve {

SwerveModule::SwerveModule(const config::SwerveModuleConfig& config)
    : m_config{config},
      m_driveMotorController{config.GetDriveMotor()},
      m_azimuthMotorController{config.GetAzimuthMotor()} {
  if (!m_config.GetTelemetryName()) {
    throw std::invalid_argument("SwerveModuleConfig must have a telemetry name!");
  }
  if (!m_config.GetLocation()) {
    throw std::invalid_argument("SwerveModuleConfig must have a position!");
  }

  // Set up motor telemetry under the swerve hierarchy.
  auto instance = nt::NetworkTableInstance::GetDefault();
  auto driveTable = instance.GetTable("SmartDashboard/swerve/" + GetName() + "/drive");
  auto driveTuning = instance.GetTable("SmartDashboard/swerve/" + GetName() + "/drive/tuning");
  auto azimuthTable = instance.GetTable("SmartDashboard/swerve/" + GetName() + "/azimuth");
  auto azimuthTuning = instance.GetTable("SmartDashboard/swerve/" + GetName() + "/azimuth/tuning");

  m_driveMotorController->SetupTelemetry(driveTable, driveTuning);
  m_azimuthMotorController->SetupTelemetry(azimuthTable, azimuthTuning);

  SeedAzimuthEncoder();
}

void SwerveModule::SeedAzimuthEncoder() {
  if (frc::RobotBase::IsReal()) {
    m_azimuthMotorController->SetEncoderPosition(m_config.GetAbsoluteEncoderAngle());
  }
}

std::string SwerveModule::GetName() const {
  return m_config.GetTelemetryName().value_or("SwerveModule");
}

const config::SwerveModuleConfig& SwerveModule::GetConfig() const { return m_config; }

void SwerveModule::SetSwerveModuleState(frc::SwerveModuleState state) {
  state = m_config.GetOptimizedState(state);
  m_driveMotorController->SetVelocity(state.speed);
  m_azimuthMotorController->SetPosition(units::degree_t{state.angle.Degrees()});
}

frc::SwerveModuleState SwerveModule::GetState() const {
  return frc::SwerveModuleState{
      m_driveMotorController->GetMeasurementVelocity(),
      frc::Rotation2d{units::radian_t{m_azimuthMotorController->GetMechanismPosition()}}};
}

frc::SwerveModulePosition SwerveModule::GetPosition() const {
  return frc::SwerveModulePosition{
      m_driveMotorController->GetMeasurementPosition(),
      frc::Rotation2d{units::radian_t{m_azimuthMotorController->GetMechanismPosition()}}};
}

void SwerveModule::UpdateTelemetry() {
  m_driveMotorController->UpdateTelemetry();
  m_azimuthMotorController->UpdateTelemetry();
}

void SwerveModule::SimIterate() {
  m_driveMotorController->SimIterate();
  m_azimuthMotorController->SimIterate();
}

motorcontrollers::SmartMotorController* SwerveModule::GetDriveMotorController() const {
  return m_driveMotorController;
}

motorcontrollers::SmartMotorController* SwerveModule::GetAzimuthMotorController() const {
  return m_azimuthMotorController;
}

}  // namespace yams::mechanisms::swerve
