// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

#include "yams/mechanisms/swerve/SwerveModule.hpp"

#include <frc/RobotBase.h>
#include <frc/geometry/Rotation2d.h>
#include <networktables/NetworkTableInstance.h>
#include <units/angle.h>

#include <stdexcept>
#include <string>

#include "yams/exceptions.hpp"

namespace yams::mechanisms::swerve {

SwerveModule::SwerveModule(config::SwerveModuleConfig* config)
    : m_driveMotorController{config->GetDriveMotor()},
      m_azimuthMotorController{config->GetAzimuthMotor()} {
  m_config = config;
  if (!m_config->GetTelemetryName()) {
    throw std::invalid_argument("SwerveModuleConfig must have a telemetry name!");
  }
  if (!m_config->GetLocation()) {
    throw std::invalid_argument("SwerveModuleConfig must have a position!");
  }
  // Mirror Java: if the azimuth motor has an external encoder configured but
  // external feedback is not enabled, that encoder can never be used.
  if (m_azimuthMotorController->GetConfig().GetExternalEncoder().has_value() &&
      !m_azimuthMotorController->GetConfig().GetUseExternalFeedback()) {
    throw exceptions::SmartMotorControllerConfigurationException(
        "External encoder cannot be used without external feedback",
        "External encoder could not be used", "WithUseExternalFeedbackEncoder(true)");
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
    // Only seed from the absolute encoder when the motor is NOT using an
    // external feedback encoder (e.g., a fused CANcoder on a TalonFX).  When
    // external feedback is active the hardware already tracks the absolute
    // angle; seeding would overwrite that with redundant data.
    const auto& cfg = m_azimuthMotorController->GetConfig();
    if (!cfg.GetExternalEncoder().has_value() || !cfg.GetUseExternalFeedback()) {
      m_azimuthMotorController->SetEncoderPosition(m_config->GetAbsoluteEncoderAngle());
    }
  }
}

std::string SwerveModule::GetName() const {
  return m_config->GetTelemetryName().value_or("SwerveModule");
}

const config::SwerveModuleConfig& SwerveModule::GetConfig() const { return *m_config; }

void SwerveModule::SetSwerveModuleState(frc::SwerveModuleState state) {
  state = m_config->GetOptimizedState(state);
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
