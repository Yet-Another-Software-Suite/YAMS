// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

#include "yams/mechanisms/swerve/SwerveModule.hpp"

#include <stdexcept>
#include <string>
#include <utility>
#include <wpi/framework/RobotBase.hpp>
#include <wpi/math/geometry/Rotation2d.hpp>
#include <wpi/units/angle.hpp>

#include "yams/exceptions.hpp"
#include "yams/motorcontrollers/SmartMotorControllerConfig.hpp"

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

  SeedAzimuthEncoder();
  m_azimuthEncoderWithoutOffsets = config->GetRawAbsoluteEncoderAngle();
}

void SwerveModule::SetupTelemetry(const std::string& mechName) {
  using TelemetryVerbosity = motorcontrollers::SmartMotorControllerConfig::TelemetryVerbosity;

  m_telemetry.SetupTelemetry(mechName + "/modules/" + GetName());

  telemetry::SwerveModuleTelemetryConfig telemetryCfg;
  if (auto specified = m_config->GetSwerveModuleTelemetryConfig()) {
    telemetryCfg = std::move(*specified);
  } else {
    telemetryCfg.WithTelemetryVerbosity(
        m_config->GetTelemetryVerbosity().value_or(TelemetryVerbosity::HIGH));
  }

  m_swerveModuleTelemetry.emplace(std::move(telemetryCfg));
  m_swerveModuleTelemetry->SetupTelemetry(mechName, *this);
}

void SwerveModule::SeedAzimuthEncoder() {
  if (wpi::RobotBase::IsReal()) {
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

config::SwerveModuleConfig& SwerveModule::GetMutableConfig() { return *m_config; }

wpi::math::SwerveModuleVelocity SwerveModule::SetSwerveModuleState(
    wpi::math::SwerveModuleVelocity state) {
  state = m_config->GetOptimizedState(state, GetState());
  m_driveMotorController->SetVelocity(state.velocity + GetCouplingVelocity());
  m_azimuthMotorController->SetPosition(wpi::units::degree_t{state.angle.Degrees()});
  return state;
}

wpi::math::SwerveModuleVelocity SwerveModule::SetSwerveModuleState(
    wpi::math::SwerveModuleVelocity state, wpi::units::newton_t feedforwardForce) {
  const wpi::math::Rotation2d requestedAngle = state.angle;
  state = m_config->GetOptimizedState(state, GetState());
  // The force is along the requested direction, so it reverses with the wheel.
  if ((state.angle - requestedAngle).Cos() < 0) {
    feedforwardForce = -feedforwardForce;
  }
  feedforwardForce = feedforwardForce * m_config->GetCosineCompensationScalar(state.angle);
  m_driveMotorController->SetVelocity(state.velocity + GetCouplingVelocity(), feedforwardForce);
  m_azimuthMotorController->SetPosition(wpi::units::degree_t{state.angle.Degrees()});
  return state;
}

wpi::math::SwerveModuleVelocity SwerveModule::SetAzimuthAngle(wpi::math::Rotation2d angle) {
  auto state = m_config->GetLockedState(angle, GetState());
  m_driveMotorController->SetVelocity(wpi::units::meters_per_second_t{0} + GetCouplingVelocity());
  m_azimuthMotorController->SetPosition(wpi::units::degree_t{state.angle.Degrees()});
  return state;
}

void SwerveModule::SetDriveVoltage(wpi::units::volt_t voltage, wpi::math::Rotation2d angle) {
  m_driveMotorController->SetVoltage(voltage);
  m_azimuthMotorController->SetPosition(wpi::units::degree_t{angle.Degrees()});
}

void SwerveModule::SetAzimuthVoltage(wpi::units::volt_t voltage) {
  m_driveMotorController->SetVelocity(wpi::units::meters_per_second_t{0} + GetCouplingVelocity());
  m_azimuthMotorController->SetVoltage(voltage);
}

void SwerveModule::ApplyTuningValue(
    telemetry::SwerveDriveTelemetryConfig::DoubleTelemetryField field, double value, bool changed) {
  if (!changed) return;
  using Field = telemetry::SwerveDriveTelemetryConfig::DoubleTelemetryField;
  switch (field) {
    case Field::ModulesDriveP:
      m_driveMotorController->SetKp(value);
      break;
    case Field::ModulesDriveI:
      m_driveMotorController->SetKi(value);
      break;
    case Field::ModulesDriveD:
      m_driveMotorController->SetKd(value);
      break;
    case Field::ModulesDriveKs:
      m_driveMotorController->SetKs(value);
      break;
    case Field::ModulesDriveKv:
      m_driveMotorController->SetKv(value);
      break;
    case Field::ModulesDriveKa:
      m_driveMotorController->SetKa(value);
      break;
    case Field::ModulesAzimuthP:
      m_azimuthMotorController->SetKp(value);
      break;
    case Field::ModulesAzimuthI:
      m_azimuthMotorController->SetKi(value);
      break;
    case Field::ModulesAzimuthD:
      m_azimuthMotorController->SetKd(value);
      break;
    case Field::ModulesAzimuthKs:
      m_azimuthMotorController->SetKs(value);
      break;
    case Field::ModulesAzimuthKv:
      m_azimuthMotorController->SetKv(value);
      break;
    case Field::ModulesAzimuthKa:
      m_azimuthMotorController->SetKa(value);
      break;
    default:
      break;
  }
}

SwerveModuleVelocityWithAzimuth SwerveModule::GetState() const {
  return SwerveModuleVelocityWithAzimuth{
      m_driveMotorController->GetMeasurementVelocity() - GetCouplingVelocity(),
      wpi::math::Rotation2d{wpi::units::radian_t{m_azimuthMotorController->GetMechanismPosition()}},
      wpi::units::radians_per_second_t{m_azimuthMotorController->GetMechanismVelocity()}};
}

wpi::math::SwerveModulePosition SwerveModule::GetPosition() const {
  const wpi::math::Rotation2d azimuthAngle{
      wpi::units::radian_t{m_azimuthMotorController->GetMechanismPosition()}};
  if (m_lastAzimuthAngle) {
    m_azimuthTravel += wpi::units::turn_t{(azimuthAngle - *m_lastAzimuthAngle).Radians()};
  }
  m_lastAzimuthAngle = azimuthAngle;
  return wpi::math::SwerveModulePosition{
      m_driveMotorController->GetMeasurementPosition() - GetCouplingDistance(m_azimuthTravel),
      azimuthAngle};
}

double SwerveModule::GetDriveRotorToMechanismRatio() const {
  const auto& gearing = m_driveMotorController->GetConfig().GetMotorGearing();
  return gearing ? gearing->GetRotorToMechanismRatio() : 1.0;
}

wpi::units::meter_t SwerveModule::GetCouplingDistance(wpi::units::turn_t azimuthAngle) const {
  const auto couplingRatio = m_config->GetCouplingRatio();
  if (!couplingRatio) {
    return wpi::units::meter_t{0};
  }
  const auto& driveConfig = m_driveMotorController->GetConfig();
  return driveConfig.ConvertFromMechanism(azimuthAngle *
                                          couplingRatio->GetOutputToInputConversionFactor() *
                                          GetDriveRotorToMechanismRatio());
}

wpi::units::meters_per_second_t SwerveModule::GetCouplingVelocity() const {
  const auto couplingRatio = m_config->GetCouplingRatio();
  if (!couplingRatio) {
    return wpi::units::meters_per_second_t{0};
  }
  const auto& driveConfig = m_driveMotorController->GetConfig();
  return driveConfig.ConvertFromMechanism(m_azimuthMotorController->GetMechanismVelocity() *
                                          couplingRatio->GetOutputToInputConversionFactor() *
                                          GetDriveRotorToMechanismRatio());
}

wpi::units::degree_t SwerveModule::GetRawAbsoluteEncoderAngle() const {
  return m_azimuthEncoderWithoutOffsets();
}

void SwerveModule::UpdateTelemetry() {
  m_driveMotorController->UpdateTelemetry();
  m_azimuthMotorController->UpdateTelemetry();
  m_swerveModuleTelemetry->Publish(*this);
  m_telemetry.UpdateLoopTime();
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
