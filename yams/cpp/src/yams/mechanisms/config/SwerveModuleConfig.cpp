// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

#include "yams/mechanisms/config/SwerveModuleConfig.hpp"

#include <frc/geometry/Rotation2d.h>
#include <units/math.h>

#include <cmath>
#include <numbers>
#include <stdexcept>
#include <string>
#include <utility>

namespace yams::mechanisms::config {

SwerveModuleConfig::SwerveModuleConfig(motorcontrollers::SmartMotorController* drive,
                                       motorcontrollers::SmartMotorController* azimuth)
    : m_driveMotor{drive}, m_azimuthMotor{azimuth} {}

SwerveModuleConfig& SwerveModuleConfig::WithSmartMotorController(
    motorcontrollers::SmartMotorController* driveMotor,
    motorcontrollers::SmartMotorController* azimuthMotor) {
  if (m_driveMotor) throw std::logic_error("Drive motor controller already set.");
  if (m_azimuthMotor) throw std::logic_error("Azimuth motor controller already set.");
  m_driveMotor = driveMotor;
  m_azimuthMotor = azimuthMotor;
  if (m_wheelCircumference && driveMotor) {
    driveMotor->GetConfig().WithMechanismCircumference(*m_wheelCircumference);
  }
  return *this;
}

SwerveModuleConfig& SwerveModuleConfig::WithCosineCompensation(bool compensate) {
  m_cosineCompensation = compensate;
  return *this;
}

SwerveModuleConfig& SwerveModuleConfig::WithAbsoluteEncoder(
    std::function<units::degree_t()> supplier) {
  m_absoluteEncoderSupplier = std::move(supplier);
  return *this;
}

SwerveModuleConfig& SwerveModuleConfig::WithAbsoluteEncoderGearing(
    const gearing::GearBox& gearing) {
  m_absoluteEncoderGearbox = gearing;
  return *this;
}

SwerveModuleConfig& SwerveModuleConfig::WithAbsoluteEncoderOffset(units::degree_t offset) {
  m_absoluteEncoderOffset = offset;
  if (m_azimuthMotor) {
    m_azimuthMotor->GetConfig().WithExternalEncoderZeroOffset(offset);
  }
  return *this;
}

SwerveModuleConfig& SwerveModuleConfig::WithLocation(frc::Translation2d location) {
  m_location = location;
  return *this;
}

SwerveModuleConfig& SwerveModuleConfig::WithLocation(units::meter_t front, units::meter_t left) {
  m_location = frc::Translation2d{front, left};
  return *this;
}

SwerveModuleConfig& SwerveModuleConfig::WithLocation(units::meter_t distance,
                                                     units::degree_t angle) {
  m_location = frc::Translation2d{distance, frc::Rotation2d{angle}};
  return *this;
}

SwerveModuleConfig& SwerveModuleConfig::WithWheelRadius(units::meter_t radius) {
  m_wheelCircumference = radius * 2.0 * std::numbers::pi;
  if (m_driveMotor) {
    m_driveMotor->GetConfig().WithMechanismCircumference(*m_wheelCircumference);
  }
  return *this;
}

SwerveModuleConfig& SwerveModuleConfig::WithWheelDiameter(units::meter_t diameter) {
  m_wheelCircumference = diameter * std::numbers::pi;
  if (m_driveMotor) {
    m_driveMotor->GetConfig().WithMechanismCircumference(*m_wheelCircumference);
  }
  return *this;
}

SwerveModuleConfig& SwerveModuleConfig::WithMinimumVelocity(units::meters_per_second_t speed) {
  m_minimumVelocity = speed;
  return *this;
}

SwerveModuleConfig& SwerveModuleConfig::WithOptimization(bool enable) {
  m_stateOptimization = enable;
  return *this;
}

SwerveModuleConfig& SwerveModuleConfig::WithTelemetry(const std::string& name,
                                                      TelemetryVerbosity verbosity) {
  m_telemetryName = name;
  m_telemetryVerbosity = verbosity;
  return *this;
}

// ---- Getters ------------------------------------------------------------------

motorcontrollers::SmartMotorController* SwerveModuleConfig::GetDriveMotor() const {
  if (!m_driveMotor) throw std::logic_error("Drive motor not configured.");
  return m_driveMotor;
}

motorcontrollers::SmartMotorController* SwerveModuleConfig::GetAzimuthMotor() const {
  if (!m_azimuthMotor) throw std::logic_error("Azimuth motor not configured.");
  return m_azimuthMotor;
}

std::optional<std::string> SwerveModuleConfig::GetTelemetryName() const { return m_telemetryName; }

std::optional<SwerveModuleConfig::TelemetryVerbosity> SwerveModuleConfig::GetTelemetryVerbosity()
    const {
  return m_telemetryVerbosity;
}

std::optional<frc::Translation2d> SwerveModuleConfig::GetLocation() const { return m_location; }

bool SwerveModuleConfig::GetStateOptimization() const { return m_stateOptimization; }

units::degree_t SwerveModuleConfig::GetAbsoluteEncoderAngle() const {
  if (m_absoluteEncoderSupplier) {
    auto rawAngle = (*m_absoluteEncoderSupplier)();
    auto geared = rawAngle * m_absoluteEncoderGearbox.GetInputToOutputConversionFactor();
    auto offset = m_absoluteEncoderOffset.value_or(units::degree_t{0});
    return geared - offset;
  }
  if (!m_azimuthMotor) return units::degree_t{0};
  return m_azimuthMotor->GetMechanismPosition();
}

double SwerveModuleConfig::GetCosineCompensatedVelocity(
    const frc::SwerveModuleState& desiredState) const {
  auto diff = desiredState.angle - frc::Rotation2d{units::radian_t{GetAbsoluteEncoderAngle()}};
  double cosineScalar = diff.Cos();
  if (cosineScalar < 0.0) cosineScalar = 1.0;
  return desiredState.speed.value() * cosineScalar;
}

frc::SwerveModuleState SwerveModuleConfig::GetOptimizedState(frc::SwerveModuleState state) const {
  if (m_minimumVelocity) {
    if (units::math::abs(state.speed) <= *m_minimumVelocity) {
      state = frc::SwerveModuleState{units::meters_per_second_t{0},
                                     frc::Rotation2d{units::radian_t{GetAbsoluteEncoderAngle()}}};
    }
  }
  if (m_stateOptimization) {
    state.Optimize(frc::Rotation2d{units::radian_t{GetAbsoluteEncoderAngle()}});
  }
  if (m_cosineCompensation) {
    state.speed = units::meters_per_second_t{GetCosineCompensatedVelocity(state)};
  }
  return state;
}

}  // namespace yams::mechanisms::config
