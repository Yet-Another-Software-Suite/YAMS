// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

#include "yams/mechanisms/config/SwerveModuleConfig.hpp"

#include <wpi/math/geometry/Rotation2d.hpp>
#include <wpi/units/math.hpp>

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
    std::function<wpi::units::degree_t()> supplier) {
  m_absoluteEncoderSupplier = std::move(supplier);
  return *this;
}

SwerveModuleConfig& SwerveModuleConfig::WithAbsoluteEncoderGearing(
    const gearing::GearBox& gearing) {
  m_absoluteEncoderGearbox = gearing;
  return *this;
}

SwerveModuleConfig& SwerveModuleConfig::WithAbsoluteEncoderOffset(wpi::units::degree_t offset) {
  m_absoluteEncoderOffset = offset;
  if (m_azimuthMotor) {
    m_azimuthMotor->GetConfig().WithExternalEncoderZeroOffset(offset);
  }
  return *this;
}

SwerveModuleConfig& SwerveModuleConfig::WithLocation(wpi::math::Translation2d location) {
  m_location = location;
  return *this;
}

SwerveModuleConfig& SwerveModuleConfig::WithLocation(wpi::units::meter_t front, wpi::units::meter_t left) {
  m_location = wpi::math::Translation2d{front, left};
  return *this;
}

SwerveModuleConfig& SwerveModuleConfig::WithLocation(wpi::units::meter_t distance,
                                                     wpi::units::degree_t angle) {
  m_location = wpi::math::Translation2d{distance, wpi::math::Rotation2d{angle}};
  return *this;
}

SwerveModuleConfig& SwerveModuleConfig::WithWheelRadius(wpi::units::meter_t radius) {
  m_wheelCircumference = radius * 2.0 * std::numbers::pi;
  if (m_driveMotor) {
    m_driveMotor->GetConfig().WithMechanismCircumference(*m_wheelCircumference);
  }
  return *this;
}

SwerveModuleConfig& SwerveModuleConfig::WithWheelDiameter(wpi::units::meter_t diameter) {
  m_wheelCircumference = diameter * std::numbers::pi;
  if (m_driveMotor) {
    m_driveMotor->GetConfig().WithMechanismCircumference(*m_wheelCircumference);
  }
  return *this;
}

SwerveModuleConfig& SwerveModuleConfig::WithMinimumVelocity(wpi::units::meters_per_second_t speed) {
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

std::optional<wpi::math::Translation2d> SwerveModuleConfig::GetLocation() const { return m_location; }

bool SwerveModuleConfig::GetStateOptimization() const { return m_stateOptimization; }

wpi::units::degree_t SwerveModuleConfig::GetAbsoluteEncoderAngle() const {
  if (m_absoluteEncoderSupplier) {
    auto rawAngle = (*m_absoluteEncoderSupplier)();
    auto geared = rawAngle * m_absoluteEncoderGearbox.GetInputToOutputConversionFactor();
    auto offset = m_absoluteEncoderOffset.value_or(wpi::units::degree_t{0});
    return geared - offset;
  }
  if (!m_azimuthMotor) return wpi::units::degree_t{0};
  return m_azimuthMotor->GetMechanismPosition();
}

double SwerveModuleConfig::GetCosineCompensatedVelocity(
    const wpi::math::SwerveModuleVelocity& desiredState) const {
  auto diff = desiredState.angle - wpi::math::Rotation2d{wpi::units::radian_t{GetAbsoluteEncoderAngle()}};
  double cosineScalar = diff.Cos();
  if (cosineScalar < 0.0) cosineScalar = 1.0;
  return desiredState.velocity.value() * cosineScalar;
}

wpi::math::SwerveModuleVelocity SwerveModuleConfig::GetOptimizedState(wpi::math::SwerveModuleVelocity state) const {
  if (m_minimumVelocity) {
    if (wpi::units::math::abs(state.velocity) <= *m_minimumVelocity) {
      state = wpi::math::SwerveModuleVelocity{wpi::units::meters_per_second_t{0},
                                     wpi::math::Rotation2d{wpi::units::radian_t{GetAbsoluteEncoderAngle()}}};
    }
  }
  if (m_stateOptimization) {
    state = state.Optimize(wpi::math::Rotation2d{wpi::units::radian_t{GetAbsoluteEncoderAngle()}});
  }
  if (m_cosineCompensation) {
    state.velocity = wpi::units::meters_per_second_t{GetCosineCompensatedVelocity(state)};
  }
  return state;
}

}  // namespace yams::mechanisms::config
