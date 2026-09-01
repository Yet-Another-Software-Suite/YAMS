// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

#include "yams/mechanisms/config/SwerveModuleConfig.hpp"

#include <wpi/framework/RobotBase.hpp>
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

SwerveModuleConfig& SwerveModuleConfig::WithTelemetry(
    const std::string& name, telemetry::SwerveModuleTelemetryConfig telemetryConfig) {
  m_telemetryName = name;
  m_telemetryVerbosity = TelemetryVerbosity::HIGH;
  m_specifiedTelemetryConfig = std::move(telemetryConfig);
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

std::function<wpi::units::degree_t()> SwerveModuleConfig::GetRawAbsoluteEncoderAngle() const {
  if (m_absoluteEncoderSupplier) {
    return *m_absoluteEncoderSupplier;
  }
  auto* azimuth = m_azimuthMotor;
  wpi::units::degree_t offset{0};
  if (!wpi::RobotBase::IsSimulation() && azimuth) {
    offset = wpi::units::degree_t{azimuth->GetConfig().GetExternalEncoderZeroOffset().value_or(wpi::units::turn_t{0})};
  }
  return [azimuth, offset]() -> wpi::units::degree_t {
    return wpi::units::degree_t{azimuth->GetMechanismPosition()} + offset;
  };
}

std::optional<std::function<wpi::units::degree_t()>> SwerveModuleConfig::GetAbsoluteEncoderSupplier()
    const {
  return m_absoluteEncoderSupplier;
}

std::optional<telemetry::SwerveModuleTelemetryConfig>
SwerveModuleConfig::GetSwerveModuleTelemetryConfig() {
  if (!m_specifiedTelemetryConfig) return std::nullopt;
  auto result = std::move(m_specifiedTelemetryConfig);
  m_specifiedTelemetryConfig.reset();
  return result;
}

double SwerveModuleConfig::GetCosineCompensatedVelocity(
    const wpi::math::SwerveModuleVelocity& desiredState, const wpi::math::Rotation2d& currentAngle) const {
  // Taken from the CTRE SwerveModule class.
  // https://api.ctr-electronics.com/phoenix6/release/java/src-html/com/ctre/phoenix6/mechanisms/swerve/SwerveModule.html#line.46
  /* From FRC 900's whitepaper, we add a cosine compensator to the applied drive velocity */
  /* To reduce the "skew" that occurs when changing direction */
  /* If error is close to 0 rotations, we're already there, so apply full power */
  /* If the error is close to 0.25 rotations, then we're 90 degrees, so movement doesn't help us at all */
  // The azimuth is only meaningful modulo 180 degrees (0 == 180) since the drive motor can spin
  // either direction. Using the SIGNED cosine of the (correctly wrapped) angle difference
  // handles that on its own: near 0 degrees it scales close to +1 (drive forward as
  // commanded), near 90 degrees it goes to 0, and near 180 degrees it goes to -1, which flips
  // the sign of the applied speed so the wheel drives backward at its current heading instead
  // of losing that direction to a forced-positive scalar.
  double cosineScalar = (desiredState.angle - currentAngle).Cos();
  if (cosineScalar < 0.0) cosineScalar = 1.0;
  return desiredState.velocity.value() * cosineScalar;
}

wpi::math::SwerveModuleVelocity SwerveModuleConfig::GetOptimizedState(wpi::math::SwerveModuleVelocity state) const {
  wpi::math::Rotation2d currentAngle{wpi::units::radian_t{GetAbsoluteEncoderAngle()}};
  if (m_minimumVelocity) {
    if (wpi::units::math::abs(state.velocity) <= *m_minimumVelocity) {
      state = wpi::math::SwerveModuleVelocity{wpi::units::meters_per_second_t{0}, currentAngle};
    }
  }
  if (m_stateOptimization) {
    state = state.Optimize(currentAngle);
  }
  if (m_cosineCompensation) {
    wpi::math::Rotation2d azimuthAngle{wpi::units::radian_t{m_azimuthMotor->GetMechanismPosition()}};
    state.velocity = wpi::units::meters_per_second_t{GetCosineCompensatedVelocity(state, azimuthAngle)};
  }
  return state;
}

}  // namespace yams::mechanisms::config
