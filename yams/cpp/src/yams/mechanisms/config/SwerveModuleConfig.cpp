// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

#include "yams/mechanisms/config/SwerveModuleConfig.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>
#include <stdexcept>
#include <string>
#include <utility>
#include <wpi/framework/RobotBase.hpp>
#include <wpi/math/geometry/Rotation2d.hpp>
#include <wpi/units/math.hpp>

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

SwerveModuleConfig& SwerveModuleConfig::WithLocation(wpi::units::meter_t front,
                                                     wpi::units::meter_t left) {
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

SwerveModuleConfig& SwerveModuleConfig::WithVelocityDeadband(wpi::units::meters_per_second_t speed) {
  m_velocityDeadband = speed;
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

std::optional<wpi::math::Translation2d> SwerveModuleConfig::GetLocation() const {
  return m_location;
}

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
    offset = wpi::units::degree_t{
        azimuth->GetConfig().GetExternalEncoderZeroOffset().value_or(wpi::units::turn_t{0})};
  }
  return [azimuth, offset]() -> wpi::units::degree_t {
    return wpi::units::degree_t{azimuth->GetMechanismPosition()} + offset;
  };
}

std::optional<std::function<wpi::units::degree_t()>>
SwerveModuleConfig::GetAbsoluteEncoderSupplier() const {
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
    const wpi::math::SwerveModuleVelocity& desiredState,
    const wpi::math::Rotation2d& currentAngle) const {
  // Taken from the CTRE SwerveModule class.
  // https://api.ctr-electronics.com/phoenix6/release/java/src-html/com/ctre/phoenix6/mechanisms/swerve/SwerveModule.html#line.46
  /* From FRC 900's whitepaper, we add a cosine compensator to the applied drive velocity */
  /* To reduce the "skew" that occurs when changing direction */
  /* If error is close to 0 rotations, we're already there, so apply full power */
  /* If the error is close to 0.25 rotations, then we're 90 degrees, so movement doesn't help us at
   * all */
  // Optimization keeps the wheel within 90 degrees of its target except while it holds the
  // orientation it was last sent to (see Optimize), where the wheel is still turning toward it and
  // driving would push the robot the wrong way, so apply nothing there.
  double cosineScalar = std::max((desiredState.angle - currentAngle).Cos(), 0.0);
  return desiredState.velocity.value() * cosineScalar;
}

wpi::math::Rotation2d SwerveModuleConfig::GetAzimuthAngle() const {
  if (m_azimuthMotor) {
    return wpi::math::Rotation2d{wpi::units::radian_t{m_azimuthMotor->GetMechanismPosition()}};
  }
  return wpi::math::Rotation2d{wpi::units::radian_t{GetAbsoluteEncoderAngle()}};
}

wpi::math::SwerveModuleVelocity SwerveModuleConfig::Optimize(
    wpi::math::SwerveModuleVelocity desired, const wpi::math::Rotation2d& currentAngle,
    const std::optional<wpi::math::Rotation2d>& lastCommanded) {
  wpi::math::SwerveModuleVelocity state = desired.Optimize(currentAngle);
  if (!lastCommanded || wpi::units::math::abs((state.angle - *lastCommanded).Degrees()) <=
                            wpi::units::degree_t{90}) {
    return state;
  }
  wpi::math::SwerveModuleVelocity lastOrientation{
      -state.velocity, state.angle.RotateBy(wpi::math::Rotation2d{wpi::units::degree_t{180}})};
  return wpi::units::math::abs((lastOrientation.angle - currentAngle).Degrees()) <
                 wpi::units::degree_t{90 + 30}
             ? lastOrientation
             : state;
}

wpi::math::SwerveModuleVelocity SwerveModuleConfig::GetOptimizedState(
    wpi::math::SwerveModuleVelocity state) const {
  wpi::math::Rotation2d currentAngle = GetAzimuthAngle();
  if (m_velocityDeadband && wpi::units::math::abs(state.velocity) <= *m_velocityDeadband) {
    // Too slow to steer by: hold the wheel where it is.
    state = wpi::math::SwerveModuleVelocity{wpi::units::meters_per_second_t{0}, currentAngle};
    m_lastCommandedAngle = currentAngle;
  } else if (m_stateOptimization) {
    state = Optimize(state, currentAngle, m_lastCommandedAngle);
    m_lastCommandedAngle = state.angle;
  }
  if (m_cosineCompensation) {
    state.velocity =
        wpi::units::meters_per_second_t{GetCosineCompensatedVelocity(state, currentAngle)};
  }
  return state;
}

wpi::math::SwerveModuleVelocity SwerveModuleConfig::GetLockedState(
    wpi::math::Rotation2d angle) const {
  wpi::math::SwerveModuleVelocity state{wpi::units::meters_per_second_t{0}, angle};
  if (m_stateOptimization) {
    state = Optimize(state, GetAzimuthAngle(), m_lastCommandedAngle);
  }
  m_lastCommandedAngle = state.angle;
  return state;
}

}  // namespace yams::mechanisms::config
