// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

#include "yams/mechanisms/swerve/SwerveDriveConfig.hpp"

#include <wpi/framework/RobotBase.hpp>
#include <wpi/units/math.hpp>

#include <cmath>
#include <numbers>
#include <stdexcept>
#include <utility>
#include <vector>

namespace yams::mechanisms::swerve {

// ---- Builder methods ---------------------------------------------------------

SwerveDriveConfig& SwerveDriveConfig::WithSubsystem(wpi::cmd::SubsystemBase* subsystem) {
  m_subsystem = subsystem;
  return *this;
}

SwerveDriveConfig& SwerveDriveConfig::WithModules(std::vector<SwerveModule*> modules) {
  m_modules = std::move(modules);
  return *this;
}

SwerveDriveConfig& SwerveDriveConfig::WithGyro(std::function<wpi::units::degree_t()> gyroSupplier) {
  m_gyroSupplier = std::move(gyroSupplier);
  return *this;
}

SwerveDriveConfig& SwerveDriveConfig::WithGyroVelocity(
    std::function<wpi::units::degrees_per_second_t()> angularVelocitySupplier) {
  m_gyroAngularVelocitySupplier = std::move(angularVelocitySupplier);
  return *this;
}

SwerveDriveConfig& SwerveDriveConfig::WithGyroOffset(wpi::units::degree_t offset) {
  m_gyroOffset = offset;
  return *this;
}

SwerveDriveConfig& SwerveDriveConfig::WithGyroInverted(bool inverted) {
  m_gyroInverted = inverted;
  return *this;
}

SwerveDriveConfig& SwerveDriveConfig::WithStartingPose(wpi::math::Pose2d pose) {
  m_initialPose = pose;
  return *this;
}

SwerveDriveConfig& SwerveDriveConfig::WithMaximumChassisSpeed(
    wpi::units::meters_per_second_t speed, wpi::units::degrees_per_second_t angularVelocity) {
  m_maximumChassisLinearVelocity = speed;
  m_maximumChassisAngularVelocity = angularVelocity;
  return *this;
}

SwerveDriveConfig& SwerveDriveConfig::WithMaximumModuleSpeed(wpi::units::meters_per_second_t speed) {
  m_maximumModuleLinearVelocity = speed;
  return *this;
}

SwerveDriveConfig& SwerveDriveConfig::WithCenterOfRotation(wpi::math::Translation2d center) {
  m_centerOfRotation = center;
  return *this;
}

SwerveDriveConfig& SwerveDriveConfig::WithCenterOfRotation(wpi::units::meter_t forward,
                                                           wpi::units::meter_t left) {
  m_centerOfRotation = wpi::math::Translation2d{forward, left};
  return *this;
}

SwerveDriveConfig& SwerveDriveConfig::WithDiscretizationTime(wpi::units::second_t dt) {
  m_discretizationSeconds = dt;
  return *this;
}

SwerveDriveConfig& SwerveDriveConfig::WithSimDiscretizationTime(wpi::units::second_t dt) {
  m_simDiscretizationSeconds = dt;
  return *this;
}

SwerveDriveConfig& SwerveDriveConfig::WithGyroAngularVelocityScaleFactor(double scaleFactor) {
  m_angularVelocityScaleFactor = scaleFactor;
  return *this;
}

SwerveDriveConfig& SwerveDriveConfig::WithSimGyroAngularVelocityScaleFactor(double scaleFactor) {
  m_simAngularVelocityScaleFactor = scaleFactor;
  return *this;
}

SwerveDriveConfig& SwerveDriveConfig::WithTranslationController(wpi::math::PIDController controller) {
  m_translationController = std::move(controller);
  return *this;
}

SwerveDriveConfig& SwerveDriveConfig::WithRotationController(wpi::math::PIDController controller) {
  controller.EnableContinuousInput(-std::numbers::pi, std::numbers::pi);
  m_rotationController = std::move(controller);
  return *this;
}

SwerveDriveConfig& SwerveDriveConfig::WithSimTranslationController(wpi::math::PIDController controller) {
  m_simTranslationController = std::move(controller);
  return *this;
}

SwerveDriveConfig& SwerveDriveConfig::WithSimRotationController(wpi::math::PIDController controller) {
  controller.EnableContinuousInput(-std::numbers::pi, std::numbers::pi);
  m_simRotationController = std::move(controller);
  return *this;
}

SwerveDriveConfig& SwerveDriveConfig::WithTelemetry(const std::string& name,
                                                     TelemetryVerbosity verbosity) {
  m_telemetryName = name;
  m_telemetryVerbosity = verbosity;
  return *this;
}

SwerveDriveConfig& SwerveDriveConfig::WithTelemetry(
    const std::string& name, telemetry::SwerveDriveTelemetryConfig telemetryConfig) {
  m_telemetryName = name;
  m_telemetryVerbosity.reset();
  m_specifiedTelemetryConfig = std::move(telemetryConfig);
  return *this;
}

// ---- Getters -----------------------------------------------------------------

const std::vector<SwerveModule*>& SwerveDriveConfig::GetModules() const { return m_modules; }

wpi::cmd::SubsystemBase* SwerveDriveConfig::GetSubsystem() const { return m_subsystem; }

wpi::math::Pose2d SwerveDriveConfig::GetInitialPose() const { return m_initialPose; }

std::optional<wpi::units::meters_per_second_t> SwerveDriveConfig::GetMaximumChassisLinearVelocity()
    const {
  return m_maximumChassisLinearVelocity;
}

std::optional<wpi::units::degrees_per_second_t> SwerveDriveConfig::GetMaximumChassisAngularVelocity()
    const {
  return m_maximumChassisAngularVelocity;
}

std::optional<wpi::units::meters_per_second_t> SwerveDriveConfig::GetMaximumModuleLinearVelocity()
    const {
  return m_maximumModuleLinearVelocity;
}

std::optional<wpi::math::Translation2d> SwerveDriveConfig::GetCenterOfRotation() const {
  return m_centerOfRotation;
}

std::optional<SwerveDriveConfig::TelemetryVerbosity> SwerveDriveConfig::GetTelemetryVerbosity()
    const {
  return m_telemetryVerbosity;
}

const std::string& SwerveDriveConfig::GetTelemetryName() const { return m_telemetryName; }

std::optional<telemetry::SwerveDriveTelemetryConfig>
SwerveDriveConfig::GetSwerveDriveTelemetryConfig() {
  if (!m_specifiedTelemetryConfig) return std::nullopt;
  auto result = std::move(m_specifiedTelemetryConfig);
  m_specifiedTelemetryConfig.reset();
  return result;
}

wpi::units::degree_t SwerveDriveConfig::GetGyroOffset() const {
  return m_gyroOffset.value_or(wpi::units::degree_t{0});
}

wpi::units::degree_t SwerveDriveConfig::GetGyroAngle() const {
  if (!m_gyroSupplier) {
    throw std::runtime_error(
        "Gyro supplier is not set! Please use .WithGyro() to set the gyro supplier!");
  }
  auto rawAngle = (*m_gyroSupplier)();
  auto angle = m_gyroInverted ? -rawAngle : rawAngle;
  return angle - GetGyroOffset();
}

wpi::math::ChassisVelocities SwerveDriveConfig::AngularVelocitySkewCorrection(
    wpi::math::ChassisVelocities robotRelativeVelocity) const {
  double scaleFactor =
      wpi::RobotBase::IsSimulation()
          ? m_simAngularVelocityScaleFactor.value_or(m_angularVelocityScaleFactor.value_or(0.0))
          : m_angularVelocityScaleFactor.value_or(0.0);

  auto angularVelocityRad = wpi::units::radian_t{
      wpi::units::radians_per_second_t{(*m_gyroAngularVelocitySupplier)()}.value() * scaleFactor};
  auto angularVelocityRotation = wpi::math::Rotation2d{angularVelocityRad};

  if (angularVelocityRotation.Radians() == wpi::units::radian_t{0}) {
    return robotRelativeVelocity;
  }

  auto gyroRotation = wpi::math::Rotation2d{wpi::units::radian_t{GetGyroAngle()}};
  auto fieldRelativeVelocity =
      (robotRelativeVelocity).ToFieldRelative(gyroRotation);
  return (fieldRelativeVelocity).ToRobotRelative(gyroRotation + angularVelocityRotation);
}

wpi::math::ChassisVelocities SwerveDriveConfig::OptimizeRobotRelativeChassisSpeeds(
    wpi::math::ChassisVelocities speeds) const {
  if (m_angularVelocityScaleFactor && m_gyroAngularVelocitySupplier) {
    speeds = AngularVelocitySkewCorrection(speeds);
  }
  if (m_discretizationSeconds) {
    auto dt = wpi::RobotBase::IsSimulation()
                  ? m_simDiscretizationSeconds.value_or(*m_discretizationSeconds)
                  : *m_discretizationSeconds;
    speeds = speeds.Discretize(dt);
  }
  return speeds;
}

wpi::math::PIDController& SwerveDriveConfig::GetTranslationPID() {
  if (wpi::RobotBase::IsSimulation() && m_simTranslationController) {
    return *m_simTranslationController;
  }
  if (!m_translationController) throw std::logic_error("Translation PID controller not set.");
  return *m_translationController;
}

wpi::math::PIDController& SwerveDriveConfig::GetRotationPID() {
  if (wpi::RobotBase::IsSimulation() && m_simRotationController) {
    return *m_simRotationController;
  }
  if (!m_rotationController) throw std::logic_error("Rotation PID controller not set.");
  return *m_rotationController;
}

wpi::math::Translation2d SwerveDriveConfig::CubeTranslation(wpi::math::Translation2d translation) {
  if (std::hypot(translation.X().value(), translation.Y().value()) <= 1.0e-6) {
    return translation;
  }
  return wpi::math::Translation2d{wpi::units::meter_t{std::pow(translation.Norm().value(), 3)},
                            translation.Angle()};
}

wpi::math::Translation2d SwerveDriveConfig::ScaleTranslation(wpi::math::Translation2d translation,
                                                       double scalar) {
  if (std::hypot(translation.X().value(), translation.Y().value()) <= 1.0e-6) {
    return translation;
  }
  return wpi::math::Translation2d{translation.Norm() * scalar, translation.Angle()};
}

}  // namespace yams::mechanisms::swerve
