// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

#include "yams/mechanisms/swerve/SwerveDriveConfig.hpp"

#include <frc/RobotBase.h>
#include <units/math.h>

#include <cmath>
#include <numbers>
#include <stdexcept>
#include <utility>
#include <vector>

namespace yams::mechanisms::swerve {

// ---- Builder methods ---------------------------------------------------------

SwerveDriveConfig& SwerveDriveConfig::WithSubsystem(frc2::SubsystemBase* subsystem) {
  m_subsystem = subsystem;
  return *this;
}

SwerveDriveConfig& SwerveDriveConfig::WithModules(std::vector<SwerveModule*> modules) {
  m_modules = std::move(modules);
  return *this;
}

SwerveDriveConfig& SwerveDriveConfig::WithGyro(std::function<units::degree_t()> gyroSupplier) {
  m_gyroSupplier = std::move(gyroSupplier);
  return *this;
}

SwerveDriveConfig& SwerveDriveConfig::WithGyroVelocity(
    std::function<units::degrees_per_second_t()> angularVelocitySupplier) {
  m_gyroAngularVelocitySupplier = std::move(angularVelocitySupplier);
  return *this;
}

SwerveDriveConfig& SwerveDriveConfig::WithGyroOffset(units::degree_t offset) {
  m_gyroOffset = offset;
  return *this;
}

SwerveDriveConfig& SwerveDriveConfig::WithGyroInverted(bool inverted) {
  m_gyroInverted = inverted;
  return *this;
}

SwerveDriveConfig& SwerveDriveConfig::WithStartingPose(frc::Pose2d pose) {
  m_initialPose = pose;
  return *this;
}

SwerveDriveConfig& SwerveDriveConfig::WithMaximumChassisSpeed(
    units::meters_per_second_t speed, units::degrees_per_second_t angularVelocity) {
  m_maximumChassisLinearVelocity = speed;
  m_maximumChassisAngularVelocity = angularVelocity;
  return *this;
}

SwerveDriveConfig& SwerveDriveConfig::WithMaximumModuleSpeed(units::meters_per_second_t speed) {
  m_maximumModuleLinearVelocity = speed;
  return *this;
}

SwerveDriveConfig& SwerveDriveConfig::WithCenterOfRotation(frc::Translation2d center) {
  m_centerOfRotation = center;
  return *this;
}

SwerveDriveConfig& SwerveDriveConfig::WithCenterOfRotation(units::meter_t forward,
                                                           units::meter_t left) {
  m_centerOfRotation = frc::Translation2d{forward, left};
  return *this;
}

SwerveDriveConfig& SwerveDriveConfig::WithDiscretizationTime(units::second_t dt) {
  m_discretizationSeconds = dt;
  return *this;
}

SwerveDriveConfig& SwerveDriveConfig::WithSimDiscretizationTime(units::second_t dt) {
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

SwerveDriveConfig& SwerveDriveConfig::WithTranslationController(frc::PIDController controller) {
  m_translationController = std::move(controller);
  return *this;
}

SwerveDriveConfig& SwerveDriveConfig::WithRotationController(frc::PIDController controller) {
  controller.EnableContinuousInput(-std::numbers::pi, std::numbers::pi);
  m_rotationController = std::move(controller);
  return *this;
}

SwerveDriveConfig& SwerveDriveConfig::WithSimTranslationController(frc::PIDController controller) {
  m_simTranslationController = std::move(controller);
  return *this;
}

SwerveDriveConfig& SwerveDriveConfig::WithSimRotationController(frc::PIDController controller) {
  controller.EnableContinuousInput(-std::numbers::pi, std::numbers::pi);
  m_simRotationController = std::move(controller);
  return *this;
}

SwerveDriveConfig& SwerveDriveConfig::WithTelemetry(TelemetryVerbosity verbosity) {
  m_telemetryVerbosity = verbosity;
  return *this;
}

// ---- Getters -----------------------------------------------------------------

const std::vector<SwerveModule*>& SwerveDriveConfig::GetModules() const { return m_modules; }

frc2::SubsystemBase* SwerveDriveConfig::GetSubsystem() const { return m_subsystem; }

frc::Pose2d SwerveDriveConfig::GetInitialPose() const { return m_initialPose; }

std::optional<units::meters_per_second_t> SwerveDriveConfig::GetMaximumChassisLinearVelocity()
    const {
  return m_maximumChassisLinearVelocity;
}

std::optional<units::degrees_per_second_t> SwerveDriveConfig::GetMaximumChassisAngularVelocity()
    const {
  return m_maximumChassisAngularVelocity;
}

std::optional<units::meters_per_second_t> SwerveDriveConfig::GetMaximumModuleLinearVelocity()
    const {
  return m_maximumModuleLinearVelocity;
}

std::optional<frc::Translation2d> SwerveDriveConfig::GetCenterOfRotation() const {
  return m_centerOfRotation;
}

std::optional<SwerveDriveConfig::TelemetryVerbosity> SwerveDriveConfig::GetTelemetryVerbosity()
    const {
  return m_telemetryVerbosity;
}

units::degree_t SwerveDriveConfig::GetGyroOffset() const {
  return m_gyroOffset.value_or(units::degree_t{0});
}

units::degree_t SwerveDriveConfig::GetGyroAngle() const {
  if (!m_gyroSupplier) {
    throw std::runtime_error(
        "Gyro supplier is not set! Please use .WithGyro() to set the gyro supplier!");
  }
  auto rawAngle = (*m_gyroSupplier)();
  auto angle = m_gyroInverted ? -rawAngle : rawAngle;
  return angle - GetGyroOffset();
}

frc::ChassisSpeeds SwerveDriveConfig::AngularVelocitySkewCorrection(
    frc::ChassisSpeeds robotRelativeVelocity) const {
  double scaleFactor =
      frc::RobotBase::IsSimulation()
          ? m_simAngularVelocityScaleFactor.value_or(m_angularVelocityScaleFactor.value_or(0.0))
          : m_angularVelocityScaleFactor.value_or(0.0);

  auto angularVelocityRad = units::radian_t{
      units::radians_per_second_t{(*m_gyroAngularVelocitySupplier)()}.value() * scaleFactor};
  auto angularVelocityRotation = frc::Rotation2d{angularVelocityRad};

  if (angularVelocityRotation.Radians() == units::radian_t{0}) {
    return robotRelativeVelocity;
  }

  auto gyroRotation = frc::Rotation2d{units::radian_t{GetGyroAngle()}};
  auto fieldRelativeVelocity =
      frc::ChassisSpeeds::FromRobotRelativeSpeeds(robotRelativeVelocity, gyroRotation);
  return frc::ChassisSpeeds::FromFieldRelativeSpeeds(fieldRelativeVelocity,
                                                     gyroRotation + angularVelocityRotation);
}

frc::ChassisSpeeds SwerveDriveConfig::OptimizeRobotRelativeChassisSpeeds(
    frc::ChassisSpeeds speeds) const {
  if (m_angularVelocityScaleFactor && m_gyroAngularVelocitySupplier) {
    speeds = AngularVelocitySkewCorrection(speeds);
  }
  if (m_discretizationSeconds) {
    auto dt = frc::RobotBase::IsSimulation()
                  ? m_simDiscretizationSeconds.value_or(*m_discretizationSeconds)
                  : *m_discretizationSeconds;
    speeds = frc::ChassisSpeeds::Discretize(speeds, dt);
  }
  return speeds;
}

frc::PIDController& SwerveDriveConfig::GetTranslationPID() {
  if (frc::RobotBase::IsSimulation() && m_simTranslationController) {
    return *m_simTranslationController;
  }
  if (!m_translationController) throw std::logic_error("Translation PID controller not set.");
  return *m_translationController;
}

frc::PIDController& SwerveDriveConfig::GetRotationPID() {
  if (frc::RobotBase::IsSimulation() && m_simRotationController) {
    return *m_simRotationController;
  }
  if (!m_rotationController) throw std::logic_error("Rotation PID controller not set.");
  return *m_rotationController;
}

frc::Translation2d SwerveDriveConfig::CubeTranslation(frc::Translation2d translation) {
  if (std::hypot(translation.X().value(), translation.Y().value()) <= 1.0e-6) {
    return translation;
  }
  return frc::Translation2d{units::meter_t{std::pow(translation.Norm().value(), 3)},
                            translation.Angle()};
}

frc::Translation2d SwerveDriveConfig::ScaleTranslation(frc::Translation2d translation,
                                                       double scalar) {
  if (std::hypot(translation.X().value(), translation.Y().value()) <= 1.0e-6) {
    return translation;
  }
  return frc::Translation2d{translation.Norm() * scalar, translation.Angle()};
}

}  // namespace yams::mechanisms::swerve
