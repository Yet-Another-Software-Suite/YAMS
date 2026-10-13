// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

#include "yams/mechanisms/swerve/SwerveDriveConfig.hpp"

#include <cmath>
#include <numbers>
#include <stdexcept>
#include <utility>
#include <vector>
#include <wpi/framework/RobotBase.hpp>
#include <wpi/math/util/MathUtil.hpp>
#include <wpi/units/math.hpp>

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
  if (!gyroSupplier) {
    m_gyroSupplier.reset();
    m_continuousHeadingRead = false;
    return *this;
  }
  return WithGyro(std::function<wpi::math::Rotation3d()>{[supplier = std::move(gyroSupplier)] {
    return wpi::math::Rotation3d{wpi::units::radian_t{0}, wpi::units::radian_t{0}, wpi::units::radian_t{supplier()}};
  }});
}

SwerveDriveConfig& SwerveDriveConfig::WithGyro(std::function<wpi::math::Rotation3d()> gyroSupplier) {
  if (gyroSupplier) {
    m_gyroSupplier = std::move(gyroSupplier);
  } else {
    m_gyroSupplier.reset();
  }
  m_continuousHeadingRead = false;
  return *this;
}

SwerveDriveConfig& SwerveDriveConfig::WithGyroHeadingAxis(config::GyroAxis axis) {
  m_gyroHeadingAxis = axis;
  m_continuousHeadingRead = false;
  return *this;
}

SwerveDriveConfig& SwerveDriveConfig::WithAntiTipping(
    wpi::units::meters_per_second_t kP, wpi::units::degree_t tippingThreshold,
    wpi::units::meters_per_second_t maxCorrectionSpeed) {
  if (kP.value() <= 0) {
    throw std::invalid_argument("Anti-tipping kP must be positive, but was " +
                                std::to_string(kP.value()) + " m/s");
  }
  if (maxCorrectionSpeed.value() <= 0) {
    throw std::invalid_argument("Anti-tipping maximum correction speed must be positive, but was " +
                                std::to_string(maxCorrectionSpeed.value()) + " m/s");
  }
  const wpi::units::radian_t threshold{tippingThreshold};
  if (threshold.value() <= 0 || threshold.value() >= std::numbers::pi / 2) {
    throw std::invalid_argument(
        "Anti-tipping threshold must be between 0 and 90 degrees, but was " +
        std::to_string(tippingThreshold.value()) + " degrees");
  }
  return WithAntiTipping(wpi::math::AntiTipping{
      wpi::units::unit_t<wpi::math::AntiTipping::kp_unit>{kP.value()}, threshold,
      maxCorrectionSpeed});
}

SwerveDriveConfig& SwerveDriveConfig::WithAntiTipping(
    std::optional<wpi::math::AntiTipping> antiTipping) {
  m_antiTipping = std::move(antiTipping);
  return *this;
}

SwerveDriveConfig& SwerveDriveConfig::WithModuleStateOptimization(bool enabled) {
  m_moduleStateOptimization = enabled;
  for (auto* module : m_modules) {
    if (module) module->GetMutableConfig().WithOptimization(enabled);
  }
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

SwerveDriveConfig& SwerveDriveConfig::WithMaximumModuleSpeed(
    wpi::units::meters_per_second_t speed) {
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

SwerveDriveConfig& SwerveDriveConfig::WithTranslationController(
    wpi::math::PIDController controller) {
  m_translationController = std::move(controller);
  return *this;
}

SwerveDriveConfig& SwerveDriveConfig::WithRotationController(wpi::math::PIDController controller) {
  controller.EnableContinuousInput(-std::numbers::pi, std::numbers::pi);
  m_rotationController = std::move(controller);
  return *this;
}

SwerveDriveConfig& SwerveDriveConfig::WithSimTranslationController(
    wpi::math::PIDController controller) {
  m_simTranslationController = std::move(controller);
  return *this;
}

SwerveDriveConfig& SwerveDriveConfig::WithSimRotationController(
    wpi::math::PIDController controller) {
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

std::optional<wpi::units::degrees_per_second_t>
SwerveDriveConfig::GetMaximumChassisAngularVelocity() const {
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
  return wpi::units::degree_t{GetGyroRotation3d().Z()};
}

wpi::math::Rotation3d SwerveDriveConfig::GetGyroRotation3d() const {
  if (!m_gyroSupplier) {
    throw std::runtime_error(
        "Gyro supplier is not set! Please use .WithGyro() to set the gyro supplier!");
  }
  auto attitude = config::ToRobotAttitude(m_gyroHeadingAxis, (*m_gyroSupplier)());
  if (m_gyroInverted) {
    attitude = wpi::math::Rotation3d{attitude.X(), attitude.Y(), -attitude.Z()};
  }
  return attitude.RotateBy(
      wpi::math::Rotation3d{wpi::units::radian_t{0}, wpi::units::radian_t{0}, -wpi::units::radian_t{GetGyroOffset()}});
}

wpi::math::ChassisVelocities SwerveDriveConfig::GetAntiTippingCorrection() const {
  if (!m_antiTipping || !m_gyroSupplier) {
    return wpi::math::ChassisVelocities{};
  }
  const auto attitude = GetGyroRotation3d();
  return m_antiTipping->Calculate(attitude).ToRobotRelative(wpi::math::Rotation2d{attitude.Z()});
}

std::optional<bool> SwerveDriveConfig::GetModuleStateOptimization() const {
  return m_moduleStateOptimization;
}

std::optional<wpi::units::second_t> SwerveDriveConfig::GetDiscretization() const {
  return wpi::RobotBase::IsSimulation() ? m_simDiscretizationSeconds : m_discretizationSeconds;
}

wpi::math::ChassisVelocities SwerveDriveConfig::AngularVelocitySkewCorrection(
    wpi::math::ChassisVelocities robotRelativeVelocity) const {
  double scaleFactor =
      wpi::RobotBase::IsSimulation()
          ? m_simAngularVelocityScaleFactor.value_or(m_angularVelocityScaleFactor.value_or(0.0))
          : m_angularVelocityScaleFactor.value_or(0.0);

  wpi::units::radians_per_second_t gyroAngularVelocity{0};
  if (m_gyroAngularVelocitySupplier) {
    gyroAngularVelocity = (*m_gyroAngularVelocitySupplier)();
  } else {
    if (!m_noGyroAngularVelocitySupplierAlert) {
      m_noGyroAngularVelocitySupplierAlert.emplace(
          "YAMS", GetTelemetryName() + "/NoGyroAngularVelocitySupplier",
          GetTelemetryName() +
              " has an angular velocity scale factor configured but no gyro angular velocity "
              "supplier (see SwerveDriveConfig::WithGyroVelocity); deriving it from the gyro "
              "angle instead.",
          wpi::util::Alert::Level::LOW);
      m_noGyroAngularVelocitySupplierAlert->Set(true);
    }
    // Differentiate the heading kept continuous: wrapped, it jumps a whole rotation when the robot
    // turns past half a rotation, and the derived angular velocity would spike.
    const double wrappedHeadingRadians = GetGyroRotation3d().Z().value();
    m_continuousHeadingRadians =
        m_continuousHeadingRead
            ? m_continuousHeadingRadians +
                  wpi::math::AngleModulus(
                      wpi::units::radian_t{wrappedHeadingRadians - m_continuousHeadingRadians})
                      .value()
            : wrappedHeadingRadians;
    m_continuousHeadingRead = true;
    gyroAngularVelocity = wpi::units::radians_per_second_t{
        m_gyroAngularVelocityFilter.Derivative(m_continuousHeadingRadians)};
  }
  auto angularVelocityRad = wpi::units::radian_t{gyroAngularVelocity.value() * scaleFactor};
  auto angularVelocityRotation = wpi::math::Rotation2d{angularVelocityRad};

  if (angularVelocityRotation.Radians() == wpi::units::radian_t{0}) {
    return robotRelativeVelocity;
  }

  auto gyroRotation = GetGyroRotation3d().ToRotation2d();
  auto fieldRelativeVelocity = (robotRelativeVelocity).ToFieldRelative(gyroRotation);
  return (fieldRelativeVelocity).ToRobotRelative(gyroRotation + angularVelocityRotation);
}

wpi::math::ChassisVelocities SwerveDriveConfig::OptimizeRobotRelativeChassisSpeeds(
    wpi::math::ChassisVelocities speeds) const {
  if (m_antiTipping) {
    speeds = speeds + GetAntiTippingCorrection();
  }
  if (m_angularVelocityScaleFactor) {
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

std::optional<std::reference_wrapper<wpi::math::PIDController>>
SwerveDriveConfig::GetTranslationPID() {
  if (wpi::RobotBase::IsSimulation() && m_simTranslationController) {
    return std::ref(*m_simTranslationController);
  }
  if (m_translationController) {
    return std::ref(*m_translationController);
  }
  return std::nullopt;
}

std::optional<std::reference_wrapper<wpi::math::PIDController>>
SwerveDriveConfig::GetRotationPID() {
  if (wpi::RobotBase::IsSimulation() && m_simRotationController) {
    return std::ref(*m_simRotationController);
  }
  if (m_rotationController) {
    return std::ref(*m_rotationController);
  }
  return std::nullopt;
}

wpi::math::Translation2d SwerveDriveConfig::CubeTranslation(wpi::math::Translation2d translation) {
  if (std::hypot(translation.X().value(), translation.Y().value()) <= 1.0e-6) {
    return translation;
  }
  return wpi::math::Translation2d{wpi::units::meter_t{std::pow(translation.Norm().value(), 3)},
                                  translation.Angle().value()};
}

wpi::math::Translation2d SwerveDriveConfig::ScaleTranslation(wpi::math::Translation2d translation,
                                                             double scalar) {
  if (std::hypot(translation.X().value(), translation.Y().value()) <= 1.0e-6) {
    return translation;
  }
  return wpi::math::Translation2d{translation.Norm() * scalar, translation.Angle().value()};
}

}  // namespace yams::mechanisms::swerve
