// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

#pragma once

#include <cmath>
#include <functional>
#include <iostream>
#include <memory>
#include <numbers>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <wpi/driverstation/Alliance.hpp>
#include <wpi/driverstation/MatchState.hpp>
#include <wpi/math/geometry/Pose2d.hpp>
#include <wpi/math/geometry/Rotation2d.hpp>
#include <wpi/math/geometry/Translation2d.hpp>
#include <wpi/math/kinematics/ChassisVelocities.hpp>
#include <wpi/math/util/MathUtil.hpp>
#include <wpi/units/angle.hpp>
#include <wpi/units/angular_velocity.hpp>
#include <wpi/units/length.hpp>
#include <wpi/units/velocity.hpp>

#include "yams/exceptions.hpp"
#include "yams/mechanisms/swerve/SwerveDrive.hpp"
#include "yams/mechanisms/swerve/SwerveDriveConfig.hpp"

namespace yams::mechanisms::swerve::utility {

template <size_t NumModules>
class SwerveInputStreamTelemetry;

template <size_t NumModules = 4>
class SwerveInputStream {
 public:
  using TelemetryVerbosity = SwerveDriveConfig::TelemetryVerbosity;

  static SwerveInputStream Of(SwerveDrive<NumModules>& drive, std::function<double()> x,
                              std::function<double()> y) {
    return SwerveInputStream{drive, std::move(x), std::move(y)};
  }

  SwerveInputStream(SwerveDrive<NumModules>& drive, std::function<double()> x,
                    std::function<double()> y, std::function<double()> rot)
      : SwerveInputStream{drive, std::move(x), std::move(y)} {
    m_controllerOmega = std::move(rot);
  }

  SwerveInputStream(SwerveDrive<NumModules>& drive, std::function<double()> x,
                    std::function<double()> y, std::function<double()> headingX,
                    std::function<double()> headingY)
      : SwerveInputStream{drive, std::move(x), std::move(y)} {
    WithControllerHeadingAxis(std::move(headingX), std::move(headingY));
  }

  /**
   * Copy this stream. Unlike a plain copy, the clone has no telemetry, so it does not publish under
   * this stream's name.
   */
  SwerveInputStream Clone() const {
    SwerveInputStream clone = *this;
    clone.m_telemetry.settings.reset();
    return clone;
  }

  /**
   * Publish this stream's telemetry under SwerveInputStream/<name>, updated every time the stream is
   * read. At HIGH the stream is also live tunable: see SwerveInputStreamTelemetry. Replaces any
   * telemetry this stream already had; NONE turns it off.
   *
   * The telemetry is created when the stream is first read or GetTelemetry() is called, so copies
   * made while building the stream do not publish. A copy of a stream with telemetry publishes under
   * the same name once read; use Clone() for a copy without telemetry.
   *
   * @param name      Name of the stream in NetworkTables (e.g., "drive").
   * @param verbosity TelemetryVerbosity to publish at.
   * @return this, for chaining.
   */
  SwerveInputStream& WithTelemetry(std::string name, TelemetryVerbosity verbosity) {
    m_telemetry.telemetry.reset();
    if (verbosity == TelemetryVerbosity::NONE) {
      m_telemetry.settings.reset();
    } else {
      m_telemetry.settings.emplace(std::move(name), verbosity);
    }
    return *this;
  }

  /**
   * Get this stream's telemetry, created by WithTelemetry(name, verbosity).
   *
   * @return The SwerveInputStreamTelemetry, or empty if telemetry is not enabled.
   */
  std::optional<std::reference_wrapper<SwerveInputStreamTelemetry<NumModules>>> GetTelemetry() {
    if (!m_telemetry.settings) {
      return std::nullopt;
    }
    if (!m_telemetry.telemetry) {
      m_telemetry.telemetry = std::make_unique<SwerveInputStreamTelemetry<NumModules>>(
          *this, m_telemetry.settings->first, m_telemetry.settings->second);
    }
    return std::ref(*m_telemetry.telemetry);
  }

  std::string GetCurrentModeName() const {
    switch (m_currentMode) {
      case SwerveInputMode::TRANSLATION_ONLY:
        return "TRANSLATION_ONLY";
      case SwerveInputMode::ANGULAR_VELOCITY:
        return "ANGULAR_VELOCITY";
      case SwerveInputMode::HEADING:
        return "HEADING";
      case SwerveInputMode::AIM:
        return "AIM";
    }
    return "UNKNOWN";
  }

  double GetAxisDeadband() const { return m_axisDeadband.value_or(0.0); }
  void SetAxisDeadband(double deadband) {
    m_axisDeadband = (deadband == 0.0) ? std::nullopt : std::optional<double>{deadband};
  }

  double GetTranslationAxisScale() const { return m_translationAxisScale.value_or(1.0); }
  void SetTranslationAxisScale(double scale) {
    m_translationAxisScale = (scale == 0.0) ? std::nullopt : std::optional<double>{scale};
  }

  double GetOmegaAxisScale() const { return m_omegaAxisScale.value_or(1.0); }
  void SetOmegaAxisScale(double scale) {
    m_omegaAxisScale = (scale == 0.0) ? std::nullopt : std::optional<double>{scale};
  }

  wpi::units::meters_per_second_t GetMaximumChassisLinearVelocity() const {
    return m_maximumChassisLinearVelocity;
  }
  void SetMaximumChassisLinearVelocity(wpi::units::meters_per_second_t velocity) {
    m_maximumChassisLinearVelocity = velocity;
  }

  wpi::units::radians_per_second_t GetMaximumChassisAngularVelocity() const {
    return m_maximumChassisAngularVelocity;
  }
  void SetMaximumChassisAngularVelocity(wpi::units::radians_per_second_t velocity) {
    m_maximumChassisAngularVelocity = velocity;
  }

  bool IsTranslationCubeEnabled() const {
    return m_translationCube.has_value() && m_translationCube.value()();
  }
  void SetTranslationCubeEnabled(bool enabled) {
    m_translationCube = enabled ? std::optional<std::function<bool()>>{[] { return true; }} : std::nullopt;
  }

  bool IsOmegaCubeEnabled() const { return m_omegaCube.has_value() && m_omegaCube.value()(); }
  void SetOmegaCubeEnabled(bool enabled) {
    m_omegaCube = enabled ? std::optional<std::function<bool()>>{[] { return true; }} : std::nullopt;
  }

  bool IsAllianceRelativeEnabled() const {
    return m_allianceRelative.has_value() && m_allianceRelative.value()();
  }
  void SetAllianceRelativeEnabled(bool enabled) {
    m_allianceRelative = enabled ? std::optional<std::function<bool()>>{[] { return true; }} : std::nullopt;
  }

  bool IsRobotRelativeEnabled() const { return m_robotRelative.has_value() && m_robotRelative.value()(); }
  void SetRobotRelativeEnabled(bool enabled) {
    m_robotRelative = enabled ? std::optional<std::function<bool()>>{[] { return true; }} : std::nullopt;
  }

  SwerveInputStream& WithMaximumLinearVelocity(wpi::units::meters_per_second_t velocity) {
    m_maximumChassisLinearVelocity = velocity;
    return *this;
  }

  SwerveInputStream& WithMaximumAngularVelocity(wpi::units::radians_per_second_t velocity) {
    m_maximumChassisAngularVelocity = velocity;
    return *this;
  }

  SwerveInputStream& WithControllerRotationAxis(std::function<double()> rot) {
    m_controllerOmega = std::move(rot);
    return *this;
  }

  SwerveInputStream& WithControllerHeadingAxis(std::function<double()> headingX,
                                               std::function<double()> headingY) {
    WithHeading([headingX, headingY] {
      return wpi::units::radian_t{std::atan2(headingX(), headingY())};
    });
    m_controllerHeadingX = std::move(headingX);
    m_controllerHeadingY = std::move(headingY);
    return *this;
  }

  SwerveInputStream& WithDeadband(double deadband) {
    m_axisDeadband = (deadband == 0.0) ? std::nullopt : std::optional<double>{deadband};
    return *this;
  }

  SwerveInputStream& WithScaleTranslation(double scale) {
    m_translationAxisScale = (scale == 0.0) ? std::nullopt : std::optional<double>{scale};
    return *this;
  }

  SwerveInputStream& WithScaleRotation(double scale) {
    m_omegaAxisScale = (scale == 0.0) ? std::nullopt : std::optional<double>{scale};
    return *this;
  }

  SwerveInputStream& WithHeadingControl(std::function<bool()> trigger) {
    m_headingEnabled = std::move(trigger);
    return *this;
  }

  SwerveInputStream& WithHeading(std::function<wpi::units::radian_t()> heading) {
    m_headingSupplier = std::move(heading);
    m_controllerHeadingX.reset();
    m_controllerHeadingY.reset();
    return *this;
  }

  SwerveInputStream& WithAim(std::function<wpi::math::Pose2d()> aimTarget,
                            std::function<bool()> trigger) {
    m_aimTarget = std::move(aimTarget);
    m_aimEnabled = std::move(trigger);
    return *this;
  }

  SwerveInputStream& WithTranslationOnly(std::function<bool()> trigger) {
    m_translationOnlyEnabled = std::move(trigger);
    return *this;
  }

  SwerveInputStream& WithCubeRotationControllerAxis(std::function<bool()> enabled) {
    m_omegaCube = std::move(enabled);
    return *this;
  }

  SwerveInputStream& WithCubeRotationControllerAxis() {
    return WithCubeRotationControllerAxis([] { return true; });
  }

  SwerveInputStream& WithCubeTranslationControllerAxis(std::function<bool()> enabled) {
    m_translationCube = std::move(enabled);
    return *this;
  }

  SwerveInputStream& WithCubeTranslationControllerAxis() {
    return WithCubeTranslationControllerAxis([] { return true; });
  }

  SwerveInputStream& WithAllianceRelativeControl(std::function<bool()> enabled) {
    m_allianceRelative = std::move(enabled);
    return *this;
  }

  SwerveInputStream& WithAllianceRelativeControl() {
    return WithAllianceRelativeControl([] { return true; });
  }

  SwerveInputStream& WithRobotRelative(std::function<bool()> enabled) {
    m_robotRelative = std::move(enabled);
    return *this;
  }

  SwerveInputStream& WithRobotRelative() {
    return WithRobotRelative([] { return true; });
  }

  SwerveInputStream& WithTranslationHeadingOffset(wpi::math::Rotation2d angle,
                                                std::function<bool()> enabled) {
    m_translationHeadingOffset = angle;
    m_translationHeadingOffsetEnabled = std::move(enabled);
    return *this;
  }

  SwerveInputStream& WithTranslationHeadingOffset(wpi::math::Rotation2d angle) {
    return WithTranslationHeadingOffset(angle, [] { return true; });
  }

  wpi::math::ChassisVelocities Get() {
    // Starts from the drive config's maximums (see the constructor); WithMaximum*Velocity and live
    // tuning override them.
    double maxLinear = m_maximumChassisLinearVelocity.value();
    wpi::units::radians_per_second_t maxAngular = m_maximumChassisAngularVelocity;

    wpi::math::Translation2d scaledTranslation = ApplyTranslationScalar(
        ApplyDeadband(m_controllerTranslationX()), ApplyDeadband(m_controllerTranslationY()));
    scaledTranslation = ApplyTranslationCube(scaledTranslation);
    scaledTranslation = ApplyAllianceAwareTranslation(scaledTranslation);

    double vx = scaledTranslation.X().value() * maxLinear;
    double vy = scaledTranslation.Y().value() * maxLinear;
    double omega = 0.0;
    wpi::math::ChassisVelocities speeds{};

    SwerveInputMode newMode = FindMode();
    if (m_currentMode != newMode) {
      TransitionMode(newMode);
    }

    switch (newMode) {
      case SwerveInputMode::TRANSLATION_ONLY: {
        auto& pid = RequireRotationPID();
        omega = pid.Calculate(wpi::units::radian_t{m_swerveDrive->GetGyroAngle()}.value(),
                              m_lockedHeading.value().Radians().value());
        break;
      }
      case SwerveInputMode::ANGULAR_VELOCITY: {
        omega = ApplyOmegaCube(ApplyRotationalScalar(ApplyDeadband(m_controllerOmega.value()()))) *
                maxAngular.value();
        break;
      }
      case SwerveInputMode::HEADING: {
        auto& pid = RequireRotationPID();
        omega = pid.Calculate(wpi::units::radian_t{m_swerveDrive->GetGyroAngle()}.value(),
                              m_headingSupplier.value()().value());
        if (m_controllerHeadingX.has_value() && m_controllerHeadingY.has_value() &&
            m_axisDeadband.has_value() &&
            std::abs(m_controllerHeadingX.value()()) + std::abs(m_controllerHeadingY.value()()) <
                m_axisDeadband.value()) {
          omega = 0.0;
        }
        break;
      }
      case SwerveInputMode::AIM: {
        auto& pid = RequireRotationPID();
        auto currentHeading =
            wpi::math::Rotation2d{wpi::units::radian_t{m_swerveDrive->GetGyroAngle()}};
        auto relativeTrl = m_aimTarget.value()().RelativeTo(m_swerveDrive->GetPose()).Translation();
        auto target = relativeTrl.Angle().value_or(wpi::math::Rotation2d{}) + currentHeading;
        omega = pid.Calculate(currentHeading.Radians().value(), target.Radians().value());
        break;
      }
    }

    m_currentMode = newMode;
    speeds = wpi::math::ChassisVelocities{wpi::units::meters_per_second_t{vx},
                                          wpi::units::meters_per_second_t{vy},
                                          wpi::units::radians_per_second_t{omega}};
    auto fieldRelativeSpeeds = ApplyTranslationHeadingOffset(ApplyRobotRelativeTranslation(speeds));
    if (auto telemetry = GetTelemetry()) {
      telemetry->get().UpdateTelemetry();
    }
    return fieldRelativeSpeeds;
  }

  wpi::math::ChassisVelocities operator()() { return Get(); }

 private:
  enum class SwerveInputMode {
    TRANSLATION_ONLY,
    ANGULAR_VELOCITY,
    HEADING,
    AIM,
  };

  SwerveInputStream(SwerveDrive<NumModules>& drive, std::function<double()> x,
                    std::function<double()> y)
      : m_swerveDrive{&drive},
        m_controllerTranslationX{std::move(x)},
        m_controllerTranslationY{std::move(y)} {
    auto& cfg = drive.GetConfig();
    if (auto maxLinear = cfg.GetMaximumChassisLinearVelocity()) {
      m_maximumChassisLinearVelocity = *maxLinear;
    }
    if (auto maxAngular = cfg.GetMaximumChassisAngularVelocity()) {
      m_maximumChassisAngularVelocity = wpi::units::radians_per_second_t{*maxAngular};
    }
  }

  SwerveDrive<NumModules>* m_swerveDrive;
  std::function<double()> m_controllerTranslationX;
  std::function<double()> m_controllerTranslationY;
  std::optional<std::function<double()>> m_controllerOmega;
  std::optional<std::function<double()>> m_controllerHeadingX;
  std::optional<std::function<double()>> m_controllerHeadingY;
  std::optional<std::function<wpi::units::radian_t()>> m_headingSupplier;
  std::optional<double> m_axisDeadband;
  std::optional<double> m_translationAxisScale;
  std::optional<double> m_omegaAxisScale;
  std::optional<std::function<wpi::math::Pose2d()>> m_aimTarget;
  std::optional<std::function<bool()>> m_headingEnabled;
  std::optional<wpi::math::Rotation2d> m_lockedHeading;
  std::optional<std::function<bool()>> m_aimEnabled;
  std::optional<std::function<bool()>> m_translationOnlyEnabled;
  std::optional<std::function<bool()>> m_translationCube;
  std::optional<std::function<bool()>> m_omegaCube;
  std::optional<std::function<bool()>> m_robotRelative;
  std::optional<std::function<bool()>> m_allianceRelative;
  std::optional<std::function<bool()>> m_translationHeadingOffsetEnabled;
  std::optional<wpi::math::Rotation2d> m_translationHeadingOffset;
  SwerveInputMode m_currentMode{SwerveInputMode::ANGULAR_VELOCITY};
  wpi::units::meters_per_second_t m_maximumChassisLinearVelocity{4.0};
  wpi::units::radians_per_second_t m_maximumChassisAngularVelocity{2.0 * std::numbers::pi};

  /**
   * Telemetry settings and the telemetry created from them. Copies and moves carry the settings but
   * not the telemetry, which points at the stream that created it; a moved-from stream stops
   * publishing.
   */
  struct TelemetryHolder {
    std::optional<std::pair<std::string, TelemetryVerbosity>> settings;
    std::unique_ptr<SwerveInputStreamTelemetry<NumModules>> telemetry;

    TelemetryHolder() = default;
    TelemetryHolder(const TelemetryHolder& other) : settings{other.settings} {}
    TelemetryHolder(TelemetryHolder&& other) noexcept : settings{std::move(other.settings)} {
      other.settings.reset();
      other.telemetry.reset();
    }
    TelemetryHolder& operator=(const TelemetryHolder& other) {
      if (this != &other) {
        settings = other.settings;
        telemetry.reset();
      }
      return *this;
    }
    TelemetryHolder& operator=(TelemetryHolder&& other) noexcept {
      if (this != &other) {
        settings = std::move(other.settings);
        telemetry.reset();
        other.settings.reset();
        other.telemetry.reset();
      }
      return *this;
    }
  };
  TelemetryHolder m_telemetry;

  wpi::math::PIDController& RequireRotationPID() {
    auto pid = m_swerveDrive->GetConfig().GetRotationPID();
    if (!pid) {
      throw exceptions::SwerveDriveConfigurationException(
          "No rotation PID controller configured, heading, aim, and translation only control are "
          "unavailable. Use SwerveDriveConfig::WithRotationController(PIDController) to fix this "
          "error.");
    }
    return pid->get();
  }

  SwerveInputMode FindMode() {
    if (m_translationOnlyEnabled.has_value() && m_translationOnlyEnabled.value()()) {
      return SwerveInputMode::TRANSLATION_ONLY;
    }
    if (m_aimEnabled.has_value() && m_aimEnabled.value()()) {
      if (m_aimTarget.has_value()) {
        return SwerveInputMode::AIM;
      }
      std::cerr << "[YAMS SwerveInputStream] AIM mode enabled but no target set. Call WithAim() first.\n";
    }
    if (m_headingEnabled.has_value() && m_headingEnabled.value()()) {
      if (m_headingSupplier.has_value()) {
        return SwerveInputMode::HEADING;
      }
      std::cerr << "[YAMS SwerveInputStream] HEADING mode enabled but no heading set. "
                   "Call WithHeading() or WithControllerHeadingAxis() first.\n";
    }
    if (!m_controllerOmega.has_value()) {
      std::cerr << "[YAMS SwerveInputStream] No rotation axis configured. "
                   "Call WithControllerRotationAxis() or WithControllerHeadingAxis(). "
                   "Falling back to TRANSLATION_ONLY.\n";
      return SwerveInputMode::TRANSLATION_ONLY;
    }
    return SwerveInputMode::ANGULAR_VELOCITY;
  }

  void TransitionMode(SwerveInputMode newMode) {
    switch (m_currentMode) {
      case SwerveInputMode::TRANSLATION_ONLY:
        m_lockedHeading = std::nullopt;
        break;
      case SwerveInputMode::HEADING:
      case SwerveInputMode::AIM:
        m_swerveDrive->ResetRotationPID();
        break;
      default:
        break;
    }
    switch (newMode) {
      case SwerveInputMode::TRANSLATION_ONLY:
        m_lockedHeading =
            wpi::math::Rotation2d{wpi::units::radian_t{m_swerveDrive->GetGyroAngle()}};
        m_swerveDrive->ResetRotationPID();
        break;
      case SwerveInputMode::HEADING:
      case SwerveInputMode::AIM:
        m_swerveDrive->ResetRotationPID();
        break;
      default:
        break;
    }
  }

  double ApplyDeadband(double axisValue) {
    return m_axisDeadband.has_value() ? wpi::math::ApplyDeadband(axisValue, m_axisDeadband.value())
                                      : axisValue;
  }

  double ApplyRotationalScalar(double axisValue) {
    return m_omegaAxisScale.has_value() ? axisValue * m_omegaAxisScale.value() : axisValue;
  }

  wpi::math::Translation2d ApplyTranslationScalar(double xAxis, double yAxis) {
    wpi::math::Translation2d t{wpi::units::meter_t{xAxis}, wpi::units::meter_t{yAxis}};
    return m_translationAxisScale.has_value()
               ? SwerveDriveConfig::ScaleTranslation(t, m_translationAxisScale.value())
               : t;
  }

  wpi::math::Translation2d ApplyTranslationCube(wpi::math::Translation2d translation) {
    if (m_translationCube.has_value() && m_translationCube.value()()) {
      return SwerveDriveConfig::CubeTranslation(translation);
    }
    return translation;
  }

  double ApplyOmegaCube(double rotationAxis) {
    if (m_omegaCube.has_value() && m_omegaCube.value()()) {
      return rotationAxis * rotationAxis * rotationAxis;
    }
    return rotationAxis;
  }

  wpi::math::ChassisVelocities ApplyRobotRelativeTranslation(wpi::math::ChassisVelocities speeds) {
    if (m_robotRelative.has_value() && m_robotRelative.value()()) {
      return speeds.ToFieldRelative(
          wpi::math::Rotation2d{wpi::units::radian_t{m_swerveDrive->GetGyroAngle()}});
    }
    return speeds;
  }

  wpi::math::Translation2d ApplyAllianceAwareTranslation(wpi::math::Translation2d translation) {
    if (m_robotRelative.has_value() && m_robotRelative.value()()) {
      return translation;
    }
    if (m_allianceRelative.has_value() && m_allianceRelative.value()()) {
      auto alliance = wpi::MatchState::GetAlliance();
      if (alliance.has_value() && alliance.value() == wpi::Alliance::RED) {
        return translation.RotateBy(wpi::math::Rotation2d{wpi::units::degree_t{180.0}});
      }
    }
    return translation;
  }

  wpi::math::ChassisVelocities ApplyTranslationHeadingOffset(wpi::math::ChassisVelocities speeds) {
    if (m_translationHeadingOffsetEnabled.has_value() &&
        m_translationHeadingOffsetEnabled.value()() && m_translationHeadingOffset.has_value()) {
      wpi::math::Translation2d vec{wpi::units::meter_t{speeds.vx.value()},
                                   wpi::units::meter_t{speeds.vy.value()}};
      auto rotated = vec.RotateBy(m_translationHeadingOffset.value());
      return wpi::math::ChassisVelocities{wpi::units::meters_per_second_t{rotated.X().value()},
                                          wpi::units::meters_per_second_t{rotated.Y().value()},
                                          speeds.omega};
    }
    return speeds;
  }
};

}  // namespace yams::mechanisms::swerve::utility

// The telemetry is a template too, so it only needs to be complete where a stream is used.
#include "yams/mechanisms/swerve/utility/SwerveInputStreamTelemetry.hpp"
