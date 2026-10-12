// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

#pragma once

#include <frc/DriverStation.h>
#include <frc/MathUtil.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/geometry/Translation2d.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/length.h>
#include <units/velocity.h>

#include <cmath>
#include <functional>
#include <iostream>
#include <memory>
#include <numbers>
#include <optional>
#include <string>
#include <utility>

#include "yams/exceptions.hpp"
#include "yams/mechanisms/swerve/SwerveDrive.hpp"
#include "yams/mechanisms/swerve/SwerveDriveConfig.hpp"

namespace yams::mechanisms::swerve::utility {

template <size_t NumModules>
class SwerveInputStreamTelemetry;

/**
 * Transforms controller axis inputs into ChassisSpeeds with built-in support for
 * angular-velocity, heading, aim-at-pose, and translation-only drive modes.
 *
 * Call Get() (or `operator()`) each loop to get the current ChassisSpeeds.
 * Passes the result to SwerveDrive::Drive() as a supplier.
 *
 * Inspired by SciBorgs FRC 1155.
 *
 * @tparam NumModules Number of swerve modules (must match the SwerveDrive).
 *
 * ### Example usage
 * @code{.cpp}
 * using namespace yams::mechanisms::swerve::utility;
 *
 * // Declare as subsystem member:
 * //   frc::XboxController m_driverController{0};
 * //   std::optional<SwerveInputStream<4>> m_driveStream;
 *
 * // In constructor:
 * m_driveStream.emplace(
 *   SwerveInputStream<4>::Of(*m_drive,
 *     [this]{ return -m_driverController.GetLeftY(); },
 *     [this]{ return -m_driverController.GetLeftX(); })
 *   .WithControllerRotationAxis([this]{ return m_driverController.GetRightX(); })
 *   .WithDeadband(0.1)
 *   .WithScaleTranslation(0.8)
 *   .WithAllianceRelativeControl()
 * );
 *
 * // Heading-based copy for direct-angle control:
 * SwerveInputStream<4> driveHeading = m_driveStream->Clone()
 *   .WithControllerHeadingAxis(
 *     [this]{ return m_driverController.GetRightX(); },
 *     [this]{ return m_driverController.GetRightY(); })
 *   .WithHeadingControl([this]{ return true; });
 *
 * // Pass to Drive():
 * m_drive->Drive([this]{ return m_driveStream->Get(); });
 * @endcode
 */
template <size_t NumModules = 4>
class SwerveInputStream {
 public:
  /**
   * Create a SwerveInputStream without a rotation component.
   *
   * A rotation source must be added with WithControllerRotationAxis() or
   * WithControllerHeadingAxis(); otherwise Get() falls back to TRANSLATION_ONLY.
   *
   * @param drive SwerveDrive to query pose and gyro from.
   * @param x     Translation X axis supplier, range [-1, 1].
   * @param y     Translation Y axis supplier, range [-1, 1].
   * @return New SwerveInputStream instance.
   */
  static SwerveInputStream Of(SwerveDrive<NumModules>& drive, std::function<double()> x,
                              std::function<double()> y) {
    return SwerveInputStream{drive, std::move(x), std::move(y)};
  }

  /**
   * Create a SwerveInputStream with an angular-velocity rotation axis.
   *
   * @param drive SwerveDrive to query pose and gyro from.
   * @param x     Translation X axis supplier, range [-1, 1].
   * @param y     Translation Y axis supplier, range [-1, 1].
   * @param rot   Rotation axis supplier, range [-1, 1].
   * @return New SwerveInputStream instance.
   */
  static SwerveInputStream Of(SwerveDrive<NumModules>& drive, std::function<double()> x,
                              std::function<double()> y, std::function<double()> rot) {
    return SwerveInputStream{drive, std::move(x), std::move(y), std::move(rot)};
  }

  /**
   * Construct with an angular-velocity rotation axis (enables ANGULAR_VELOCITY mode).
   *
   * @param drive SwerveDrive to query from.
   * @param x     Translation X axis supplier, range [-1, 1].
   * @param y     Translation Y axis supplier, range [-1, 1].
   * @param rot   Rotation axis supplier, range [-1, 1].
   */
  SwerveInputStream(SwerveDrive<NumModules>& drive, std::function<double()> x,
                    std::function<double()> y, std::function<double()> rot)
      : SwerveInputStream{drive, std::move(x), std::move(y)} {
    m_controllerOmega = std::move(rot);
  }

  /**
   * Construct with heading X/Y axes (enables HEADING mode when WithHeadingControl() is active).
   *
   * @param drive    SwerveDrive to query from.
   * @param x        Translation X axis supplier, range [-1, 1].
   * @param y        Translation Y axis supplier, range [-1, 1].
   * @param headingX Heading X axis supplier, range [-1, 1].
   * @param headingY Heading Y axis supplier, range [-1, 1].
   */
  SwerveInputStream(SwerveDrive<NumModules>& drive, std::function<double()> x,
                    std::function<double()> y, std::function<double()> headingX,
                    std::function<double()> headingY)
      : SwerveInputStream{drive, std::move(x), std::move(y)} {
    WithControllerHeadingAxis(std::move(headingX), std::move(headingY));
  }

  using TelemetryVerbosity = SwerveDriveConfig::TelemetryVerbosity;

  /**
   * Return a copy of this stream; subsequent With* calls on the copy do not affect the original.
   * Unlike a plain copy, the clone has no telemetry, so it does not publish under this stream's
   * name.
   */
  SwerveInputStream Clone() const {
    SwerveInputStream clone = *this;
    clone.m_telemetry.settings.reset();
    return clone;
  }

  // ---- Telemetry and live tuning ---------------------------------------------

  /**
   * Publish this stream's telemetry under SwerveInputStream/<name>, updated every time the stream
   * is read. At HIGH the stream is also live tunable: see SwerveInputStreamTelemetry. Replaces any
   * telemetry this stream already had; NONE turns it off.
   *
   * The telemetry is created when the stream is first read or GetTelemetry() is called, so copies
   * made while building the stream do not publish. A copy of a stream with telemetry publishes
   * under the same name once read; use Clone() for a copy without telemetry.
   *
   * @param name      Name of the stream in NetworkTables (e.g., "drive").
   * @param verbosity TelemetryVerbosity to publish at.
   * @return *this for chaining.
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

  /** Get the name of the drive mode used by the last Get() call, e.g. "ANGULAR_VELOCITY". */
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

  /** Get the controller axis deadband, 0 when none is applied. */
  double GetAxisDeadband() const { return m_axisDeadband.value_or(0.0); }

  /** Set the deadband applied to every axis, 0 to disable. */
  void SetAxisDeadband(double deadband) {
    m_axisDeadband = (deadband == 0.0) ? std::nullopt : std::optional<double>{deadband};
  }

  /** Get the translation axis scalar, 1 when unscaled. */
  double GetTranslationAxisScale() const { return m_translationAxisScale.value_or(1.0); }

  /** Set the scalar in (0, 1] applied to the translation axes, 0 to disable. */
  void SetTranslationAxisScale(double scale) {
    m_translationAxisScale = (scale == 0.0) ? std::nullopt : std::optional<double>{scale};
  }

  /** Get the angular velocity axis scalar, 1 when unscaled. */
  double GetOmegaAxisScale() const { return m_omegaAxisScale.value_or(1.0); }

  /** Set the scalar in (0, 1] applied to the angular velocity axis, 0 to disable. */
  void SetOmegaAxisScale(double scale) {
    m_omegaAxisScale = (scale == 0.0) ? std::nullopt : std::optional<double>{scale};
  }

  /** Get the chassis linear velocity a full translation axis maps to. */
  units::meters_per_second_t GetMaximumChassisLinearVelocity() const {
    return m_maximumChassisLinearVelocity;
  }

  /** Set the chassis linear velocity a full translation axis maps to. */
  void SetMaximumChassisLinearVelocity(units::meters_per_second_t velocity) {
    m_maximumChassisLinearVelocity = velocity;
  }

  /** Get the chassis angular velocity a full angular velocity axis maps to. */
  units::radians_per_second_t GetMaximumChassisAngularVelocity() const {
    return m_maximumChassisAngularVelocity;
  }

  /** Set the chassis angular velocity a full angular velocity axis maps to. */
  void SetMaximumChassisAngularVelocity(units::radians_per_second_t velocity) {
    m_maximumChassisAngularVelocity = velocity;
  }

  /** Check if the translation magnitude is cubed. */
  bool IsTranslationCubeEnabled() const {
    return m_translationCube.has_value() && m_translationCube.value()();
  }

  /** Enable or disable cubing the translation magnitude. */
  void SetTranslationCubeEnabled(bool enabled) { m_translationCube = Constant(enabled); }

  /** Check if the angular velocity axis is cubed. */
  bool IsOmegaCubeEnabled() const { return m_omegaCube.has_value() && m_omegaCube.value()(); }

  /** Enable or disable cubing the angular velocity axis. */
  void SetOmegaCubeEnabled(bool enabled) { m_omegaCube = Constant(enabled); }

  /** Check if translation is flipped 180 degrees while on the red alliance. */
  bool IsAllianceRelativeEnabled() const {
    return m_allianceRelative.has_value() && m_allianceRelative.value()();
  }

  /** Enable or disable flipping translation 180 degrees while on the red alliance. */
  void SetAllianceRelativeEnabled(bool enabled) { m_allianceRelative = Constant(enabled); }

  /** Check if the translation axes are treated as robot relative. */
  bool IsRobotRelativeEnabled() const {
    return m_robotRelative.has_value() && m_robotRelative.value()();
  }

  /** Enable or disable treating the translation axes as robot relative. */
  void SetRobotRelativeEnabled(bool enabled) { m_robotRelative = Constant(enabled); }

  // ---- Builder methods -------------------------------------------------------

  /**
   * Set the maximum chassis linear velocity (default 4 m/s).
   *
   * @param velocity Maximum linear velocity.
   * @return *this for chaining.
   */
  SwerveInputStream& WithMaximumLinearVelocity(units::meters_per_second_t velocity) {
    m_maximumChassisLinearVelocity = velocity;
    return *this;
  }

  /**
   * Set the maximum chassis angular velocity (default 2π rad/s = 1 rotation/s).
   *
   * @param velocity Maximum angular velocity.
   * @return *this for chaining.
   */
  SwerveInputStream& WithMaximumAngularVelocity(units::radians_per_second_t velocity) {
    m_maximumChassisAngularVelocity = velocity;
    return *this;
  }

  /**
   * Add an angular-velocity rotation axis (enables ANGULAR_VELOCITY mode).
   *
   * @param rot Supplier returning rotation input in range [-1, 1].
   * @return *this for chaining.
   */
  SwerveInputStream& WithControllerRotationAxis(std::function<double()> rot) {
    m_controllerOmega = std::move(rot);
    return *this;
  }

  /**
   * Add heading X/Y axes (enables HEADING mode when WithHeadingControl() is active). The robot
   * stops turning while the stick is inside the deadband. Replaces any heading from WithHeading().
   *
   * @param headingX Supplier for the heading X component, range [-1, 1].
   * @param headingY Supplier for the heading Y component, range [-1, 1].
   * @return *this for chaining.
   */
  SwerveInputStream& WithControllerHeadingAxis(std::function<double()> headingX,
                                               std::function<double()> headingY) {
    WithHeading([headingX, headingY] { return units::radian_t{std::atan2(headingX(), headingY())}; });
    m_controllerHeadingX = std::move(headingX);
    m_controllerHeadingY = std::move(headingY);
    return *this;
  }

  /**
   * Set the field relative heading to face while heading control is enabled, from any source such
   * as a fixed angle or the angle to a target. Replaces any controller heading axis.
   *
   * @param heading Supplier of the field relative heading.
   * @return *this for chaining.
   */
  SwerveInputStream& WithHeading(std::function<units::radian_t()> heading) {
    m_headingSupplier = std::move(heading);
    m_controllerHeadingX.reset();
    m_controllerHeadingY.reset();
    return *this;
  }

  /**
   * Set a deadband applied to all controller axes.
   *
   * @param deadband Deadband value, range [0, 1).  Pass 0 to disable.
   * @return *this for chaining.
   */
  SwerveInputStream& WithDeadband(double deadband) {
    m_axisDeadband = (deadband == 0.0) ? std::nullopt : std::optional<double>{deadband};
    return *this;
  }

  /**
   * Scale the translation axis magnitude by a constant factor.
   *
   * @param scale Scale factor, range (0, 1].  Pass 0 to disable.
   * @return *this for chaining.
   */
  SwerveInputStream& WithScaleTranslation(double scale) {
    m_translationAxisScale = (scale == 0.0) ? std::nullopt : std::optional<double>{scale};
    return *this;
  }

  /**
   * Scale the rotation axis magnitude by a constant factor.
   *
   * @param scale Scale factor, range (0, 1].  Pass 0 to disable.
   * @return *this for chaining.
   */
  SwerveInputStream& WithScaleRotation(double scale) {
    m_omegaAxisScale = (scale == 0.0) ? std::nullopt : std::optional<double>{scale};
    return *this;
  }

  /**
   * Enable heading-based control while the supplier returns true.
   *
   * Requires WithHeading() or WithControllerHeadingAxis() to be configured.
   *
   * @param trigger Supplier that enables HEADING mode when true.
   * @return *this for chaining.
   */
  SwerveInputStream& WithHeadingControl(std::function<bool()> trigger) {
    m_headingEnabled = std::move(trigger);
    return *this;
  }

  /**
   * Enable aim-at-pose mode while the trigger is true.
   *
   * @param aimTarget Supplier for the field-relative pose to aim at.
   * @param trigger   Supplier that enables AIM mode when true.
   * @return *this for chaining.
   */
  SwerveInputStream& WithAim(std::function<frc::Pose2d()> aimTarget,
                             std::function<bool()> trigger) {
    m_aimTarget = std::move(aimTarget);
    m_aimEnabled = std::move(trigger);
    return *this;
  }

  /**
   * Hold the current heading and translate only while the trigger is true.
   *
   * @param trigger Supplier that enables TRANSLATION_ONLY mode when true.
   * @return *this for chaining.
   */
  SwerveInputStream& WithTranslationOnly(std::function<bool()> trigger) {
    m_translationOnlyEnabled = std::move(trigger);
    return *this;
  }

  /**
   * Cube the angular-velocity axis for a non-linear control feel.
   *
   * @param enabled Supplier that enables cubing when true.
   * @return *this for chaining.
   */
  SwerveInputStream& WithCubeRotationControllerAxis(std::function<bool()> enabled) {
    m_omegaCube = std::move(enabled);
    return *this;
  }

  /** Enable cubing of the angular-velocity axis unconditionally. @return *this for chaining. */
  SwerveInputStream& WithCubeRotationControllerAxis() {
    return WithCubeRotationControllerAxis([] { return true; });
  }

  /**
   * Cube the translation axis magnitude for a non-linear control feel.
   *
   * @param enabled Supplier that enables cubing when true.
   * @return *this for chaining.
   */
  SwerveInputStream& WithCubeTranslationControllerAxis(std::function<bool()> enabled) {
    m_translationCube = std::move(enabled);
    return *this;
  }

  /** Enable cubing of the translation axis unconditionally. @return *this for chaining. */
  SwerveInputStream& WithCubeTranslationControllerAxis() {
    return WithCubeTranslationControllerAxis([] { return true; });
  }

  /**
   * Flip the translation when on the Red alliance so forward always aims toward the opponent wall.
   *
   * @param enabled Supplier that enables alliance-relative control when true.
   * @return *this for chaining.
   */
  SwerveInputStream& WithAllianceRelativeControl(std::function<bool()> enabled) {
    m_allianceRelative = std::move(enabled);
    return *this;
  }

  /** Enable alliance-relative control unconditionally. @return *this for chaining. */
  SwerveInputStream& WithAllianceRelativeControl() {
    return WithAllianceRelativeControl([] { return true; });
  }

  /**
   * Route the output ChassisSpeeds through robot-relative conversion.
   *
   * @param enabled Supplier that enables robot-relative output when true.
   * @return *this for chaining.
   */
  SwerveInputStream& WithRobotRelative(std::function<bool()> enabled) {
    m_robotRelative = std::move(enabled);
    return *this;
  }

  /** Enable robot-relative output unconditionally. @return *this for chaining. */
  SwerveInputStream& WithRobotRelative() {
    return WithRobotRelative([] { return true; });
  }

  /**
   * Rotate the output translation vector by a fixed heading offset while the trigger is true.
   *
   * @param angle   Offset rotation to apply.
   * @param enabled Supplier that enables the offset when true.
   * @return *this for chaining.
   */
  SwerveInputStream& WithTranslationHeadingOffset(frc::Rotation2d angle,
                                                  std::function<bool()> enabled) {
    m_translationHeadingOffset = angle;
    m_translationHeadingOffsetEnabled = std::move(enabled);
    return *this;
  }

  /**
   * Rotate the output translation vector by a fixed heading offset unconditionally.
   *
   * @param angle Offset rotation to apply.
   * @return *this for chaining.
   */
  SwerveInputStream& WithTranslationHeadingOffset(frc::Rotation2d angle) {
    return WithTranslationHeadingOffset(angle, [] { return true; });
  }

  // ---- Output ---------------------------------------------------------------

  /**
   * Compute and return the current ChassisSpeeds based on all configured inputs and the active
   * drive mode.  Call this once per robot loop.
   *
   * @return Field-relative (or robot-relative, if configured) ChassisSpeeds.
   */
  frc::ChassisSpeeds Get() {
    double maxLinear = m_maximumChassisLinearVelocity.value();
    units::radians_per_second_t maxAngular = m_maximumChassisAngularVelocity;

    frc::Translation2d scaledTranslation = ApplyTranslationScalar(
        ApplyDeadband(m_controllerTranslationX()), ApplyDeadband(m_controllerTranslationY()));
    scaledTranslation = ApplyTranslationCube(scaledTranslation);
    scaledTranslation = ApplyAllianceAwareTranslation(scaledTranslation);

    double vx = scaledTranslation.X().value() * maxLinear;
    double vy = scaledTranslation.Y().value() * maxLinear;
    double omega = 0.0;
    frc::ChassisSpeeds speeds{};

    SwerveInputMode newMode = FindMode();
    if (m_currentMode != newMode) {
      TransitionMode(newMode);
    }

    switch (newMode) {
      case SwerveInputMode::TRANSLATION_ONLY: {
        auto& pid = RequireRotationPID();
        omega = pid.Calculate(units::radian_t{m_swerveDrive->GetGyroAngle()}.value(),
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
        omega = pid.Calculate(units::radian_t{m_swerveDrive->GetGyroAngle()}.value(),
                              m_headingSupplier.value()().value());
        // Do not turn while the heading stick is inside the deadband.
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
        auto currentHeading = frc::Rotation2d{units::radian_t{m_swerveDrive->GetGyroAngle()}};
        auto relativeTrl = m_aimTarget.value()().RelativeTo(m_swerveDrive->GetPose()).Translation();
        auto target = relativeTrl.Angle() + currentHeading;
        omega = pid.Calculate(currentHeading.Radians().value(), target.Radians().value());
        break;
      }
    }

    m_currentMode = newMode;
    speeds = frc::ChassisSpeeds{units::meters_per_second_t{vx}, units::meters_per_second_t{vy},
                                units::radians_per_second_t{omega}};
    auto fieldRelativeSpeeds = ApplyTranslationHeadingOffset(ApplyRobotRelativeTranslation(speeds));
    if (auto telemetry = GetTelemetry()) {
      telemetry->get().UpdateTelemetry();
    }
    return fieldRelativeSpeeds;
  }

  /**
   * Callable operator so a SwerveInputStream can be used as a
   * `std::function<frc::ChassisSpeeds()>` supplier directly.
   */
  frc::ChassisSpeeds operator()() { return Get(); }

 private:
  enum class SwerveInputMode {
    TRANSLATION_ONLY,
    ANGULAR_VELOCITY,
    HEADING,
    AIM,
  };

  // Base private constructor — requires a rotation source to be added via WithController* methods.
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
      m_maximumChassisAngularVelocity = units::radians_per_second_t{*maxAngular};
    }
  }

  SwerveDrive<NumModules>* m_swerveDrive;
  std::function<double()> m_controllerTranslationX;
  std::function<double()> m_controllerTranslationY;
  std::optional<std::function<double()>> m_controllerOmega;
  std::optional<std::function<double()>> m_controllerHeadingX;
  std::optional<std::function<double()>> m_controllerHeadingY;
  std::optional<std::function<units::radian_t()>> m_headingSupplier;
  std::optional<double> m_axisDeadband;
  std::optional<double> m_translationAxisScale;
  std::optional<double> m_omegaAxisScale;
  std::optional<std::function<frc::Pose2d()>> m_aimTarget;
  std::optional<std::function<bool()>> m_headingEnabled;
  std::optional<frc::Rotation2d> m_lockedHeading;
  std::optional<std::function<bool()>> m_aimEnabled;
  std::optional<std::function<bool()>> m_translationOnlyEnabled;
  std::optional<std::function<bool()>> m_translationCube;
  std::optional<std::function<bool()>> m_omegaCube;
  std::optional<std::function<bool()>> m_robotRelative;
  std::optional<std::function<bool()>> m_allianceRelative;
  std::optional<std::function<bool()>> m_translationHeadingOffsetEnabled;
  std::optional<frc::Rotation2d> m_translationHeadingOffset;
  SwerveInputMode m_currentMode{SwerveInputMode::ANGULAR_VELOCITY};
  units::meters_per_second_t m_maximumChassisLinearVelocity{4.0};
  units::radians_per_second_t m_maximumChassisAngularVelocity{2.0 * std::numbers::pi};

  /**
   * Telemetry settings and the telemetry created from them. Copies and moves carry the settings
   * but not the telemetry, which points at the stream that created it; a moved-from stream stops
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

  // ---- Private helpers -------------------------------------------------------

  /** A supplier that is always true, or no supplier when disabled. */
  static std::optional<std::function<bool()>> Constant(bool enabled) {
    if (!enabled) {
      return std::nullopt;
    }
    return std::function<bool()>{[] { return true; }};
  }

  /** Get the rotation PID controller the heading, aim and translation only modes need. */
  frc::PIDController& RequireRotationPID() {
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
      std::cerr << "[YAMS SwerveInputStream] AIM mode enabled but no target set. "
                   "Call WithAim() first.\n";
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
    // Exit current mode
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
    // Enter new mode
    switch (newMode) {
      case SwerveInputMode::TRANSLATION_ONLY:
        m_lockedHeading = frc::Rotation2d{units::radian_t{m_swerveDrive->GetGyroAngle()}};
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
    return m_axisDeadband.has_value() ? frc::ApplyDeadband(axisValue, m_axisDeadband.value())
                                      : axisValue;
  }

  double ApplyRotationalScalar(double axisValue) {
    return m_omegaAxisScale.has_value() ? axisValue * m_omegaAxisScale.value() : axisValue;
  }

  frc::Translation2d ApplyTranslationScalar(double xAxis, double yAxis) {
    frc::Translation2d t{units::meter_t{xAxis}, units::meter_t{yAxis}};
    return m_translationAxisScale.has_value()
               ? SwerveDriveConfig::ScaleTranslation(t, m_translationAxisScale.value())
               : t;
  }

  frc::Translation2d ApplyTranslationCube(frc::Translation2d translation) {
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

  frc::ChassisSpeeds ApplyRobotRelativeTranslation(frc::ChassisSpeeds speeds) {
    if (m_robotRelative.has_value() && m_robotRelative.value()()) {
      return frc::ChassisSpeeds::FromRobotRelativeSpeeds(
          speeds, frc::Rotation2d{units::radian_t{m_swerveDrive->GetGyroAngle()}});
    }
    return speeds;
  }

  frc::Translation2d ApplyAllianceAwareTranslation(frc::Translation2d translation) {
    // Robot relative translation is not on the field, so there is nothing to flip for the alliance.
    if (m_robotRelative.has_value() && m_robotRelative.value()()) {
      return translation;
    }
    if (m_allianceRelative.has_value() && m_allianceRelative.value()()) {
      auto alliance = frc::DriverStation::GetAlliance();
      if (alliance.has_value() && alliance.value() == frc::DriverStation::Alliance::kRed) {
        return translation.RotateBy(frc::Rotation2d{units::degree_t{180.0}});
      }
    }
    return translation;
  }

  frc::ChassisSpeeds ApplyTranslationHeadingOffset(frc::ChassisSpeeds speeds) {
    if (m_translationHeadingOffsetEnabled.has_value() &&
        m_translationHeadingOffsetEnabled.value()() && m_translationHeadingOffset.has_value()) {
      frc::Translation2d vec{units::meter_t{speeds.vx.value()}, units::meter_t{speeds.vy.value()}};
      auto rotated = vec.RotateBy(m_translationHeadingOffset.value());
      return frc::ChassisSpeeds{units::meters_per_second_t{rotated.X().value()},
                                units::meters_per_second_t{rotated.Y().value()}, speeds.omega};
    }
    return speeds;
  }
};

}  // namespace yams::mechanisms::swerve::utility

// The telemetry is a template too, so it only needs to be complete where a stream is used.
#include "yams/mechanisms/swerve/utility/SwerveInputStreamTelemetry.hpp"
