// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

#pragma once

#include <frc/geometry/Pose2d.h>
#include <frc/geometry/struct/Pose2dStruct.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <frc/kinematics/struct/ChassisSpeedsStruct.h>
#include <frc/kinematics/struct/SwerveModuleStateStruct.h>

#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include "yams/motorcontrollers/SmartMotorControllerConfig.hpp"
#include "yams/telemetry/SmartMotorControllerTelemetry.hpp"
#include "yams/telemetry/StructArrayTelemetry.hpp"
#include "yams/telemetry/StructTelemetry.hpp"

namespace yams::telemetry {

/**
 * Swerve drive telemetry configuration.
 *
 * Use this builder to select exactly which fields are published to NetworkTables and/or
 * DataLog. Every field is disabled by default; call the individual With*() methods to opt in, or
 * use WithTelemetryVerbosity() to enable a predefined set.
 *
 * The field enums are owned by this class (rather than by SwerveDriveTelemetry) so that this
 * config header has no dependency on SwerveDriveTelemetry.hpp; SwerveDriveTelemetry re-exposes
 * them via type aliases for API parity.
 *
 * ### Example
 * @code{.cpp}
 * SwerveDriveTelemetryConfig telemetryCfg =
 *     SwerveDriveTelemetryConfig()
 *         .WithTelemetryVerbosity(TelemetryVerbosity::HIGH)
 *         .WithPose()
 *         .WithGyro()
 *         .WithCurrentRobotRelativeChassisSpeeds()
 *         .WithFieldRelativeChassisSpeeds()
 *         .WithDesiredRobotRelativeChassisSpeeds()
 *         .WithDesiredModuleStates()
 *         .WithCurrentModuleStates()
 *         .WithoutNetworkTables()
 *         .WithDataLogName("swerve");
 * @endcode
 */
class SwerveDriveTelemetryConfig {
 public:
  using TelemetryVerbosity = motorcontrollers::SmartMotorControllerConfig::TelemetryVerbosity;

  /** Struct telemetry field for a SwerveDrive, backed by frc::Pose2d. */
  enum class StructTelemetryField {
    /** Live-tunable target pose used by SwerveDriveTelemetry::ApplyTuningValues(). */
    TargetPose,
    /** Estimated field-relative pose of the robot. */
    Pose,
    /** Last-commanded desired robot-relative chassis speeds. */
    DesiredRobotRelativeChassisSpeeds,
    /** Measured robot-relative chassis speeds. */
    CurrentRobotRelativeChassisSpeeds,
    /** Measured field-relative chassis speeds. */
    FieldRelativeChassisSpeeds,
  };

  /** Struct array telemetry field for a SwerveDrive, backed by frc::SwerveModuleState. */
  enum class StructArrayTelemetryField {
    /** Last-commanded desired module states. */
    DesiredModuleStates,
    /** Measured module states. */
    CurrentModuleStates,
  };

  /** Double telemetry field for a SwerveDrive. */
  enum class DoubleTelemetryField {
    /** Translational proportional gain for auto-aligning the robot. */
    TranslationP,
    /** Translational integral gain for auto-aligning the robot. */
    TranslationI,
    /** Translational derivative gain for auto-aligning the robot. */
    TranslationD,
    /** Rotational proportional gain for auto-aligning the robot. */
    RotationP,
    /** Rotational integral gain for auto-aligning the robot. */
    RotationI,
    /** Rotational derivative gain for auto-aligning the robot. */
    RotationD,
    /** Gyro angle, in degrees. */
    Gyro,
  };

  SwerveDriveTelemetryConfig();

  /**
   * Set up a DataLog entry for this drive.
   *
   * @param dataLogName DataLog entry name.
   * @return *this for chaining.
   */
  SwerveDriveTelemetryConfig& WithDataLogName(const std::string& dataLogName);

  /**
   * Enable or disable NT4 telemetry. This will not create NT4 entries and is generally only
   * advisable during competition matches.
   *
   * @param enabled true to enable NT4 output.
   * @return *this for chaining.
   */
  SwerveDriveTelemetryConfig& WithNetworkTables(bool enabled);

  /** Disable NetworkTable output. @return *this for chaining. */
  SwerveDriveTelemetryConfig& WithoutNetworkTables();

  /**
   * Enable a preset bundle of fields based on verbosity level.
   *
   * @param verbosity Verbosity level to apply.
   * @return *this for chaining.
   */
  SwerveDriveTelemetryConfig& WithTelemetryVerbosity(TelemetryVerbosity verbosity);

  /** Enables the pose logging. @return *this for chaining. */
  SwerveDriveTelemetryConfig& WithPose();
  /** Enables the gyro angle logging. @return *this for chaining. */
  SwerveDriveTelemetryConfig& WithGyro();
  /** Enables the last-commanded desired robot relative chassis speeds logging. */
  SwerveDriveTelemetryConfig& WithDesiredRobotRelativeChassisSpeeds();
  /** Enables the measured robot relative chassis speeds logging. */
  SwerveDriveTelemetryConfig& WithCurrentRobotRelativeChassisSpeeds();
  /** Enables the measured field relative chassis speeds logging. */
  SwerveDriveTelemetryConfig& WithFieldRelativeChassisSpeeds();
  /** Enables the last-commanded desired module states logging. */
  SwerveDriveTelemetryConfig& WithDesiredModuleStates();
  /** Enables the measured module states logging. */
  SwerveDriveTelemetryConfig& WithCurrentModuleStates();

  /** @return Optional DataLog entry name. */
  std::optional<std::string> GetDataLogName() const;
  /** @return true if NT4 output is enabled. */
  bool GetNT4Enabled() const;

  /** @return Configured DoubleTelemetry for each DoubleTelemetryField. */
  std::unordered_map<DoubleTelemetryField, DoubleTelemetry<DoubleTelemetryField>>&
  GetDoubleFields();

  /** @return Configured StructTelemetry<Pose2d> for TargetPose/Pose. */
  std::unordered_map<StructTelemetryField, StructTelemetry<frc::Pose2d, StructTelemetryField>>&
  GetPoseFields();

  /** @return Configured StructTelemetry<ChassisSpeeds> for the chassis speeds fields. */
  std::unordered_map<StructTelemetryField,
                     StructTelemetry<frc::ChassisSpeeds, StructTelemetryField>>&
  GetChassisSpeedsFields();

  /** @return Configured StructArrayTelemetry<SwerveModuleState> for the module state fields. */
  std::unordered_map<StructArrayTelemetryField,
                     StructArrayTelemetry<frc::SwerveModuleState, StructArrayTelemetryField>>&
  GetModuleStatesFields();

  /**
   * Escape hatch for unimplemented fields which should be enabled or disabled.
   *
   * @param field Field to configure.
   * @param value Enable on true, disable on false.
   * @return *this for chaining.
   */
  SwerveDriveTelemetryConfig& WithCustom(DoubleTelemetryField field, bool value);
  /** @overload */
  SwerveDriveTelemetryConfig& WithCustom(StructTelemetryField field, bool value);
  /** @overload */
  SwerveDriveTelemetryConfig& WithCustom(StructArrayTelemetryField field, bool value);
  /** @overload */
  SwerveDriveTelemetryConfig& WithCustom(const std::vector<DoubleTelemetryField>& fields,
                                         bool value);
  /** @overload */
  SwerveDriveTelemetryConfig& WithCustom(const std::vector<StructTelemetryField>& fields,
                                         bool value);
  /** @overload */
  SwerveDriveTelemetryConfig& WithCustom(const std::vector<StructArrayTelemetryField>& fields,
                                         bool value);

 private:
  std::optional<std::string> m_dataLogName;
  bool m_nt4Telemetry{true};

  std::unordered_map<DoubleTelemetryField, DoubleTelemetry<DoubleTelemetryField>> m_doubleFields;
  std::unordered_map<StructTelemetryField, StructTelemetry<frc::Pose2d, StructTelemetryField>>
      m_poseFields;
  std::unordered_map<StructTelemetryField,
                     StructTelemetry<frc::ChassisSpeeds, StructTelemetryField>>
      m_chassisSpeedsFields;
  std::unordered_map<StructArrayTelemetryField,
                     StructArrayTelemetry<frc::SwerveModuleState, StructArrayTelemetryField>>
      m_moduleStatesFields;
};

}  // namespace yams::telemetry
