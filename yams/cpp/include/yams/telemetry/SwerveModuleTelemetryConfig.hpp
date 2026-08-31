// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

#pragma once

#include <frc/kinematics/SwerveModuleState.h>
#include <frc/kinematics/struct/SwerveModuleStateStruct.h>

#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include "yams/motorcontrollers/SmartMotorControllerConfig.hpp"
#include "yams/telemetry/SmartMotorControllerTelemetry.hpp"
#include "yams/telemetry/StructTelemetry.hpp"

namespace yams::telemetry {

/**
 * Swerve module telemetry configuration.
 *
 * Use this builder to select exactly which fields are published to NetworkTables and/or
 * DataLog. Every field is disabled by default; call the individual With*() methods to opt in, or
 * use WithTelemetryVerbosity() to enable a predefined set.
 *
 * ### Example
 * @code{.cpp}
 * SwerveModuleTelemetryConfig telemetryCfg =
 *     SwerveModuleTelemetryConfig()
 *         .WithTelemetryVerbosity(TelemetryVerbosity::HIGH)
 *         .WithAbsoluteEncoder()
 *         .WithState()
 *         .WithoutNetworkTables()
 *         .WithDataLogName("swerve/modules/frontleft");
 * @endcode
 */
class SwerveModuleTelemetryConfig {
 public:
  using TelemetryVerbosity = motorcontrollers::SmartMotorControllerConfig::TelemetryVerbosity;

  /** Double telemetry field for a SwerveModule. */
  enum class DoubleTelemetryField {
    /** Absolute encoder angle, without offsets applied, in degrees. */
    AbsoluteEncoder,
  };

  /** Struct telemetry field for a SwerveModule, backed by frc::SwerveModuleState. */
  enum class StructTelemetryField {
    /** Measured SwerveModuleState of the module. */
    State,
  };

  SwerveModuleTelemetryConfig();

  /**
   * Set up a DataLog entry for this module.
   *
   * @param dataLogName DataLog entry name.
   * @return *this for chaining.
   */
  SwerveModuleTelemetryConfig& WithDataLogName(const std::string& dataLogName);

  /**
   * Enable or disable NT4 telemetry. This will not create NT4 entries and is generally only
   * advisable during competition matches.
   *
   * @param enabled true to enable NT4 output.
   * @return *this for chaining.
   */
  SwerveModuleTelemetryConfig& WithNetworkTables(bool enabled);

  /** Disable NetworkTable output. @return *this for chaining. */
  SwerveModuleTelemetryConfig& WithoutNetworkTables();

  /**
   * Enable a preset bundle of fields based on verbosity level.
   *
   * @param verbosity Verbosity level to apply.
   * @return *this for chaining.
   */
  SwerveModuleTelemetryConfig& WithTelemetryVerbosity(TelemetryVerbosity verbosity);

  /** Enables the absolute encoder angle logging. @return *this for chaining. */
  SwerveModuleTelemetryConfig& WithAbsoluteEncoder();
  /** Enables the SwerveModuleState logging. @return *this for chaining. */
  SwerveModuleTelemetryConfig& WithState();

  /** @return Optional DataLog entry name. */
  std::optional<std::string> GetDataLogName() const;
  /** @return true if NT4 output is enabled. */
  bool GetNT4Enabled() const;

  /** @return Configured DoubleTelemetry for each DoubleTelemetryField. */
  std::unordered_map<DoubleTelemetryField, DoubleTelemetry<DoubleTelemetryField>>&
  GetDoubleFields();

  /** @return Configured StructTelemetry<SwerveModuleState> for each StructTelemetryField. */
  std::unordered_map<StructTelemetryField,
                     StructTelemetry<frc::SwerveModuleState, StructTelemetryField>>&
  GetStructFields();

  /**
   * Escape hatch for unimplemented fields which should be enabled or disabled.
   *
   * @param field Field to configure.
   * @param value Enable on true, disable on false.
   * @return *this for chaining.
   */
  SwerveModuleTelemetryConfig& WithCustom(DoubleTelemetryField field, bool value);
  /** @overload */
  SwerveModuleTelemetryConfig& WithCustom(StructTelemetryField field, bool value);
  /** @overload */
  SwerveModuleTelemetryConfig& WithCustom(const std::vector<DoubleTelemetryField>& fields,
                                          bool value);
  /** @overload */
  SwerveModuleTelemetryConfig& WithCustom(const std::vector<StructTelemetryField>& fields,
                                          bool value);

 private:
  std::optional<std::string> m_dataLogName;
  bool m_nt4Telemetry{true};

  std::unordered_map<DoubleTelemetryField, DoubleTelemetry<DoubleTelemetryField>> m_doubleFields;
  std::unordered_map<StructTelemetryField,
                     StructTelemetry<frc::SwerveModuleState, StructTelemetryField>>
      m_structFields;
};

}  // namespace yams::telemetry
