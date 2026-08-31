// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

#pragma once

#include <networktables/NetworkTable.h>

#include <memory>
#include <string>

#include "yams/telemetry/SwerveModuleTelemetryConfig.hpp"

namespace yams::mechanisms::swerve {
class SwerveModule;
}

namespace yams::telemetry {

/**
 * Swerve module telemetry.
 *
 * Publishes the module's raw absolute encoder angle (without offsets applied) and current
 * SwerveModuleState to NetworkTables and/or a WPILib DataLog, and wires the drive/azimuth
 * SmartMotorControllers' own telemetry under the same module subtable.
 *
 * This class is managed internally by SwerveModule. You do not normally instantiate it
 * directly; instead configure telemetry through
 * SwerveModuleConfig::WithTelemetry(name, SwerveModuleTelemetryConfig) or
 * SwerveModuleConfig::WithTelemetry(name, TelemetryVerbosity) before constructing the module.
 */
class SwerveModuleTelemetry {
 public:
  using DoubleTelemetryField = SwerveModuleTelemetryConfig::DoubleTelemetryField;
  using StructTelemetryField = SwerveModuleTelemetryConfig::StructTelemetryField;

  /**
   * Create SwerveModule telemetry for logging in NetworkTables and DataLog.
   *
   * @param config Telemetry configuration selecting which fields to publish.
   */
  explicit SwerveModuleTelemetry(SwerveModuleTelemetryConfig config);

  /**
   * Setup telemetry for the module.
   *
   * @param mechName Telemetry name of the parent SwerveDrive.
   * @param module   Module to use for telemetry.
   */
  void SetupTelemetry(const std::string& mechName, mechanisms::swerve::SwerveModule& module);

  /**
   * Publish telemetry to NetworkTables and DataLog.
   *
   * @param module Module to publish telemetry from.
   */
  void Publish(mechanisms::swerve::SwerveModule& module);

  /** Release all NT4 pub/sub handles. */
  void Close();

 private:
  SwerveModuleTelemetryConfig m_config;

  std::shared_ptr<nt::NetworkTable> m_dataTable;
  std::shared_ptr<nt::NetworkTable> m_tuningTable;
};

}  // namespace yams::telemetry
