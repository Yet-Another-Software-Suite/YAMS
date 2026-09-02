// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

#pragma once

#include <networktables/NetworkTable.h>

#include <cstddef>
#include <memory>

#include "yams/telemetry/SwerveDriveTelemetryConfig.hpp"

namespace yams::mechanisms::swerve {
template <std::size_t NumModules>
class SwerveDrive;
}

namespace yams::telemetry {

/**
 * Swerve drive telemetry.
 *
 * Publishes pose, chassis speeds, gyro angle, and module states to NetworkTables and/or a WPILib
 * DataLog, and exposes auto-align (DriveToPose) PID gains plus a live-tunable target pose for
 * on-the-fly tuning. Also drives per-module telemetry setup via SwerveModule::SetupTelemetry().
 *
 * SetupTelemetry(), Publish(), and ApplyTuningValues() are templated on the SwerveDrive's module
 * count so that this class itself does not need to be templated; their bodies are defined at the
 * bottom of SwerveDrive.hpp (after SwerveDrive<NumModules> is fully defined), which is the only
 * place both this class's members and SwerveDrive<NumModules>'s interface are simultaneously
 * complete.
 *
 * This class is managed internally by SwerveDrive. You do not normally instantiate it directly;
 * instead configure telemetry through SwerveDriveConfig::WithTelemetry(...).
 */
class SwerveDriveTelemetry {
 public:
  using DoubleTelemetryField = SwerveDriveTelemetryConfig::DoubleTelemetryField;
  using BooleanTelemetryField = SwerveDriveTelemetryConfig::BooleanTelemetryField;
  using StructTelemetryField = SwerveDriveTelemetryConfig::StructTelemetryField;
  using StructArrayTelemetryField = SwerveDriveTelemetryConfig::StructArrayTelemetryField;

  /**
   * Create SwerveDrive telemetry for logging in NetworkTables and DataLog.
   *
   * @param config Telemetry configuration selecting which fields to publish.
   */
  explicit SwerveDriveTelemetry(SwerveDriveTelemetryConfig config);

  /**
   * Setup telemetry for the drive; also sets up telemetry for each module.
   *
   * @param drive Drive to set up telemetry for.
   */
  template <std::size_t NumModules>
  void SetupTelemetry(mechanisms::swerve::SwerveDrive<NumModules>* drive);

  /**
   * Publish telemetry to NetworkTables and DataLog.
   *
   * @param drive Drive to publish telemetry from.
   */
  template <std::size_t NumModules>
  void Publish(mechanisms::swerve::SwerveDrive<NumModules>* drive);

  /**
   * Apply the tuning values from NetworkTables to the drive.
   *
   * @param drive Drive to control.
   */
  template <std::size_t NumModules>
  void ApplyTuningValues(mechanisms::swerve::SwerveDrive<NumModules>* drive);

  /** Release all NT4 pub/sub handles. */
  void Close();

 private:
  SwerveDriveTelemetryConfig m_config;

  std::shared_ptr<nt::NetworkTable> m_dataTable;
  std::shared_ptr<nt::NetworkTable> m_tuningTable;
};

}  // namespace yams::telemetry
