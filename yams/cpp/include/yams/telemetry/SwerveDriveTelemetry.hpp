// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

#pragma once

#include <cstddef>
#include <memory>
#include <wpi/nt/NetworkTable.hpp>

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

  /**
   * Stop applying a tuning voltage, as when the tuning command ends, and hold every wheel still
   * where it is with its closed-loop controller.
   *
   * @param drive Drive being tuned.
   */
  template <std::size_t NumModules>
  void StopTuning(mechanisms::swerve::SwerveDrive<NumModules>* drive);

  /** Release all NT4 pub/sub handles. */
  void Close();

 private:
  /**
   * Switch the drive or azimuth motors between a tuning voltage and their closed-loop controllers.
   * A closed-loop controller run on the SystemCore would replace the voltage on its next loop, so
   * it is stopped while the voltage is applied.
   *
   * @param drive Drive being tuned.
   * @param azimuth True for the azimuth motors, false for the drive motors.
   * @param on True to apply a voltage, false to go back to the closed-loop controller.
   */
  template <std::size_t NumModules>
  void SetVoltageControl(mechanisms::swerve::SwerveDrive<NumModules>* drive, bool azimuth, bool on);

  /**
   * Hold every wheel still where it is, with its closed-loop controllers.
   *
   * @param drive Drive being tuned.
   */
  template <std::size_t NumModules>
  void HoldWheels(mechanisms::swerve::SwerveDrive<NumModules>* drive);

  SwerveDriveTelemetryConfig m_config;
  /** Whether the drive motors are driven at a tuning voltage, without their closed-loop
   *  controllers. */
  bool m_driveVoltageControl = false;
  /** Whether the azimuth motors are driven at a tuning voltage, without their closed-loop
   *  controllers. */
  bool m_azimuthVoltageControl = false;
  /** The tuning modes on in the last loop, to send the setpoints again when they change, or -1
   *  for none yet. */
  int m_lastModes = -1;

  std::shared_ptr<wpi::nt::NetworkTable> m_dataTable;
  std::shared_ptr<wpi::nt::NetworkTable> m_tuningTable;
};

}  // namespace yams::telemetry
