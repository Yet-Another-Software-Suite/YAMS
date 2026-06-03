// Copyright (c) 2026 YAMS Contributors
// SPDX-License-Identifier: LGPL-3.0-or-later

#pragma once

#include <optional>
#include <string>
#include <unordered_map>

#include "yams/motorcontrollers/SmartMotorControllerConfig.h"
#include "yams/telemetry/SmartMotorControllerTelemetry.h"

namespace yams::telemetry {

/**
 * Configures which telemetry fields are published for a SmartMotorController.
 *
 * Use WithTelemetryVerbosity() for a preset bundle of fields, or enable individual
 * fields with the With*() methods.  Pass to SmartMotorController::SetupTelemetry().
 */
class SmartMotorControllerTelemetryConfig {
 public:
  SmartMotorControllerTelemetryConfig();

  using TelemetryVerbosity = motorcontrollers::SmartMotorControllerConfig::TelemetryVerbosity;

  /**
   * Set the DataLog prefix for offline recording.
   *
   * @param dataLogName Prefix path inside the log.
   * @return *this for chaining.
   */
  SmartMotorControllerTelemetryConfig& WithDataLogName(const std::string& dataLogName);

  /**
   * Enable or disable NT4 output.
   *
   * @param enabled false to suppress NT4 publishing (e.g. during matches).
   * @return *this for chaining.
   */
  SmartMotorControllerTelemetryConfig& WithNetworkTables(bool enabled);

  /** Disable NT4 output. @return *this for chaining. */
  SmartMotorControllerTelemetryConfig& WithoutNetworkTables();

  /**
   * Enable a preset bundle of fields based on verbosity level.
   *
   * HIGH includes all MID and LOW fields plus tunable gain entries.
   * MID adds voltage and current fields.
   * LOW enables the core position/velocity/setpoint fields.
   *
   * @param verbosity Verbosity level to apply.
   * @return *this for chaining.
   */
  SmartMotorControllerTelemetryConfig& WithTelemetryVerbosity(TelemetryVerbosity verbosity);

  // ---- Individual field enable methods ------------------------------------

  SmartMotorControllerTelemetryConfig& WithMechanismLowerLimit();
  SmartMotorControllerTelemetryConfig& WithMechanismUpperLimit();
  SmartMotorControllerTelemetryConfig& WithTemperatureLimit();
  SmartMotorControllerTelemetryConfig& WithVelocityControl();
  SmartMotorControllerTelemetryConfig& WithElevatorFeedforward();
  SmartMotorControllerTelemetryConfig& WithArmFeedforward();
  SmartMotorControllerTelemetryConfig& WithSimpleFeedforward();
  SmartMotorControllerTelemetryConfig& WithMotionProfile();
  SmartMotorControllerTelemetryConfig& WithSetpointPosition();
  SmartMotorControllerTelemetryConfig& WithSetpointVelocity();
  SmartMotorControllerTelemetryConfig& WithOutputVoltage();
  SmartMotorControllerTelemetryConfig& WithStatorCurrent();
  SmartMotorControllerTelemetryConfig& WithTemperature();
  SmartMotorControllerTelemetryConfig& WithMeasurementPosition();
  SmartMotorControllerTelemetryConfig& WithMeasurementVelocity();
  SmartMotorControllerTelemetryConfig& WithMechanismPosition();
  SmartMotorControllerTelemetryConfig& WithMechanismVelocity();
  SmartMotorControllerTelemetryConfig& WithRotorPosition();
  SmartMotorControllerTelemetryConfig& WithRotorVelocity();

  // ---- Accessors ----------------------------------------------------------

  std::optional<std::string> GetDataLogName() const;
  bool GetNT4Enabled() const;

  /**
   * Apply motor-controller-specific constraints (disable unsupported fields,
   * set default values from config) and return the configured field maps.
   *
   * @param smc Motor controller to configure for.
   */
  std::unordered_map<DoubleTelemetryField, DoubleTelemetry>& GetDoubleFields(
      motorcontrollers::SmartMotorController& smc);
  std::unordered_map<BooleanTelemetryField, BooleanTelemetry>& GetBoolFields(
      motorcontrollers::SmartMotorController& smc);

 private:
  std::optional<std::string> m_dataLogName;
  bool m_nt4Telemetry{true};

  std::unordered_map<DoubleTelemetryField, DoubleTelemetry> m_doubleFields;
  std::unordered_map<BooleanTelemetryField, BooleanTelemetry> m_boolFields;
};

}  // namespace yams::telemetry
