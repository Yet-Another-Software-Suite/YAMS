// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

#pragma once

#include <optional>
#include <string>
#include <unordered_map>

#include "yams/motorcontrollers/SmartMotorControllerConfig.hpp"
#include "yams/telemetry/SmartMotorControllerTelemetry.hpp"

namespace yams::telemetry {

/**
 * Configures which telemetry fields are published for a SmartMotorController.
 *
 * Use WithTelemetryVerbosity() for a preset bundle of fields, or enable individual
 * fields with the With*() methods.  Pass to SmartMotorController::WithTelemetry().
 *
 * ### Quick preset via SmartMotorControllerConfig
 * @code{.cpp}
 * // The simplest approach: name the motor in SmartMotorControllerConfig.
 * // This enables HIGH-verbosity telemetry automatically.
 * SmartMotorControllerConfig cfg;
 * cfg.WithTelemetry("ArmMotor");                       // HIGH verbosity (default)
 * cfg.WithTelemetry("ArmMotor", Cfg::TelemetryVerbosity::MID);  // or choose a level
 * @endcode
 *
 * ### Fine-grained override via SmartMotorController::WithTelemetry()
 * @code{.cpp}
 * // After constructing the SmartMotorController, supply an explicit config
 * // to override the preset chosen by SmartMotorControllerConfig::WithTelemetry().
 * using Cfg = yams::motorcontrollers::SmartMotorControllerConfig;
 * using TelCfg = yams::telemetry::SmartMotorControllerTelemetryConfig;
 *
 * ctre::phoenix6::hardware::TalonFX talon{1};
 * TalonFXWrapper smc{talon, frc::DCMotor::KrakenX60(1), cfg};
 *
 * smc.WithTelemetry(
 *     TelCfg{}
 *         .WithTelemetryVerbosity(Cfg::TelemetryVerbosity::MID)
 *         .WithMechanismPosition()
 *         .WithMechanismVelocity()
 *         .WithStatorCurrent()
 *         .WithOutputVoltage()
 *         .WithDataLogName("/Robot/Arm/Motor")
 *         .WithoutNetworkTables());   // suppress NT4 during matches
 * @endcode
 *
 * ### Logging-only (no NT4) with a custom field selection
 * @code{.cpp}
 * smc.WithTelemetry(
 *     TelCfg{}
 *         .WithRotorPosition()
 *         .WithRotorVelocity()
 *         .WithTemperature()
 *         .WithDataLogName("/Robot/Shooter/Motor")
 *         .WithoutNetworkTables());
 * @endcode
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

  /** Enable mechanism lower-limit logging. @return *this for chaining. */
  SmartMotorControllerTelemetryConfig& WithMechanismLowerLimit();
  /** Enable mechanism upper-limit logging. @return *this for chaining. */
  SmartMotorControllerTelemetryConfig& WithMechanismUpperLimit();
  /** Enable temperature-limit (over-temp flag) logging. @return *this for chaining. */
  SmartMotorControllerTelemetryConfig& WithTemperatureLimit();
  /** Enable velocity-control active flag logging. @return *this for chaining. */
  SmartMotorControllerTelemetryConfig& WithVelocityControl();
  /** Enable elevator feedforward active flag logging. @return *this for chaining. */
  SmartMotorControllerTelemetryConfig& WithElevatorFeedforward();
  /** Enable arm feedforward active flag logging. @return *this for chaining. */
  SmartMotorControllerTelemetryConfig& WithArmFeedforward();
  /** Enable simple motor feedforward active flag logging. @return *this for chaining. */
  SmartMotorControllerTelemetryConfig& WithSimpleFeedforward();
  /** Enable motion profile active flag logging. @return *this for chaining. */
  SmartMotorControllerTelemetryConfig& WithMotionProfile();
  /** Enable setpoint position logging (read-only). @return *this for chaining. */
  SmartMotorControllerTelemetryConfig& WithSetpointPosition();
  /** Enable setpoint velocity logging (read-only). @return *this for chaining. */
  SmartMotorControllerTelemetryConfig& WithSetpointVelocity();
  /** Enable output voltage logging. @return *this for chaining. */
  SmartMotorControllerTelemetryConfig& WithOutputVoltage();
  /** Enable stator current logging. @return *this for chaining. */
  SmartMotorControllerTelemetryConfig& WithStatorCurrent();
  /** Enable motor temperature logging. @return *this for chaining. */
  SmartMotorControllerTelemetryConfig& WithTemperature();
  /** Enable linear measurement position logging. @return *this for chaining. */
  SmartMotorControllerTelemetryConfig& WithMeasurementPosition();
  /** Enable linear measurement velocity logging. @return *this for chaining. */
  SmartMotorControllerTelemetryConfig& WithMeasurementVelocity();
  /** Enable mechanism angular position logging. @return *this for chaining. */
  SmartMotorControllerTelemetryConfig& WithMechanismPosition();
  /** Enable mechanism angular velocity logging. @return *this for chaining. */
  SmartMotorControllerTelemetryConfig& WithMechanismVelocity();
  /** Enable raw rotor position logging. @return *this for chaining. */
  SmartMotorControllerTelemetryConfig& WithRotorPosition();
  /** Enable raw rotor velocity logging. @return *this for chaining. */
  SmartMotorControllerTelemetryConfig& WithRotorVelocity();

  // ---- Accessors ----------------------------------------------------------

  /** @return Optional DataLog prefix path if configured. */
  std::optional<std::string> GetDataLogName() const;
  /** @return true if NT4 output is enabled. */
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
