// Copyright (c) 2026 YAMS Contributors
// SPDX-License-Identifier: LGPL-3.0-or-later

#pragma once

#include <networktables/BooleanTopic.h>
#include <networktables/DoubleTopic.h>
#include <networktables/NetworkTable.h>
#include <wpi/DataLog.h>

#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

// Forward declarations
namespace yams::motorcontrollers {
class SmartMotorController;
class SmartMotorControllerConfig;
}  // namespace yams::motorcontrollers

namespace yams::telemetry {

// ---- Field enums ------------------------------------------------------------

enum class DoubleTelemetryField {
  ExponentialProfileKV,
  ExponentialProfileKA,
  ExponentialProfileMaxInput,
  TrapezoidalProfileMaxVelocity,
  TrapezoidalProfileMaxAcceleration,
  TrapezoidalProfileMaxJerk,
  TunableClosedLoopControllerSlot,
  ActiveClosedLoopControllerSlot,
  kS,
  kV,
  kA,
  kG,
  kP,
  kI,
  kD,
  TunableSetpointPosition,
  SetpointPosition,
  TunableSetpointVelocity,
  SetpointVelocity,
  OutputVoltage,
  StatorCurrent,
  StatorCurrentLimit,
  SupplyCurrent,
  SupplyCurrentLimit,
  MotorTemperature,
  MeasurementPosition,
  MeasurementVelocity,
  MeasurementAcceleration,
  MeasurementLowerLimit,
  MeasurementUpperLimit,
  MechanismPosition,
  MechanismVelocity,
  MechanismAcceleration,
  MechanismLowerLimit,
  MechanismUpperLimit,
  RotorPosition,
  RotorVelocity,
  ExternalEncoderPosition,
  ExternalEncoderVelocity,
  ClosedloopRampRate,
  OpenloopRampRate,
};

enum class BooleanTelemetryField {
  MechanismUpperLimit,
  MechanismLowerLimit,
  TemperatureLimit,
  VelocityControl,
  ElevatorFeedForward,
  ArmFeedForward,
  SimpleMotorFeedForward,
  MotionProfile,
  MotorInversion,
  EncoderInversion,
};

// ---- UnsupportedTelemetryFields ---------------------------------------------

struct UnsupportedTelemetryFields {
  std::optional<std::vector<BooleanTelemetryField>> boolFields;
  std::optional<std::vector<DoubleTelemetryField>> doubleFields;
};

// ---- DoubleTelemetry --------------------------------------------------------

/**
 * NT4 pub/sub wrapper for a single double-valued telemetry field.
 *
 * Tunable fields use a subscriber so that values changed in NetworkTables are
 * reflected back into the motor controller via ApplyTuningValues.  Read-only
 * fields only publish.
 */
class DoubleTelemetry {
 public:
  DoubleTelemetry(std::string key, double defaultVal, DoubleTelemetryField field, bool tunable,
                  std::string unit);

  void SetDefaultValue(double value);
  void SetupNetworkTables(std::shared_ptr<nt::NetworkTable> dataTable,
                          std::shared_ptr<nt::NetworkTable> tuningTable);
  void SetupNetworkTable(std::shared_ptr<nt::NetworkTable> dataTable);
  void SetupDataLog(const std::string& prefix);
  void TransformUnit(const motorcontrollers::SmartMotorControllerConfig& cfg);

  bool Set(double value);
  double Get() const;
  bool IsTunable();

  void Enable() { m_enabled = true; }
  void Disable() { m_enabled = false; }
  void Display(bool state) { m_enabled = state; }
  DoubleTelemetryField GetField() const { return m_field; }
  bool IsEnabled() const { return m_enabled; }

  void Close();

 private:
  DoubleTelemetryField m_field;
  std::string m_key;
  bool m_tunable;
  bool m_enabled{false};
  std::string m_unit;
  double m_defaultValue;
  double m_cachedValue;

  std::optional<nt::DoublePublisher> m_publisher;
  std::optional<nt::DoubleSubscriber> m_subscriber;
  std::optional<nt::DoublePublisher> m_subPublisher;  // tunable: publish + subscribe
  std::optional<wpi::log::DoubleLogEntry> m_dataLogEntry;

  std::shared_ptr<nt::NetworkTable> m_tuningTable;
  std::shared_ptr<nt::NetworkTable> m_dataTable;
};

// ---- BooleanTelemetry -------------------------------------------------------

/**
 * NT4 pub/sub wrapper for a single boolean-valued telemetry field.
 */
class BooleanTelemetry {
 public:
  BooleanTelemetry(std::string key, bool defaultVal, BooleanTelemetryField field, bool tunable);

  void SetDefaultValue(bool value);
  void SetupNetworkTables(std::shared_ptr<nt::NetworkTable> dataTable,
                          std::shared_ptr<nt::NetworkTable> tuningTable);
  void SetupNetworkTable(std::shared_ptr<nt::NetworkTable> dataTable);
  void SetupDataLog(const std::string& prefix);

  bool Set(bool value);
  bool Get() const;
  bool IsTunable();

  void Enable() { m_enabled = true; }
  void Disable() { m_enabled = false; }
  void Display(bool state) { m_enabled = state; }
  BooleanTelemetryField GetField() const { return m_field; }
  bool IsEnabled() const { return m_enabled; }

  void Close();

 private:
  BooleanTelemetryField m_field;
  std::string m_key;
  bool m_tunable;
  bool m_enabled{false};
  bool m_defaultValue;
  bool m_cachedValue;

  std::optional<nt::BooleanPublisher> m_publisher;
  std::optional<nt::BooleanSubscriber> m_subscriber;
  std::optional<nt::BooleanPublisher> m_pubSub;  // tunable: publish + subscribe
  std::optional<wpi::log::BooleanLogEntry> m_dataLogEntry;

  std::shared_ptr<nt::NetworkTable> m_tuningTable;
  std::shared_ptr<nt::NetworkTable> m_dataTable;
};

// ---- SmartMotorControllerTelemetry ------------------------------------------

/**
 * Telemetry coordinator for a SmartMotorController.
 *
 * Owns all DoubleTelemetry and BooleanTelemetry pub/sub handles.  Call
 * SetupTelemetry() once during initialization, then Publish() and
 * ApplyTuningValues() each loop.
 */
class SmartMotorControllerTelemetry {
 public:
  /**
   * Initialize telemetry pub/sub using the supplied tables and config.
   *
   * @param smc          Motor controller whose fields drive which entries are enabled.
   * @param publishTable NT4 table for read-only sensor data.
   * @param tuningTable  NT4 table for live-tunable gains.
   * @param doubleFields Configured DoubleTelemetry map from SmartMotorControllerTelemetryConfig.
   * @param boolFields   Configured BooleanTelemetry map from SmartMotorControllerTelemetryConfig.
   * @param nt4Enabled   Whether to publish to NT4.
   * @param dataLogName  Optional DataLog prefix.
   */
  void SetupTelemetry(
      motorcontrollers::SmartMotorController& smc,
      std::shared_ptr<nt::NetworkTable> publishTable,
      std::shared_ptr<nt::NetworkTable> tuningTable,
      std::unordered_map<DoubleTelemetryField, DoubleTelemetry>& doubleFields,
      std::unordered_map<BooleanTelemetryField, BooleanTelemetry>& boolFields, bool nt4Enabled,
      std::optional<std::string> dataLogName);

  /**
   * Publish current motor controller state to NetworkTables / DataLog.
   *
   * @param smc Motor controller to read from.
   */
  void Publish(motorcontrollers::SmartMotorController& smc);

  /**
   * Read tunable fields from NetworkTables and apply changed values to smc.
   *
   * @param smc Motor controller to tune.
   */
  void ApplyTuningValues(motorcontrollers::SmartMotorController& smc);

  /** True if any live-setpoint tuning fields are enabled. */
  bool TuningEnabled() const;

  /** Release all NT4 pub/sub handles. */
  void Close();

 private:
  std::shared_ptr<nt::NetworkTable> m_dataTable;
  std::shared_ptr<nt::NetworkTable> m_tuningTable;

  std::unordered_map<DoubleTelemetryField, DoubleTelemetry>* m_doubleFields{nullptr};
  std::unordered_map<BooleanTelemetryField, BooleanTelemetry>* m_boolFields{nullptr};
};

}  // namespace yams::telemetry
