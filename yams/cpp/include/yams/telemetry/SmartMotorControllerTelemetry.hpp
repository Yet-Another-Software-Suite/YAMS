// Copyright (c) 2026 Yet Another Software Suite
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

/** Identifies a double-valued telemetry field published by a SmartMotorController. */
enum class DoubleTelemetryField {
  ExponentialProfileKV,               ///< Exponential profile kV constant (tunable).
  ExponentialProfileKA,               ///< Exponential profile kA constant (tunable).
  ExponentialProfileMaxInput,         ///< Exponential profile maximum input voltage (tunable).
  TrapezoidalProfileMaxVelocity,      ///< Trapezoidal profile max velocity — RPM or m/s (tunable).
  TrapezoidalProfileMaxAcceleration,  ///< Trapezoidal profile max acceleration — RPM/s or m/s²
                                      ///< (tunable).
  TrapezoidalProfileMaxJerk,          ///< Trapezoidal profile max jerk — RPM/s² (tunable).
  TunableClosedLoopControllerSlot,    ///< Live-tunable closed-loop gain slot selector.
  ActiveClosedLoopControllerSlot,     ///< Currently active closed-loop gain slot (read-only).
  kS,                                 ///< Static friction feedforward gain kS (tunable).
  kV,                                 ///< Velocity feedforward gain kV (tunable).
  kA,                                 ///< Acceleration feedforward gain kA (tunable).
  kG,                                 ///< Gravity feedforward gain kG (tunable).
  kP,                                 ///< Proportional feedback gain kP (tunable).
  kI,                                 ///< Integral feedback gain kI (tunable).
  kD,                                 ///< Derivative feedback gain kD (tunable).
  TunableSetpointPosition,            ///< Live-tunable position setpoint — degrees or meters.
  SetpointPosition,                   ///< Read-only position setpoint — rotations or meters.
  TunableSetpointVelocity,            ///< Live-tunable velocity setpoint — RPM or m/s.
  SetpointVelocity,                   ///< Read-only velocity setpoint — RPS or m/s.
  OutputVoltage,                      ///< Voltage currently applied to the motor (V).
  StatorCurrent,                      ///< Stator (output) current draw (A).
  StatorCurrentLimit,                 ///< Configured stator current limit (A, tunable).
  SupplyCurrent,                      ///< Supply (input) current draw (A).
  SupplyCurrentLimit,                 ///< Configured supply current limit (A, tunable).
  MotorTemperature,                   ///< Motor controller temperature (°F).
  MeasurementPosition,                ///< Linear measurement position (m).
  MeasurementVelocity,                ///< Linear measurement velocity (m/s).
  MeasurementAcceleration,            ///< Linear measurement acceleration (m/s²).
  MeasurementLowerLimit,              ///< Configured lower linear soft limit (m, tunable).
  MeasurementUpperLimit,              ///< Configured upper linear soft limit (m, tunable).
  MechanismPosition,                  ///< Mechanism position in rotations.
  MechanismVelocity,                  ///< Mechanism velocity in rotations per second.
  MechanismAcceleration,              ///< Mechanism acceleration in rotations per second squared.
  MechanismLowerLimit,                ///< Configured lower angular soft limit (degrees, tunable).
  MechanismUpperLimit,                ///< Configured upper angular soft limit (degrees, tunable).
  RotorPosition,                      ///< Raw rotor position in rotations.
  RotorVelocity,                      ///< Raw rotor velocity in rotations per second.
  ExternalEncoderPosition,            ///< External (absolute) encoder position in rotations.
  ExternalEncoderVelocity,  ///< External (absolute) encoder velocity in rotations per second.
  ClosedloopRampRate,       ///< Closed-loop duty-cycle ramp rate (s, tunable).
  OpenloopRampRate,         ///< Open-loop duty-cycle ramp rate (s, tunable).
};

/** Identifies a boolean-valued telemetry field published by a SmartMotorController. */
enum class BooleanTelemetryField {
  MechanismUpperLimit,     ///< true when the mechanism is at or past its upper angular limit.
  MechanismLowerLimit,     ///< true when the mechanism is at or past its lower angular limit.
  TemperatureLimit,        ///< true when the motor temperature has reached the cutoff threshold.
  VelocityControl,         ///< true when a velocity setpoint is active.
  ElevatorFeedForward,     ///< true when an elevator feedforward model is configured.
  ArmFeedForward,          ///< true when an arm feedforward model is configured.
  SimpleMotorFeedForward,  ///< true when a simple motor feedforward model is configured.
  MotionProfile,           ///< true when a motion profile (trapezoidal or exponential) is active.
  MotorInversion,          ///< true when the motor output direction is inverted (tunable).
  EncoderInversion,        ///< true when the encoder count direction is inverted (tunable).
};

// ---- UnsupportedTelemetryFields ---------------------------------------------

/**
 * Set of telemetry fields not supported by a particular SmartMotorController implementation.
 *
 * Returned by GetUnsupportedTelemetryFields() so the telemetry system can disable
 * entries that the hardware cannot provide (e.g. SPARK controllers have no supply current).
 */
struct UnsupportedTelemetryFields {
  std::optional<std::vector<BooleanTelemetryField>> boolFields;   ///< Unsupported boolean fields.
  std::optional<std::vector<DoubleTelemetryField>> doubleFields;  ///< Unsupported double fields.
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
  /**
   * Construct a DoubleTelemetry entry.
   *
   * @param key        NT4 table key for this field.
   * @param defaultVal Default numeric value published on startup.
   * @param field      DoubleTelemetryField identifier.
   * @param tunable    true if the field should be placed in the tuning table with a subscriber.
   * @param unit       Unit string embedded in the NT4 topic metadata (e.g. "amps", "volts").
   */
  DoubleTelemetry(std::string key, double defaultVal, DoubleTelemetryField field, bool tunable,
                  std::string unit);

  /**
   * Override the default published value (e.g. to seed a limit from the config).
   *
   * @param value New default value.
   */
  void SetDefaultValue(double value);

  /**
   * Create the NT4 publisher (and subscriber if tunable) under the given tables.
   *
   * @param dataTable   NT4 table for read-only sensor data.
   * @param tuningTable NT4 table for live-tunable gains.
   */
  void SetupNetworkTables(std::shared_ptr<nt::NetworkTable> dataTable,
                          std::shared_ptr<nt::NetworkTable> tuningTable);

  /**
   * Create a read-only NT4 publisher under the data table (no tuning table).
   *
   * @param dataTable NT4 table for sensor data.
   */
  void SetupNetworkTable(std::shared_ptr<nt::NetworkTable> dataTable);

  /**
   * Create a DataLog entry under the given prefix path.
   *
   * @param prefix DataLog path prefix (a trailing "/" is added if missing).
   */
  void SetupDataLog(const std::string& prefix);

  /**
   * Resolve the "tunable_*" / "position" / "velocity" unit placeholders to concrete unit strings
   * based on whether the config uses linear (meter) or angular (degree/rotation) control.
   *
   * @param cfg SmartMotorControllerConfig used to determine the active measurement space.
   */
  void TransformUnit(const motorcontrollers::SmartMotorControllerConfig& cfg);

  /**
   * Publish a value; returns false if the field is disabled or a tunable subscriber disagrees.
   *
   * @param value Value to publish.
   * @return true if the value was accepted and published.
   */
  bool Set(double value);

  /**
   * Read the current value from the subscriber (tunable) or return the default.
   *
   * @return Current field value.
   */
  double Get() const;

  /**
   * Return true if a tunable subscriber is active and the value has changed since the last call.
   *
   * @return true when the NT4 subscriber holds a value different from the cached value.
   */
  bool IsTunable();

  /** Enable this field so it is published / read each loop. */
  void Enable() { m_enabled = true; }
  /** Disable this field so publishing and reading are skipped. */
  void Disable() { m_enabled = false; }
  /**
   * Enable or disable this field.
   *
   * @param state true to enable.
   */
  void Display(bool state) { m_enabled = state; }
  /** @return The DoubleTelemetryField identifier for this entry. */
  DoubleTelemetryField GetField() const { return m_field; }
  /** @return true if this field is currently enabled. */
  bool IsEnabled() const { return m_enabled; }

  /** Release all NT4 publishers/subscribers and DataLog entries. */
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
  /**
   * Construct a BooleanTelemetry entry.
   *
   * @param key        NT4 table key for this field.
   * @param defaultVal Default boolean value published on startup.
   * @param field      BooleanTelemetryField identifier.
   * @param tunable    true if the field should be placed in the tuning table with a subscriber.
   */
  BooleanTelemetry(std::string key, bool defaultVal, BooleanTelemetryField field, bool tunable);

  /**
   * Override the default published value.
   *
   * @param value New default value.
   */
  void SetDefaultValue(bool value);

  /**
   * Create the NT4 publisher (and subscriber if tunable) under the given tables.
   *
   * @param dataTable   NT4 table for read-only status data.
   * @param tuningTable NT4 table for live-tunable fields.
   */
  void SetupNetworkTables(std::shared_ptr<nt::NetworkTable> dataTable,
                          std::shared_ptr<nt::NetworkTable> tuningTable);

  /**
   * Create a read-only NT4 publisher under the data table (no tuning table).
   *
   * @param dataTable NT4 table for status data.
   */
  void SetupNetworkTable(std::shared_ptr<nt::NetworkTable> dataTable);

  /**
   * Create a DataLog entry under the given prefix path.
   *
   * @param prefix DataLog path prefix (a trailing "/" is added if missing).
   */
  void SetupDataLog(const std::string& prefix);

  /**
   * Publish a value; returns false if a tunable subscriber disagrees.
   *
   * @param value Value to publish.
   * @return true if the value was accepted and published.
   */
  bool Set(bool value);

  /**
   * Read the current value from the subscriber (tunable) or return the default.
   *
   * @return Current field value.
   */
  bool Get() const;

  /**
   * Return true if a tunable subscriber is active and the value has changed since the last call.
   *
   * @return true when the NT4 subscriber holds a value different from the cached value.
   */
  bool IsTunable();

  /** Enable this field so it is published / read each loop. */
  void Enable() { m_enabled = true; }
  /** Disable this field so publishing and reading are skipped. */
  void Disable() { m_enabled = false; }
  /**
   * Enable or disable this field.
   *
   * @param state true to enable.
   */
  void Display(bool state) { m_enabled = state; }
  /** @return The BooleanTelemetryField identifier for this entry. */
  BooleanTelemetryField GetField() const { return m_field; }
  /** @return true if this field is currently enabled. */
  bool IsEnabled() const { return m_enabled; }

  /** Release all NT4 publishers/subscribers and DataLog entries. */
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
  void SetupTelemetry(motorcontrollers::SmartMotorController& smc,
                      std::shared_ptr<nt::NetworkTable> publishTable,
                      std::shared_ptr<nt::NetworkTable> tuningTable,
                      std::unordered_map<DoubleTelemetryField, DoubleTelemetry>& doubleFields,
                      std::unordered_map<BooleanTelemetryField, BooleanTelemetry>& boolFields,
                      bool nt4Enabled, std::optional<std::string> dataLogName);

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
