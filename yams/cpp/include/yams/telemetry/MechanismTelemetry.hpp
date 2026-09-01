// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

#pragma once

#include <wpi/datalog/DataLog.hpp>
#include <wpi/nt/DoubleTopic.hpp>
#include <wpi/nt/NetworkTable.hpp>
#include <wpi/nt/StructArrayTopic.hpp>
#include <wpi/nt/StructTopic.hpp>
#include <wpi/system/DataLogManager.hpp>
#include <wpi/util/json.hpp>

#include <functional>
#include <memory>
#include <optional>
#include <span>
#include <string>

namespace yams::motorcontrollers {
class SmartMotorController;
}

namespace yams::telemetry {

/**
 * Mechanism-level telemetry coordinator.
 *
 * Manages the root NT4 tables under "Mechanisms/<name>" and "Tuning/<name>",
 * publishes loop timing, and is the primary coordinator of mechanism telemetry:
 * every field published through PublishDouble()/PublishStruct()/PublishStructArray(),
 * and every SmartMotorController wired in through SetupTelemetry() or
 * AddMotorController(), shares this mechanism's tables and (optionally) DataLog prefix.
 */
class MechanismTelemetry {
 public:
  /**
   * Set up NT4 tables and SMC telemetry for a named mechanism.
   *
   * @param mechanismName  Telemetry name (sub-table key).
   * @param motorController Motor controller to set up telemetry for.
   */
  void SetupTelemetry(const std::string& mechanismName,
                      motorcontrollers::SmartMotorController& motorController);

  /**
   * Set up NT4 tables for a named mechanism (no motor controller).
   *
   * @param mechanismName Telemetry name (sub-table key).
   */
  void SetupTelemetry(const std::string& mechanismName);

  /**
   * Set up NT4 tables for a named mechanism, additionally logging every field published
   * through PublishDouble(), PublishStruct(), and PublishStructArray() to a WPILib DataLog
   * under the given name.
   *
   * @param mechanismName Telemetry name (sub-table key).
   * @param dataLogName   DataLog entry name prefix for this mechanism's fields.
   */
  void SetupTelemetry(const std::string& mechanismName, const std::string& dataLogName);

  /**
   * Wire an additional SmartMotorController's telemetry into a named child table of this
   * mechanism, without disturbing the mechanism's own data/tuning table references or
   * loop-time publisher.
   *
   * Use this for mechanisms driven by more than one SmartMotorController (e.g. a swerve
   * module's drive and azimuth motors, or a differential mechanism's left and right motors)
   * so that every motor shares the single SetupTelemetry() call and loop timer for the
   * mechanism, instead of each motor re-running SetupTelemetry(name, motorController) and
   * clobbering the previous motor's table/loop-time state.
   *
   * @param subTableName    Name of the child table to nest this motor's telemetry under,
   *                        relative to the mechanism's data and tuning tables.
   * @param motorController SmartMotorController to set up telemetry for.
   */
  void AddMotorController(const std::string& subTableName,
                          motorcontrollers::SmartMotorController& motorController);

  /**
   * Publish a mechanism-level double field under this mechanism's data table, and — if this
   * mechanism was set up with a DataLog name via SetupTelemetry(name, dataLogName) — to the
   * DataLog as well. For fields tied to a SmartMotorController use SetupTelemetry(name, smc)
   * or AddMotorController() instead.
   *
   * @param key  NetworkTables key, relative to this mechanism's data table.
   * @param unit Unit metadata for the field (consumed by Advantage Scope/Elastic), or an
   *             empty string for none.
   * @return Callable to push values to.
   */
  std::function<void(double)> PublishDouble(const std::string& key, const std::string& unit = "");

  /**
   * Publish a mechanism-level struct field under this mechanism's data table, and — if this
   * mechanism was set up with a DataLog name via SetupTelemetry(name, dataLogName) — to the
   * DataLog as well.
   *
   * @tparam T    Type of the published value.
   * @param key   NetworkTables key, relative to this mechanism's data table.
   * @return Callable to push values to.
   */
  template <typename T>
  std::function<void(const T&)> PublishStruct(const std::string& key) {
    auto publisher = std::make_shared<wpi::nt::StructPublisher<T>>(
        m_networkTable->template GetStructTopic<T>(key).Publish());
    std::shared_ptr<wpi::log::StructLogEntry<T>> logEntry;
    if (m_dataLogName) {
      logEntry = std::make_shared<wpi::log::StructLogEntry<T>>(wpi::DataLogManager::GetLog(),
                                                                *m_dataLogName + "/" + key);
    }
    return [publisher, logEntry](const T& value) {
      publisher->Set(value);
      if (logEntry) logEntry->Append(value);
    };
  }

  /**
   * Publish a mechanism-level struct array field under this mechanism's data table, and — if
   * this mechanism was set up with a DataLog name via SetupTelemetry(name, dataLogName) — to
   * the DataLog as well.
   *
   * @tparam T    Type of the published array elements.
   * @param key   NetworkTables key, relative to this mechanism's data table.
   * @return Callable to push values to.
   */
  template <typename T>
  std::function<void(std::span<const T>)> PublishStructArray(const std::string& key) {
    auto publisher = std::make_shared<wpi::nt::StructArrayPublisher<T>>(
        m_networkTable->template GetStructArrayTopic<T>(key).Publish());
    std::shared_ptr<wpi::log::StructArrayLogEntry<T>> logEntry;
    if (m_dataLogName) {
      logEntry = std::make_shared<wpi::log::StructArrayLogEntry<T>>(
          wpi::DataLogManager::GetLog(), *m_dataLogName + "/" + key);
    }
    return [publisher, logEntry](std::span<const T> value) {
      publisher->Set(value);
      if (logEntry) logEntry->Append(value);
    };
  }

  /** Publish loop time (time since the last call) to NT4. */
  void UpdateLoopTime();

  /** @return The data NT4 table ("Mechanisms/<name>"). */
  std::shared_ptr<wpi::nt::NetworkTable> GetDataTable() const;

  /** @return The tuning NT4 table ("Tuning/<name>"). */
  std::shared_ptr<wpi::nt::NetworkTable> GetTuningTable() const;

 private:
  void SetupLoopTime();

  std::shared_ptr<wpi::nt::NetworkTable> m_networkTable;
  std::shared_ptr<wpi::nt::NetworkTable> m_tuningNetworkTable;
  std::optional<wpi::nt::DoublePublisher> m_loopTimePublisher;
  double m_prevTimestamp{0.0};
  std::optional<std::string> m_dataLogName;
};

}  // namespace yams::telemetry
