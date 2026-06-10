// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

#pragma once

#include <networktables/DoubleTopic.h>
#include <networktables/NetworkTable.h>

#include <memory>
#include <optional>
#include <string>

namespace yams::motorcontrollers {
class SmartMotorController;
}

namespace yams::telemetry {

/**
 * Mechanism-level telemetry coordinator.
 *
 * Manages the root NT4 tables under "Mechanisms/<name>" and "Tuning/<name>",
 * publishes loop timing, and forwards SetupTelemetry to the underlying
 * SmartMotorController.
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

  /** Publish loop time (time since the last call) to NT4. */
  void UpdateLoopTime();

  /** @return The data NT4 table ("Mechanisms/<name>"). */
  std::shared_ptr<nt::NetworkTable> GetDataTable() const;

  /** @return The tuning NT4 table ("Tuning/<name>"). */
  std::shared_ptr<nt::NetworkTable> GetTuningTable() const;

 private:
  void SetupLoopTime();

  std::shared_ptr<nt::NetworkTable> m_networkTable;
  std::shared_ptr<nt::NetworkTable> m_tuningNetworkTable;
  std::optional<nt::DoublePublisher> m_loopTimePublisher;
  double m_prevTimestamp{0.0};
};

}  // namespace yams::telemetry
