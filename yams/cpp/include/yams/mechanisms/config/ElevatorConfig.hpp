// Copyright (c) 2026 YAMS Contributors
// SPDX-License-Identifier: LGPL-3.0-or-later

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <units/length.h>
#include <units/velocity.h>

#include <optional>
#include <string>

#include "yams/motorcontrollers/SmartMotorController.hpp"

namespace yams::mechanisms::config {

/**
 * Configuration class for an Elevator mechanism.
 *
 * Uses a fluent builder pattern; all With* methods return *this for chaining.
 */
class ElevatorConfig {
 public:
  ElevatorConfig() = default;

  // ---- Builder methods -------------------------------------------------------

  /**
   * Set the SmartMotorController driving the elevator.
   *
   * @param smc Pointer to the motor controller (must outlive this config).
   * @return *this for chaining.
   */
  ElevatorConfig& WithMotorController(motorcontrollers::SmartMotorController* smc);

  /**
   * Set the subsystem that owns the elevator (used for command requirements).
   *
   * @param subsystem Pointer to the subsystem.
   * @return *this for chaining.
   */
  ElevatorConfig& WithSubsystem(frc2::SubsystemBase* subsystem);

  /**
   * Set the NetworkTables / SmartDashboard name for telemetry.
   *
   * @param name Telemetry name string.
   * @return *this for chaining.
   */
  ElevatorConfig& WithTelemetryName(const std::string& name);

  /**
   * Set the encoder starting height (seeds the motor encoder on init).
   *
   * @param height Starting height of the carriage.
   * @return *this for chaining.
   */
  ElevatorConfig& WithStartingHeight(units::meter_t height);

  /**
   * Set the minimum (lower hard) height for simulation and soft-limit purposes.
   *
   * @param height Minimum height.
   * @return *this for chaining.
   */
  ElevatorConfig& WithMinimumHeight(units::meter_t height);

  /**
   * Set the maximum (upper hard) height for simulation and soft-limit purposes.
   *
   * @param height Maximum height.
   * @return *this for chaining.
   */
  ElevatorConfig& WithMaximumHeight(units::meter_t height);

  // ---- Getters ---------------------------------------------------------------

  /** Get the motor controller pointer. */
  motorcontrollers::SmartMotorController* GetMotorController() const;

  /** Get the subsystem pointer. */
  frc2::SubsystemBase* GetSubsystem() const;

  /** Get the telemetry name. */
  std::string GetTelemetryName() const;

  /** Get the optional starting height. */
  std::optional<units::meter_t> GetStartingHeight() const;

  /** Get the optional minimum height. */
  std::optional<units::meter_t> GetMinHeight() const;

  /** Get the optional maximum height. */
  std::optional<units::meter_t> GetMaxHeight() const;

 private:
  motorcontrollers::SmartMotorController* m_smc{nullptr};
  frc2::SubsystemBase* m_subsystem{nullptr};
  std::string m_telemetryName;
  std::optional<units::meter_t> m_startingHeight;
  std::optional<units::meter_t> m_minHeight;
  std::optional<units::meter_t> m_maxHeight;
};

}  // namespace yams::mechanisms::config
