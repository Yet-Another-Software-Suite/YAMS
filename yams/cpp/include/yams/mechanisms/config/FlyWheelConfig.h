// Copyright (c) 2026 YAMS Contributors
// SPDX-License-Identifier: LGPL-3.0-or-later

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <units/length.h>

#include <optional>
#include <string>

#include "yams/motorcontrollers/SmartMotorController.h"

namespace yams::mechanisms::config {

/**
 * Configuration class for a FlyWheel (velocity) mechanism.
 *
 * Uses a fluent builder pattern; all With* methods return *this for chaining.
 */
class FlyWheelConfig {
 public:
  FlyWheelConfig() = default;

  // ---- Builder methods -------------------------------------------------------

  /**
   * Set the SmartMotorController driving the flywheel.
   *
   * @param smc Pointer to the motor controller (must outlive this config).
   * @return *this for chaining.
   */
  FlyWheelConfig& WithMotorController(motorcontrollers::SmartMotorController* smc);

  /**
   * Set the subsystem that owns the flywheel (used for command requirements).
   *
   * @param subsystem Pointer to the subsystem.
   * @return *this for chaining.
   */
  FlyWheelConfig& WithSubsystem(frc2::SubsystemBase* subsystem);

  /**
   * Set the NetworkTables / SmartDashboard name for telemetry.
   *
   * @param name Telemetry name string.
   * @return *this for chaining.
   */
  FlyWheelConfig& WithTelemetryName(const std::string& name);

  /**
   * Set the roller/flywheel outer diameter.
   *
   * Used to convert between angular and linear velocity for surface-speed
   * setpoints and simulation.
   *
   * @param diameter Outer diameter of the roller.
   * @return *this for chaining.
   */
  FlyWheelConfig& WithRollerDiameter(units::meter_t diameter);

  // ---- Getters ---------------------------------------------------------------

  /** Get the motor controller pointer. */
  motorcontrollers::SmartMotorController* GetMotorController() const;

  /** Get the subsystem pointer. */
  frc2::SubsystemBase* GetSubsystem() const;

  /** Get the telemetry name. */
  std::string GetTelemetryName() const;

  /** Get the optional roller diameter. */
  std::optional<units::meter_t> GetRollerDiameter() const;

 private:
  motorcontrollers::SmartMotorController* m_smc{nullptr};
  frc2::SubsystemBase* m_subsystem{nullptr};
  std::string m_telemetryName;
  std::optional<units::meter_t> m_rollerDiameter;
};

}  // namespace yams::mechanisms::config
