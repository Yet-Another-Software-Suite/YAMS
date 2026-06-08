// Copyright (c) 2026 YAMS Contributors
// SPDX-License-Identifier: LGPL-3.0-or-later

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc/util/Color.h>
#include <frc/util/Color8Bit.h>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/moment_of_inertia.h>

#include <optional>
#include <string>

#include "yams/motorcontrollers/SmartMotorController.hpp"

namespace yams::mechanisms::config {

/**
 * Configuration class for a Pivot mechanism.
 *
 * Pivots are similar to Arms but do not necessarily have a meaningful arm
 * length — they typically rotate a mechanism around a fixed axis.  Uses a
 * fluent builder pattern; all With* methods return *this for chaining.
 */
class PivotConfig {
 public:
  PivotConfig() = default;

  // ---- Builder methods -------------------------------------------------------

  /**
   * Set the SmartMotorController driving the pivot.
   *
   * @param smc Pointer to the motor controller (must outlive this config).
   * @return *this for chaining.
   */
  PivotConfig& WithMotorController(motorcontrollers::SmartMotorController* smc);

  /**
   * Set the subsystem that owns the pivot (used for command requirements).
   *
   * @param subsystem Pointer to the subsystem.
   * @return *this for chaining.
   */
  PivotConfig& WithSubsystem(frc2::SubsystemBase* subsystem);

  /**
   * Set the NetworkTables / SmartDashboard name for telemetry.
   *
   * @param name Telemetry name string.
   * @return *this for chaining.
   */
  PivotConfig& WithTelemetryName(const std::string& name);

  /**
   * Set the encoder starting angle (seeds the motor encoder on init).
   *
   * @param angle Starting angle of the pivot.
   * @return *this for chaining.
   */
  PivotConfig& WithStartingAngle(units::degree_t angle);

  /**
   * Set the minimum (lower hard) angle for simulation and soft-limit purposes.
   *
   * @param angle Minimum angle.
   * @return *this for chaining.
   */
  PivotConfig& WithMinAngle(units::degree_t angle);

  /**
   * Set the maximum (upper hard) angle for simulation and soft-limit purposes.
   *
   * @param angle Maximum angle.
   * @return *this for chaining.
   */
  PivotConfig& WithMaxAngle(units::degree_t angle);

  /**
   * Set the moment of inertia of the pivot (required for simulation).
   *
   * @param moi Pivot MOI in kg·m².
   * @return *this for chaining.
   */
  PivotConfig& WithMOI(units::kilogram_square_meter_t moi);

  /**
   * Set the Mechanism2d simulation colour for the pivot ligament (default: green).
   *
   * @param color Desired colour.
   * @return *this for chaining.
   */
  PivotConfig& WithSimColor(const frc::Color8Bit& color);

  // ---- Getters ---------------------------------------------------------------

  /** Get the motor controller pointer. */
  motorcontrollers::SmartMotorController* GetMotorController() const;

  /** Get the subsystem pointer. */
  frc2::SubsystemBase* GetSubsystem() const;

  /** Get the telemetry name. */
  std::string GetTelemetryName() const;

  /** Get the optional starting angle. */
  std::optional<units::degree_t> GetStartingAngle() const;

  /** Get the optional minimum (lower hard-limit) angle. */
  std::optional<units::degree_t> GetMinAngle() const;

  /** Get the optional maximum (upper hard-limit) angle. */
  std::optional<units::degree_t> GetMaxAngle() const;

  /** Get the optional moment of inertia. */
  std::optional<units::kilogram_square_meter_t> GetMOI() const;

  /** Get the Mechanism2d simulation colour. */
  frc::Color8Bit GetSimColor() const;

 private:
  motorcontrollers::SmartMotorController* m_smc{nullptr};
  frc2::SubsystemBase* m_subsystem{nullptr};
  std::string m_telemetryName;
  std::optional<units::degree_t> m_startingAngle;
  std::optional<units::degree_t> m_minAngle;
  std::optional<units::degree_t> m_maxAngle;
  std::optional<units::kilogram_square_meter_t> m_moi;
  frc::Color8Bit m_simColor{frc::Color::kGreen};
};

}  // namespace yams::mechanisms::config
