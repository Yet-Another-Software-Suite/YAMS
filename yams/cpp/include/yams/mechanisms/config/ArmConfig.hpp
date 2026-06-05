// Copyright (c) 2026 YAMS Contributors
// SPDX-License-Identifier: LGPL-3.0-or-later

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/length.h>

#include <optional>
#include <string>

#include "yams/motorcontrollers/SmartMotorController.hpp"

namespace yams::mechanisms::config {

/**
 * Configuration class for an Arm mechanism.
 *
 * Uses a fluent builder pattern; all With* methods return *this for chaining.
 */
class ArmConfig {
 public:
  ArmConfig() = default;

  // ---- Builder methods -------------------------------------------------------

  /**
   * Set the SmartMotorController driving the arm.
   *
   * @param smc Pointer to the motor controller (must outlive this config).
   * @return *this for chaining.
   */
  ArmConfig& WithMotorController(motorcontrollers::SmartMotorController* smc);

  /**
   * Set the subsystem that owns the arm (used for command requirements).
   *
   * @param subsystem Pointer to the subsystem.
   * @return *this for chaining.
   */
  ArmConfig& WithSubsystem(frc2::SubsystemBase* subsystem);

  /**
   * Set the NetworkTables / SmartDashboard name for telemetry.
   *
   * @param name Telemetry name string.
   * @return *this for chaining.
   */
  ArmConfig& WithTelemetryName(const std::string& name);

  /**
   * Set the encoder starting angle (seeds the motor encoder on init).
   *
   * @param angle Starting angle of the arm.
   * @return *this for chaining.
   */
  ArmConfig& WithStartingAngle(units::degree_t angle);

  /**
   * Set the minimum (lower hard) angle for simulation and soft-limit purposes.
   *
   * @param angle Minimum angle.
   * @return *this for chaining.
   */
  ArmConfig& WithMinAngle(units::degree_t angle);

  /**
   * Set the maximum (upper hard) angle for simulation and soft-limit purposes.
   *
   * @param angle Maximum angle.
   * @return *this for chaining.
   */
  ArmConfig& WithMaxAngle(units::degree_t angle);

  /**
   * Set the physical arm length for MOI estimation and visualisation.
   *
   * @param length Length of the arm.
   * @return *this for chaining.
   */
  ArmConfig& WithArmLength(units::meter_t length);

  // ---- Getters ---------------------------------------------------------------

  /** Get the motor controller pointer. */
  motorcontrollers::SmartMotorController* GetMotorController() const;

  /** Get the subsystem pointer. */
  frc2::SubsystemBase* GetSubsystem() const;

  /** Get the telemetry name. */
  std::string GetTelemetryName() const;

  /** Get the optional starting angle. */
  std::optional<units::degree_t> GetStartingAngle() const;

  /** Get the optional minimum angle. */
  std::optional<units::degree_t> GetMinAngle() const;

  /** Get the optional maximum angle. */
  std::optional<units::degree_t> GetMaxAngle() const;

  /** Get the optional arm length. */
  std::optional<units::meter_t> GetArmLength() const;

 private:
  motorcontrollers::SmartMotorController* m_smc{nullptr};
  frc2::SubsystemBase* m_subsystem{nullptr};
  std::string m_telemetryName;
  std::optional<units::degree_t> m_startingAngle;
  std::optional<units::degree_t> m_minAngle;
  std::optional<units::degree_t> m_maxAngle;
  std::optional<units::meter_t> m_armLength;
};

}  // namespace yams::mechanisms::config
