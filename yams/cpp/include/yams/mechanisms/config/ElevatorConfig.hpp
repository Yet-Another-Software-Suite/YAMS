// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

#pragma once

#include <frc/util/Color.h>
#include <frc/util/Color8Bit.h>
#include <units/angle.h>
#include <units/length.h>
#include <units/mass.h>
#include <units/velocity.h>

#include <optional>
#include <string>

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
   * Set the NetworkTables / SmartDashboard name for telemetry.
   *
   * @param name Telemetry name string.
   * @return *this for chaining.
   */
  ElevatorConfig& WithTelemetryName(const std::string& name);

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

  /**
   * Set the carriage mass (required for simulation).
   *
   * @param mass Carriage mass in kg.
   * @return *this for chaining.
   */
  ElevatorConfig& WithCarriageMass(units::kilogram_t mass);

  /**
   * Mark the elevator as horizontal (disables gravity simulation).
   *
   * @param horizontal True to treat the elevator as horizontal.
   * @return *this for chaining.
   */
  ElevatorConfig& WithIsHorizontal(bool horizontal);

  /**
   * Set the Mechanism2d simulation colour for the elevator ligament (default: orange).
   *
   * @param color Desired colour.
   * @return *this for chaining.
   */
  ElevatorConfig& WithSimColor(const frc::Color8Bit& color);

  /**
   * Set the angle of the elevator ligament in the Mechanism2d window (default: 90°/vertical).
   *
   * @param angle Ligament angle.
   * @return *this for chaining.
   */
  ElevatorConfig& WithAngle(units::degree_t angle);

  // ---- Getters ---------------------------------------------------------------

  /** Get the telemetry name. */
  std::string GetTelemetryName() const;

  /** Get the optional minimum height. */
  std::optional<units::meter_t> GetMinHeight() const;

  /** Get the optional maximum height. */
  std::optional<units::meter_t> GetMaxHeight() const;

  /** Get the optional carriage mass. */
  std::optional<units::kilogram_t> GetCarriageMass() const;

  /** Returns true when the elevator is configured as horizontal (gravity disabled). */
  bool IsHorizontal() const;

  /** Get the Mechanism2d simulation colour. */
  frc::Color8Bit GetSimColor() const;

  /** Get the Mechanism2d ligament angle (default: 90°). */
  units::degree_t GetAngle() const;

 private:
  std::string m_telemetryName;
  std::optional<units::meter_t> m_minHeight;
  std::optional<units::meter_t> m_maxHeight;
  std::optional<units::kilogram_t> m_carriageMass;
  std::optional<units::meter_t> m_drumRadius;
  bool m_isHorizontal{false};
  frc::Color8Bit m_simColor{frc::Color::kOrange};
  units::degree_t m_angle{90.0};
};

}  // namespace yams::mechanisms::config
