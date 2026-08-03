// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

#pragma once

#include <wpi/util/Color.hpp>
#include <wpi/util/Color8Bit.hpp>
#include <wpi/units/angle.hpp>
#include <wpi/units/angular_velocity.hpp>
#include <wpi/units/moment_of_inertia.hpp>

#include <optional>
#include <string>

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
   * Set the NetworkTables / SmartDashboard name for telemetry.
   *
   * @param name Telemetry name string.
   * @return *this for chaining.
   */
  PivotConfig& WithTelemetryName(const std::string& name);

  /**
   * Set the minimum (lower hard) angle for simulation and soft-limit purposes.
   *
   * @param angle Minimum angle.
   * @return *this for chaining.
   */
  PivotConfig& WithMinAngle(wpi::units::degree_t angle);

  /**
   * Set the maximum (upper hard) angle for simulation and soft-limit purposes.
   *
   * @param angle Maximum angle.
   * @return *this for chaining.
   */
  PivotConfig& WithMaxAngle(wpi::units::degree_t angle);

  /**
   * Set the Mechanism2d simulation colour for the pivot ligament (default: green).
   *
   * @param color Desired colour.
   * @return *this for chaining.
   */
  PivotConfig& WithSimColor(const wpi::util::Color8Bit& color);

  // ---- Getters ---------------------------------------------------------------

  /** Get the telemetry name. */
  std::string GetTelemetryName() const;

  /** Get the optional minimum (lower hard-limit) angle. */
  std::optional<wpi::units::degree_t> GetMinAngle() const;

  /** Get the optional maximum (upper hard-limit) angle. */
  std::optional<wpi::units::degree_t> GetMaxAngle() const;

  /** Get the Mechanism2d simulation colour. */
  wpi::util::Color8Bit GetSimColor() const;

 private:
  std::string m_telemetryName;
  std::optional<wpi::units::degree_t> m_minAngle;
  std::optional<wpi::units::degree_t> m_maxAngle;
  wpi::util::Color8Bit m_simColor{wpi::util::Color::GREEN};
};

}  // namespace yams::mechanisms::config
