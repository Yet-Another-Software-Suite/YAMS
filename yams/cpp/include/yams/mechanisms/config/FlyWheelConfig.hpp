// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

#pragma once

#include <frc/util/Color.h>
#include <frc/util/Color8Bit.h>
#include <units/length.h>

#include <optional>
#include <string>

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

  /** Set the simulation colour for the Mechanism2d ligament (default: orange). */
  FlyWheelConfig& WithSimColor(const frc::Color8Bit& color);

  // ---- Getters ---------------------------------------------------------------

  /** Get the telemetry name. */
  std::string GetTelemetryName() const;

  /** Get the optional roller diameter. */
  std::optional<units::meter_t> GetRollerDiameter() const;

  /** Get the Mechanism2d sim colour. */
  frc::Color8Bit GetSimColor() const;

 private:
  std::string m_telemetryName;
  std::optional<units::meter_t> m_rollerDiameter;
  frc::Color8Bit m_simColor{frc::Color::kOrange};
};

}  // namespace yams::mechanisms::config
