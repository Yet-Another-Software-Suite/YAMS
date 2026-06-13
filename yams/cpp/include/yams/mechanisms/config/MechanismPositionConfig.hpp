// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

#pragma once

#include <units/angle.h>
#include <units/length.h>

#include <optional>
#include <string>

namespace yams::mechanisms::config {

/**
 * A named position setpoint for a SmartPositionalMechanism.
 *
 * Holds either an angular or a linear target value (not both) together with a
 * human-readable label used for telemetry and logging.
 */
class MechanismPositionConfig {
 public:
  /**
   * Create a named position config with no setpoint (name-only label).
   *
   * @param name Human-readable name for this position.
   */
  explicit MechanismPositionConfig(const std::string& name);

  /**
   * Create a named angular position config.
   *
   * @param name  Human-readable name for this position.
   * @param angle Target angle setpoint.
   */
  MechanismPositionConfig(const std::string& name, units::degree_t angle);

  /**
   * Create a named linear position config.
   *
   * @param name     Human-readable name for this position.
   * @param distance Target distance setpoint.
   */
  MechanismPositionConfig(const std::string& name, units::meter_t distance);

  /**
   * Get the human-readable name of this position.
   *
   * @return Name string.
   */
  const std::string& GetName() const;

  /**
   * Get the optional angular setpoint.
   *
   * @return Angle if this is an angular position config, otherwise empty.
   */
  std::optional<units::degree_t> GetAngle() const;

  /**
   * Get the optional linear setpoint.
   *
   * @return Distance if this is a linear position config, otherwise empty.
   */
  std::optional<units::meter_t> GetDistance() const;

  /**
   * Return true if this config holds an angular setpoint.
   *
   * @return true when an angle was provided at construction.
   */
  bool IsAngular() const;

  /**
   * Return true if this config holds a linear setpoint.
   *
   * @return true when a distance was provided at construction.
   */
  bool IsLinear() const;

 private:
  std::string m_name;
  std::optional<units::degree_t> m_angle;
  std::optional<units::meter_t> m_distance;
};

}  // namespace yams::mechanisms::config
