// Copyright (c) 2026 YAMS Contributors
// SPDX-License-Identifier: LGPL-3.0-or-later

#pragma once

#include <units/angle.h>
#include <units/length.h>

#include <optional>
#include <string>

namespace yams::mechanisms::config {

class MechanismPositionConfig {
 public:
  explicit MechanismPositionConfig(const std::string& name);
  MechanismPositionConfig(const std::string& name, units::degree_t angle);
  MechanismPositionConfig(const std::string& name, units::meter_t distance);

  const std::string& GetName() const;
  std::optional<units::degree_t> GetAngle() const;
  std::optional<units::meter_t> GetDistance() const;

  bool IsAngular() const;
  bool IsLinear() const;

 private:
  std::string m_name;
  std::optional<units::degree_t> m_angle;
  std::optional<units::meter_t> m_distance;
};

}  // namespace yams::mechanisms::config
