// Copyright (c) 2026 YAMS Contributors
// SPDX-License-Identifier: LGPL-3.0-or-later

#pragma once

#include <units/angle.h>

#include <functional>
#include <optional>

namespace yams::mechanisms::config {

class SensorConfig {
 public:
  enum class SensorType { CANCODER, CANDI, THROUGHBORE, SPARK_ABSOLUTE, CUSTOM };

  SensorConfig& WithSensorType(SensorType type);
  SensorConfig& WithOffset(units::degree_t offset);
  SensorConfig& WithInverted(bool inverted);
  SensorConfig& WithAngleSupplier(std::function<units::degree_t()> supplier);
  SensorConfig& WithSynchronizationTolerance(units::degree_t tolerance);

  SensorType GetSensorType() const;
  std::optional<units::degree_t> GetOffset() const;
  bool GetInverted() const;
  std::function<units::degree_t()> GetAngleSupplier() const;
  std::optional<units::degree_t> GetSynchronizationTolerance() const;

 private:
  SensorType m_sensorType{SensorType::CUSTOM};
  std::optional<units::degree_t> m_offset;
  bool m_inverted{false};
  std::function<units::degree_t()> m_angleSupplier{nullptr};
  std::optional<units::degree_t> m_syncTolerance;
};

}  // namespace yams::mechanisms::config
