// Copyright (c) 2026 YAMS Contributors
// SPDX-License-Identifier: LGPL-3.0-or-later

#pragma once

#include <units/angle.h>

#include <functional>
#include <optional>

namespace yams::mechanisms::config {

/**
 * Configuration for an absolute encoder used to seed or synchronize a relative encoder.
 *
 * Supports CTRE CANcoder, CANdi, REV SPARK absolute encoder, Through-bore encoders,
 * and a CUSTOM type backed by an arbitrary angle supplier.  Uses a fluent builder pattern;
 * all With* methods return *this for chaining.
 */
class SensorConfig {
 public:
  /** Supported absolute encoder hardware types. */
  enum class SensorType { CANCODER, CANDI, THROUGHBORE, SPARK_ABSOLUTE, CUSTOM };

  /**
   * Set the absolute encoder hardware type.
   *
   * @param type Sensor hardware type.
   * @return *this for chaining.
   */
  SensorConfig& WithSensorType(SensorType type);

  /**
   * Set the zero-offset applied to the raw encoder reading.
   *
   * @param offset Offset angle to subtract from the raw encoder value.
   * @return *this for chaining.
   */
  SensorConfig& WithOffset(units::degree_t offset);

  /**
   * Set whether the encoder reading should be negated.
   *
   * @param inverted true to invert the encoder direction.
   * @return *this for chaining.
   */
  SensorConfig& WithInverted(bool inverted);

  /**
   * Set a custom angle supplier used when SensorType is CUSTOM.
   *
   * @param supplier Callable returning the current absolute angle.
   * @return *this for chaining.
   */
  SensorConfig& WithAngleSupplier(std::function<units::degree_t()> supplier);

  /**
   * Set the tolerance used when synchronizing the relative encoder to this sensor.
   *
   * Synchronization is skipped if the relative and absolute encoder readings differ by
   * more than this tolerance (guards against synchronizing onto a noisy reading).
   *
   * @param tolerance Maximum allowable discrepancy before synchronization is accepted.
   * @return *this for chaining.
   */
  SensorConfig& WithSynchronizationTolerance(units::degree_t tolerance);

  /**
   * Get the configured sensor hardware type.
   *
   * @return SensorType.
   */
  SensorType GetSensorType() const;

  /**
   * Get the optional zero-offset.
   *
   * @return Offset angle if set, otherwise empty.
   */
  std::optional<units::degree_t> GetOffset() const;

  /**
   * Get whether the encoder is inverted.
   *
   * @return true if the encoder direction is inverted.
   */
  bool GetInverted() const;

  /**
   * Get the custom angle supplier.
   *
   * @return Supplier function (may be nullptr if not set).
   */
  std::function<units::degree_t()> GetAngleSupplier() const;

  /**
   * Get the optional synchronization tolerance.
   *
   * @return Tolerance if set, otherwise empty.
   */
  std::optional<units::degree_t> GetSynchronizationTolerance() const;

 private:
  SensorType m_sensorType{SensorType::CUSTOM};
  std::optional<units::degree_t> m_offset;
  bool m_inverted{false};
  std::function<units::degree_t()> m_angleSupplier{nullptr};
  std::optional<units::degree_t> m_syncTolerance;
};

}  // namespace yams::mechanisms::config
