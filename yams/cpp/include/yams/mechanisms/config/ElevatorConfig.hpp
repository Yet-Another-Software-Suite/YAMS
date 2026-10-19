// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

#pragma once

#include <concepts>
#include <optional>
#include <string>
#include <vector>
#include <wpi/units/angle.hpp>
#include <wpi/units/length.hpp>
#include <wpi/units/mass.hpp>
#include <wpi/units/velocity.hpp>
#include <wpi/util/Color.hpp>
#include <wpi/util/Color8Bit.hpp>

#include "yams/mechanisms/config/MechanismPositionConfig.hpp"

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
  ElevatorConfig& WithMinimumHeight(wpi::units::meter_t height);

  /**
   * Set the maximum (upper hard) height for simulation and soft-limit purposes.
   *
   * @param height Maximum height.
   * @return *this for chaining.
   */
  ElevatorConfig& WithMaximumHeight(wpi::units::meter_t height);

  /**
   * Set the carriage mass (required for simulation).
   *
   * @param mass Carriage mass in kg.
   * @return *this for chaining.
   */
  ElevatorConfig& WithCarriageMass(wpi::units::kilogram_t mass);

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
  ElevatorConfig& WithSimColor(const wpi::util::Color8Bit& color);

  /**
   * Set the angle of the elevator ligament in the Mechanism2d window (default: 90°/vertical).
   *
   * @param angle Ligament angle.
   * @return *this for chaining.
   */
  ElevatorConfig& WithAngle(wpi::units::degree_t angle);

  /**
   * Set the elevator's moving stages, for the pose of each stage from Elevator::GetCascadePose3d()
   * or Elevator::GetContinuousPose3d(bool).
   *
   * The stage heights must add up to the elevator's maximum height: the Elevator throws if they do
   * not. Without a minimum and maximum height, the elevator's height is from 0 to the stage heights'
   * total.
   *
   * @param stageHeights How far each moving stage travels relative to the stage below it, from the
   *        stage nearest the fixed stage to the carriage, which is the last. One for each stage that
   *        moves, such as WithStages(0.6_m, 0.6_m, 0.5_m).
   * @return *this for chaining.
   * @throws std::invalid_argument if no height is given, or a height is not positive.
   */
  template <typename... Heights>
    requires(std::convertible_to<Heights, wpi::units::meter_t> && ...)
  ElevatorConfig& WithStages(Heights... stageHeights) {
    return WithStages(std::vector<wpi::units::meter_t>{wpi::units::meter_t{stageHeights}...});
  }

  /**
   * Set the elevator's moving stages from a list of stage heights. See WithStages(Heights...).
   *
   * @param stageHeights How far each moving stage travels relative to the stage below it, from the
   *        stage nearest the fixed stage to the carriage.
   * @return *this for chaining.
   * @throws std::invalid_argument if no height is given, or a height is not positive.
   */
  ElevatorConfig& WithStages(std::vector<wpi::units::meter_t> stageHeights);

  /**
   * Set where the mechanism is on the robot, for GetPose3d(): where it is mounted, how it is turned
   * there, the plane it moves in, and the mechanism it is mounted on, if any.
   *
   * @param mechanismPositionConfig The mechanism's position.
   * @return *this for chaining.
   */
  ElevatorConfig& WithMechanismPositionConfig(const MechanismPositionConfig& mechanismPositionConfig) {
    m_mechanismPositionConfig = mechanismPositionConfig;
    return *this;
  }

  // ---- Getters ---------------------------------------------------------------

  /** Get the telemetry name. */
  std::string GetTelemetryName() const;

  /** Get the minimum height, or 0 without one when the stages are set. */
  std::optional<wpi::units::meter_t> GetMinHeight() const;

  /** Get the maximum height, or the stage heights' total without one. */
  std::optional<wpi::units::meter_t> GetMaxHeight() const;

  /** Get the optional carriage mass. */
  std::optional<wpi::units::kilogram_t> GetCarriageMass() const;

  /** Returns true when the elevator is configured as horizontal (gravity disabled). */
  bool IsHorizontal() const;

  /** Get the Mechanism2d simulation colour. */
  wpi::util::Color8Bit GetSimColor() const;

  /** Get the Mechanism2d ligament angle (default: 90°). */
  wpi::units::degree_t GetAngle() const;

  /** Get the number of stages that move, counting the carriage as the last, if set. */
  std::optional<int> GetStages() const;

  /**
   * Get how far each moving stage travels relative to the stage below it, from the stage nearest the
   * fixed stage to the carriage. Empty if WithStages() was not called.
   */
  const std::vector<wpi::units::meter_t>& GetStageHeights() const;

  /** Get how far the stages travel together, the sum of the stage heights, if set. */
  std::optional<wpi::units::meter_t> GetStageHeightTotal() const;

  /** Get where the mechanism is on the robot. */
  const MechanismPositionConfig& GetMechanismPositionConfig() const { return m_mechanismPositionConfig; }

 private:
  std::string m_telemetryName;
  std::optional<wpi::units::meter_t> m_minHeight;
  std::optional<wpi::units::meter_t> m_maxHeight;
  std::optional<wpi::units::kilogram_t> m_carriageMass;
  std::optional<wpi::units::meter_t> m_drumRadius;
  bool m_isHorizontal{false};
  wpi::util::Color8Bit m_simColor{wpi::util::Color::ORANGE};
  wpi::units::degree_t m_angle{90.0};
  std::optional<int> m_stages;
  std::vector<wpi::units::meter_t> m_stageHeights;
  std::optional<wpi::units::meter_t> m_stageHeightTotal;
  MechanismPositionConfig m_mechanismPositionConfig;
};

}  // namespace yams::mechanisms::config
