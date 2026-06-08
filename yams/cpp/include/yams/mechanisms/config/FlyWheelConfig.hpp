// Copyright (c) 2026 YAMS Contributors
// SPDX-License-Identifier: LGPL-3.0-or-later

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc/util/Color.h>
#include <frc/util/Color8Bit.h>
#include <units/angular_velocity.h>
#include <units/length.h>

#include <optional>
#include <string>

#include "yams/motorcontrollers/SmartMotorController.hpp"

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
   * Set the SmartMotorController driving the flywheel.
   *
   * @param smc Pointer to the motor controller (must outlive this config).
   * @return *this for chaining.
   */
  FlyWheelConfig& WithMotorController(motorcontrollers::SmartMotorController* smc);

  /**
   * Set the subsystem that owns the flywheel (used for command requirements).
   *
   * @param subsystem Pointer to the subsystem.
   * @return *this for chaining.
   */
  FlyWheelConfig& WithSubsystem(frc2::SubsystemBase* subsystem);

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

  /** Set the lower soft-limit velocity used for soft-stop and speedometer visualisation. */
  FlyWheelConfig& WithLowerSoftLimit(units::degrees_per_second_t limit);

  /** Set the upper soft-limit velocity used for soft-stop and speedometer visualisation. */
  FlyWheelConfig& WithUpperSoftLimit(units::degrees_per_second_t limit);

  /** Set both soft-limit velocities at once. */
  FlyWheelConfig& WithSoftLimits(units::degrees_per_second_t lower,
                                  units::degrees_per_second_t upper);

  /**
   * Enable speedometer-style simulation visualisation with the given maximum velocity.
   * The ligament angle is driven by current_velocity / maxVelocity rather than raw position.
   */
  FlyWheelConfig& WithSpeedometerSimulation(units::degrees_per_second_t maxVelocity);

  /** Enable speedometer simulation (max velocity must already be set via soft limits or separately). */
  FlyWheelConfig& WithSpeedometerSimulation();

  // ---- Getters ---------------------------------------------------------------

  /** Get the motor controller pointer. */
  motorcontrollers::SmartMotorController* GetMotorController() const;

  /** Get the subsystem pointer. */
  frc2::SubsystemBase* GetSubsystem() const;

  /** Get the telemetry name. */
  std::string GetTelemetryName() const;

  /** Get the optional roller diameter. */
  std::optional<units::meter_t> GetRollerDiameter() const;

  /** Get the Mechanism2d sim colour. */
  frc::Color8Bit GetSimColor() const;

  /** Returns true if speedometer-style visualisation is active. */
  bool IsUsingSpeedometerSimulation() const;

  /** Get the optional speedometer maximum velocity. */
  std::optional<units::degrees_per_second_t> GetSpeedometerMaxVelocity() const;

  /** Get the optional upper soft-limit velocity. */
  std::optional<units::degrees_per_second_t> GetUpperSoftLimit() const;

  /** Get the optional lower soft-limit velocity. */
  std::optional<units::degrees_per_second_t> GetLowerSoftLimit() const;

 private:
  motorcontrollers::SmartMotorController* m_smc{nullptr};
  frc2::SubsystemBase* m_subsystem{nullptr};
  std::string m_telemetryName;
  std::optional<units::meter_t> m_rollerDiameter;
  frc::Color8Bit m_simColor{frc::Color::kOrange};
  bool m_useSpeedometer{false};
  std::optional<units::degrees_per_second_t> m_speedometerMaxVelocity;
  std::optional<units::degrees_per_second_t> m_upperSoftLimit;
  std::optional<units::degrees_per_second_t> m_lowerSoftLimit;
};

}  // namespace yams::mechanisms::config
