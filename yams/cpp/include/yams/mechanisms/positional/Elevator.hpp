// Copyright (c) 2026 YAMS Contributors
// SPDX-License-Identifier: LGPL-3.0-or-later

#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/button/Trigger.h>
#include <units/length.h>
#include <units/time.h>
#include <units/voltage.h>

#include <functional>
#include <string>

#include "SmartPositionalMechanism.h"
#include "yams/mechanisms/config/ElevatorConfig.hpp"

namespace yams::mechanisms::positional {

/**
 * Smart mechanism implementation for a linear elevator.
 *
 * Drives a carriage along a linear axis using a SmartMotorController
 * configured for measurement-space (meter) closed-loop control.
 */
class Elevator : public SmartPositionalMechanism {
 public:
  /**
   * Construct an Elevator from an ElevatorConfig.
   *
   * @param config Fully-populated elevator configuration.
   */
  explicit Elevator(const config::ElevatorConfig& config);

  // ---- SmartMechanism overrides ---------------------------------------------

  /** Advance the elevator's simulation by one loop iteration. */
  void SimIterate() override;

  /** Publish elevator telemetry to NetworkTables / SmartDashboard. */
  void UpdateTelemetry() override;

  /** Update the Mechanism2d ligament to reflect the current height. */
  void VisualizationUpdate() override;

  /** Get the human-readable name of this elevator. */
  std::string GetName() const override;

  // ---- SmartPositionalMechanism overrides -----------------------------------

  /**
   * Trigger that becomes true when the elevator is at or above its maximum
   * configured height.
   *
   * @return Trigger for the upper hard limit.
   */
  frc2::Trigger Max() override;

  /**
   * Trigger that becomes true when the elevator is at or below its minimum
   * configured height.
   *
   * @return Trigger for the lower hard limit.
   */
  frc2::Trigger Min() override;

  /**
   * Build a SysId characterisation routine for this elevator.
   *
   * @param maxVoltage  Maximum voltage for the quasistatic test.
   * @param step        Voltage ramp rate for the dynamic test (V/s).
   * @param duration    Duration of each test step.
   * @return CommandPtr that runs the full SysId sequence.
   */
  frc2::CommandPtr SysId(units::volt_t maxVoltage, frc2::sysid::ramp_rate_t step,
                         units::second_t duration) override;

  // ---- Elevator-specific interface ------------------------------------------

  /**
   * Command the elevator to move to a fixed height and hold it.
   *
   * @param height Target carriage height.
   * @return CommandPtr that requires the configured subsystem.
   */
  frc2::CommandPtr GoToHeight(units::meter_t height);

  /**
   * Command the elevator to track a supplier-provided height setpoint.
   *
   * @param height Supplier returning the desired height each loop.
   * @return CommandPtr that requires the configured subsystem.
   */
  frc2::CommandPtr GoToHeight(std::function<units::meter_t()> height);

  /**
   * Get the current carriage height from the motor encoder.
   *
   * @return Current height in meters.
   */
  units::meter_t GetHeight() const;

  /**
   * Check whether the elevator is within tolerance of a target height.
   *
   * @param target    Desired height.
   * @param tolerance Allowable error (default 0.01 m).
   * @return true if |current − target| ≤ tolerance.
   */
  bool IsAtHeight(units::meter_t target, units::meter_t tolerance = units::meter_t{0.01}) const;

  /**
   * Trigger that becomes true when the elevator is within tolerance of a
   * target height.
   *
   * @param target    Desired height.
   * @param tolerance Allowable error (default 0.01 m).
   * @return Trigger for the at-height condition.
   */
  frc2::Trigger AtHeight(units::meter_t target, units::meter_t tolerance = units::meter_t{0.01});

 private:
  config::ElevatorConfig m_elevatorConfig;
  std::string m_name{"Elevator"};
};

}  // namespace yams::mechanisms::positional
