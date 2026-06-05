// Copyright (c) 2026 YAMS Contributors
// SPDX-License-Identifier: LGPL-3.0-or-later

#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/button/Trigger.h>
#include <units/angle.h>
#include <units/time.h>
#include <units/voltage.h>

#include <functional>
#include <string>

#include "SmartPositionalMechanism.hpp"
#include "yams/mechanisms/config/ArmConfig.hpp"

namespace yams::mechanisms::positional {

/**
 * Smart mechanism implementation for a single-jointed arm.
 *
 * Drives an arm joint using a SmartMotorController configured for
 * mechanism-space (degree) closed-loop angular control.
 */
class Arm : public SmartPositionalMechanism {
 public:
  /**
   * Construct an Arm from an ArmConfig.
   *
   * @param config Fully-populated arm configuration.
   */
  explicit Arm(const config::ArmConfig& config);

  // ---- SmartMechanism overrides ---------------------------------------------

  /** Advance the arm's simulation by one loop iteration. */
  void SimIterate() override;

  /** Publish arm telemetry to NetworkTables / SmartDashboard. */
  void UpdateTelemetry() override;

  /** Update the Mechanism2d ligament to reflect the current angle. */
  void VisualizationUpdate() override;

  /** Get the human-readable name of this arm. */
  std::string GetName() const override;

  // ---- SmartPositionalMechanism overrides -----------------------------------

  /**
   * Trigger that becomes true when the arm is at or past its maximum
   * configured angle.
   *
   * @return Trigger for the upper angular hard limit.
   */
  frc2::Trigger Max() override;

  /**
   * Trigger that becomes true when the arm is at or past its minimum
   * configured angle.
   *
   * @return Trigger for the lower angular hard limit.
   */
  frc2::Trigger Min() override;

  /**
   * Build a SysId characterisation routine for this arm.
   *
   * @param maxVoltage  Maximum voltage for the quasistatic test.
   * @param step        Voltage ramp rate for the dynamic test (V/s).
   * @param duration    Duration of each test step.
   * @return CommandPtr that runs the full SysId sequence.
   */
  frc2::CommandPtr SysId(units::volt_t maxVoltage, frc2::sysid::ramp_rate_t step,
                         units::second_t duration) override;

  // ---- Arm-specific interface -----------------------------------------------

  /**
   * Command the arm to move to a fixed angle and hold it.
   *
   * @param angle Target joint angle.
   * @return CommandPtr that requires the configured subsystem.
   */
  frc2::CommandPtr GoToAngle(units::degree_t angle);

  /**
   * Command the arm to track a supplier-provided angle setpoint.
   *
   * @param angle Supplier returning the desired angle each loop.
   * @return CommandPtr that requires the configured subsystem.
   */
  frc2::CommandPtr GoToAngle(std::function<units::degree_t()> angle);

  /**
   * Get the current joint angle from the motor encoder.
   *
   * @return Current mechanism angle in degrees.
   */
  units::degree_t GetAngle() const;

  /**
   * Check whether the arm is within tolerance of a target angle.
   *
   * @param target    Desired angle.
   * @param tolerance Allowable error (default 1 deg).
   * @return true if |current − target| ≤ tolerance.
   */
  bool IsAtAngle(units::degree_t target, units::degree_t tolerance = units::degree_t{1.0}) const;

  /**
   * Trigger that becomes true when the arm is within tolerance of a target
   * angle.
   *
   * @param target    Desired angle.
   * @param tolerance Allowable error (default 1 deg).
   * @return Trigger for the at-angle condition.
   */
  frc2::Trigger AtAngle(units::degree_t target, units::degree_t tolerance = units::degree_t{1.0});

 private:
  config::ArmConfig m_armConfig;
  std::string m_name{"Arm"};
};

}  // namespace yams::mechanisms::positional
