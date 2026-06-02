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

#include "SmartPositionalMechanism.h"
#include "yams/mechanisms/config/PivotConfig.h"

namespace yams::mechanisms::positional {

/**
 * Smart mechanism implementation for a pivot joint.
 *
 * Similar to an Arm but intended for mechanisms that rotate a fixed-length
 * assembly (e.g. a shooter hood or wrist) rather than a cantilevered arm with
 * a meaningful length.  Uses mechanism-space (degree) closed-loop control.
 */
class Pivot : public SmartPositionalMechanism {
 public:
  /**
   * Construct a Pivot from a PivotConfig.
   *
   * @param config Fully-populated pivot configuration.
   */
  explicit Pivot(const config::PivotConfig& config);

  // ---- SmartMechanism overrides ---------------------------------------------

  /** Advance the pivot's simulation by one loop iteration. */
  void SimIterate() override;

  /** Publish pivot telemetry to NetworkTables / SmartDashboard. */
  void UpdateTelemetry() override;

  /** Update the Mechanism2d ligament to reflect the current angle. */
  void VisualizationUpdate() override;

  /** Get the human-readable name of this pivot. */
  std::string GetName() const override;

  // ---- SmartPositionalMechanism overrides -----------------------------------

  /**
   * Trigger that becomes true when the pivot is at or past its maximum
   * configured angle.
   *
   * @return Trigger for the upper angular hard limit.
   */
  frc2::Trigger Max() override;

  /**
   * Trigger that becomes true when the pivot is at or past its minimum
   * configured angle.
   *
   * @return Trigger for the lower angular hard limit.
   */
  frc2::Trigger Min() override;

  /**
   * Build a SysId characterisation routine for this pivot.
   *
   * @param maxVoltage  Maximum voltage for the quasistatic test.
   * @param step        Voltage ramp rate for the dynamic test (V/s).
   * @param duration    Duration of each test step.
   * @return CommandPtr that runs the full SysId sequence.
   */
  frc2::CommandPtr SysId(units::volt_t maxVoltage, frc2::sysid::ramp_rate_t step,
                         units::second_t duration) override;

  // ---- Pivot-specific interface ---------------------------------------------

  /**
   * Command the pivot to move to a fixed angle and hold it.
   *
   * @param angle Target pivot angle.
   * @return CommandPtr that requires the configured subsystem.
   */
  frc2::CommandPtr GoToAngle(units::degree_t angle);

  /**
   * Command the pivot to track a supplier-provided angle setpoint.
   *
   * @param angle Supplier returning the desired angle each loop.
   * @return CommandPtr that requires the configured subsystem.
   */
  frc2::CommandPtr GoToAngle(std::function<units::degree_t()> angle);

  /**
   * Get the current pivot angle from the motor encoder.
   *
   * @return Current mechanism angle in degrees.
   */
  units::degree_t GetAngle() const;

  /**
   * Check whether the pivot is within tolerance of a target angle.
   *
   * @param target    Desired angle.
   * @param tolerance Allowable error (default 1 deg).
   * @return true if |current − target| ≤ tolerance.
   */
  bool IsAtAngle(units::degree_t target, units::degree_t tolerance = units::degree_t{1.0}) const;

  /**
   * Trigger that becomes true when the pivot is within tolerance of a target
   * angle.
   *
   * @param target    Desired angle.
   * @param tolerance Allowable error (default 1 deg).
   * @return Trigger for the at-angle condition.
   */
  frc2::Trigger AtAngle(units::degree_t target, units::degree_t tolerance = units::degree_t{1.0});

 private:
  config::PivotConfig m_pivotConfig;
  std::string m_name{"Pivot"};
};

}  // namespace yams::mechanisms::positional
