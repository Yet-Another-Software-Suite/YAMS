// Copyright (c) 2026 YAMS Contributors
// SPDX-License-Identifier: LGPL-3.0-or-later

#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/button/Trigger.h>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/length.h>
#include <units/time.h>
#include <units/velocity.h>
#include <units/voltage.h>

#include <functional>
#include <string>

#include "SmartVelocityMechanism.h"
#include "yams/mechanisms/config/FlyWheelConfig.hpp"

namespace yams::mechanisms::velocity {

/**
 * Smart mechanism implementation for a flywheel (velocity-controlled roller).
 *
 * Drives a spinning element using a SmartMotorController configured for
 * mechanism-space (degrees/second) closed-loop velocity control.  If a roller
 * diameter is provided, surface-speed commands and reads are also supported.
 */
class FlyWheel : public SmartVelocityMechanism {
 public:
  /**
   * Construct a FlyWheel from a FlyWheelConfig.
   *
   * @param config Fully-populated flywheel configuration.
   */
  explicit FlyWheel(const config::FlyWheelConfig& config);

  // ---- SmartMechanism overrides ---------------------------------------------

  /** Advance the flywheel's simulation by one loop iteration. */
  void SimIterate() override;

  /** Publish flywheel telemetry to NetworkTables / SmartDashboard. */
  void UpdateTelemetry() override;

  /** Update the Mechanism2d ligament to reflect the current velocity. */
  void VisualizationUpdate() override;

  /** Get the human-readable name of this flywheel. */
  std::string GetName() const override;

  // ---- SmartVelocityMechanism overrides -------------------------------------

  /**
   * Trigger that becomes true when the flywheel is at or above its maximum
   * configured velocity.
   *
   * @return Trigger for the upper velocity limit.
   */
  frc2::Trigger Max() override;

  /**
   * Trigger that becomes true when the flywheel is at or below its minimum
   * configured velocity (e.g. stalled or reversed).
   *
   * @return Trigger for the lower velocity limit.
   */
  frc2::Trigger Min() override;

  /**
   * Build a SysId characterisation routine for this flywheel.
   *
   * @param maxVoltage  Maximum voltage for the quasistatic test.
   * @param step        Voltage ramp rate for the dynamic test (V/s).
   * @param duration    Duration of each test step.
   * @return CommandPtr that runs the full SysId sequence.
   */
  frc2::CommandPtr SysId(units::volt_t maxVoltage, frc2::sysid::ramp_rate_t step,
                         units::second_t duration) override;

  // ---- FlyWheel-specific interface ------------------------------------------

  /**
   * Command the flywheel to spin at a fixed angular velocity and hold it.
   *
   * @param velocity Target angular velocity in degrees per second.
   * @return CommandPtr that requires the configured subsystem.
   */
  frc2::CommandPtr Spin(units::degrees_per_second_t velocity);

  /**
   * Command the flywheel to track a supplier-provided angular velocity.
   *
   * @param velocity Supplier returning the desired angular velocity each loop.
   * @return CommandPtr that requires the configured subsystem.
   */
  frc2::CommandPtr Spin(std::function<units::degrees_per_second_t()> velocity);

  /**
   * Command the flywheel to reach a target surface (tangential) speed.
   *
   * Requires that a roller diameter was provided in the config.  If no
   * diameter is configured, the command falls back to a no-op warning.
   *
   * @param surfaceSpeed Desired surface speed in metres per second.
   * @return CommandPtr that requires the configured subsystem.
   */
  frc2::CommandPtr SpinSurface(units::meters_per_second_t surfaceSpeed);

  /**
   * Get the current angular velocity of the flywheel.
   *
   * @return Current mechanism velocity in degrees per second.
   */
  units::degrees_per_second_t GetVelocity() const;

  /**
   * Get the current surface speed of the roller.
   *
   * Returns zero if no roller diameter is configured.
   *
   * @return Surface speed in metres per second.
   */
  units::meters_per_second_t GetSurfaceSpeed() const;

  /**
   * Check whether the flywheel is within tolerance of a target velocity.
   *
   * @param target    Desired angular velocity.
   * @param tolerance Allowable error (default 5 deg/s).
   * @return true if |current − target| ≤ tolerance.
   */
  bool AtVelocity(units::degrees_per_second_t target,
                  units::degrees_per_second_t tolerance = units::degrees_per_second_t{5.0}) const;

  /**
   * Trigger that becomes true when the flywheel is within tolerance of a
   * target velocity.
   *
   * @param target    Desired angular velocity.
   * @param tolerance Allowable error (default 5 deg/s).
   * @return Trigger for the at-velocity condition.
   */
  frc2::Trigger IsAtVelocity(units::degrees_per_second_t target,
                             units::degrees_per_second_t tolerance = units::degrees_per_second_t{
                                 5.0});

 private:
  config::FlyWheelConfig m_flyWheelConfig;
  std::string m_name{"FlyWheel"};
};

}  // namespace yams::mechanisms::velocity
