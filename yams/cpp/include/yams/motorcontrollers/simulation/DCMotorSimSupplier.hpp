// Copyright (c) 2026 YAMS Contributors
// SPDX-License-Identifier: LGPL-3.0-or-later

#pragma once

#include <frc/simulation/DCMotorSim.h>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/time.h>
#include <units/voltage.h>

#include <functional>

#include "yams/gearing/MechanismGearing.hpp"
#include "yams/motorcontrollers/SimSupplier.hpp"

namespace yams::motorcontrollers::simulation {

/**
 * SimSupplier backed by a WPILib DCMotorSim.
 *
 * Suitable for flywheel- and pivot-style mechanisms that use a simple DC motor
 * physics model.  The duty cycle is read from the motor controller each iteration
 * unless an explicit input voltage has been set (e.g., for SysId characterization).
 */
class DCMotorSimSupplier : public SimSupplier {
 public:
  /**
   * Create a DCMotorSimSupplier.
   *
   * @param sim               WPILib DCMotorSim to advance each loop.
   * @param dutyCycleSupplier Callable returning the current motor duty cycle in [-1, 1].
   * @param gearing           Mechanism gearing used to derive rotor position/velocity.
   * @param period            Simulation update period.
   */
  DCMotorSimSupplier(frc::sim::DCMotorSim& sim, std::function<double()> dutyCycleSupplier,
                     const gearing::MechanismGearing& gearing, units::second_t period);

  void UpdateSim() override;
  void SetInputVoltage(units::volt_t volts) override;

  units::degree_t GetMechanismPosition() override;
  units::degrees_per_second_t GetMechanismVelocity() override;
  units::degrees_per_second_squared_t GetMechanismAcceleration() override;
  units::degree_t GetRotorPosition() override;
  units::degrees_per_second_t GetRotorVelocity() override;
  units::degrees_per_second_squared_t GetRotorAcceleration() override;

  void SetMechanismPosition(units::degree_t angle) override;
  void SetMechanismVelocity(units::degrees_per_second_t velocity) override;
  void SetRotorPosition(units::degree_t angle) override;
  void SetRotorVelocity(units::degrees_per_second_t velocity) override;

  bool IsWatchdogExpired() override;
  void FeedWatchdog() override;

 private:
  frc::sim::DCMotorSim& m_sim;
  std::function<double()> m_dutyCycleSupplier;
  gearing::MechanismGearing m_gearing;
  units::second_t m_period;
  bool m_inputFed{false};
  bool m_watchdogFed{false};
};

}  // namespace yams::motorcontrollers::simulation
