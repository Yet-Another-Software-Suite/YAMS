// Copyright (c) 2026 Yet Another Software Suite
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
 * unless an explicit input voltage has been set externally.
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

  units::turn_t GetMechanismPosition() override;
  units::turns_per_second_t GetMechanismVelocity() override;
  units::turns_per_second_squared_t GetMechanismAcceleration() override;
  units::turn_t GetRotorPosition() override;
  units::turns_per_second_t GetRotorVelocity() override;
  units::turns_per_second_squared_t GetRotorAcceleration() override;

  void SetMechanismPosition(units::turn_t angle) override;
  void SetMechanismVelocity(units::turns_per_second_t velocity) override;
  void SetRotorPosition(units::turn_t angle) override;
  void SetRotorVelocity(units::turns_per_second_t velocity) override;

  bool IsWatchdogExpired() override;
  void FeedWatchdog() override;
  void StarveWatchdog() override;
  units::ampere_t GetCurrentDrawAmps() override;
  units::volt_t GetMechanismSupplyVoltage() override;
  units::volt_t GetMechanismStatorVoltage() override;
  void SetMechanismStatorVoltage(units::volt_t volts) override;

 private:
  frc::sim::DCMotorSim& m_sim;
  std::function<double()> m_dutyCycleSupplier;
  gearing::MechanismGearing m_gearing;
  units::second_t m_period;
  bool m_inputFed{false};
  bool m_watchdogFed{false};
  units::volt_t m_lastInputVoltage{0};
};

}  // namespace yams::motorcontrollers::simulation
