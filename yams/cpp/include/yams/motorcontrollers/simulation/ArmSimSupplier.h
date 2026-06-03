// Copyright (c) 2026 YAMS Contributors
// SPDX-License-Identifier: LGPL-3.0-or-later

#pragma once

#include <frc/simulation/SingleJointedArmSim.h>
#include <units/angle.h>
#include <units/angular_acceleration.h>
#include <units/angular_velocity.h>
#include <units/time.h>
#include <units/voltage.h>

#include <functional>

#include "yams/gearing/MechanismGearing.h"
#include "yams/motorcontrollers/SimSupplier.h"

namespace yams::motorcontrollers::simulation {

/**
 * SimSupplier backed by a WPILib SingleJointedArmSim.
 *
 * Suitable for arm mechanisms (Arm, DoubleJointedArm) that need gravity
 * compensation in simulation.  The duty cycle is read from the motor controller
 * each iteration unless an explicit input voltage has been set.
 */
class ArmSimSupplier : public SimSupplier {
 public:
  /**
   * Create an ArmSimSupplier.
   *
   * @param sim               WPILib SingleJointedArmSim to advance each loop.
   * @param dutyCycleSupplier Callable returning the current motor duty cycle in [-1, 1].
   * @param gearing           Mechanism gearing used to derive rotor position/velocity.
   * @param period            Simulation update period.
   */
  ArmSimSupplier(frc::sim::SingleJointedArmSim& sim,
                 std::function<double()> dutyCycleSupplier,
                 const gearing::MechanismGearing& gearing,
                 units::second_t period);

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
  frc::sim::SingleJointedArmSim& m_sim;
  std::function<double()> m_dutyCycleSupplier;
  gearing::MechanismGearing m_gearing;
  units::second_t m_period;
  bool m_inputFed{false};
  bool m_watchdogFed{false};
};

}  // namespace yams::motorcontrollers::simulation
