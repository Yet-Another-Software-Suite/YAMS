// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

#pragma once

#include <wpi/simulation/SingleJointedArmSim.hpp>
#include <wpi/units/angle.hpp>
#include <wpi/units/angular_acceleration.hpp>
#include <wpi/units/angular_velocity.hpp>
#include <wpi/units/time.hpp>
#include <wpi/units/voltage.hpp>

#include <functional>

#include "yams/gearing/MechanismGearing.hpp"
#include "yams/motorcontrollers/SimSupplier.hpp"

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
  ArmSimSupplier(wpi::sim::SingleJointedArmSim& sim, std::function<double()> dutyCycleSupplier,
                 const gearing::MechanismGearing& gearing, wpi::units::second_t period);

  void UpdateSim() override;
  void SetInputVoltage(wpi::units::volt_t volts) override;

  wpi::units::turn_t GetMechanismPosition() override;
  wpi::units::turns_per_second_t GetMechanismVelocity() override;
  wpi::units::turns_per_second_squared_t GetMechanismAcceleration() override;
  wpi::units::turn_t GetRotorPosition() override;
  wpi::units::turns_per_second_t GetRotorVelocity() override;
  wpi::units::turns_per_second_squared_t GetRotorAcceleration() override;

  void SetMechanismPosition(wpi::units::turn_t angle) override;
  void SetMechanismVelocity(wpi::units::turns_per_second_t velocity) override;
  void SetRotorPosition(wpi::units::turn_t angle) override;
  void SetRotorVelocity(wpi::units::turns_per_second_t velocity) override;

  bool IsWatchdogExpired() override;
  void FeedWatchdog() override;
  void StarveWatchdog() override;
  wpi::units::ampere_t GetCurrentDrawAmps() override;
  wpi::units::volt_t GetMechanismSupplyVoltage() override;
  wpi::units::volt_t GetMechanismStatorVoltage() override;
  void SetMechanismStatorVoltage(wpi::units::volt_t volts) override;

 private:
  wpi::sim::SingleJointedArmSim& m_sim;
  std::function<double()> m_dutyCycleSupplier;
  gearing::MechanismGearing m_gearing;
  wpi::units::second_t m_period;
  bool m_inputFed{false};
  bool m_watchdogFed{false};
  wpi::units::volt_t m_lastInputVoltage{0};
};

}  // namespace yams::motorcontrollers::simulation
