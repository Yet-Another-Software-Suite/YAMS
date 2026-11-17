// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

#pragma once

#include <wpi/units/angle.hpp>
#include <wpi/units/angular_acceleration.hpp>
#include <wpi/units/angular_velocity.hpp>
#include <wpi/units/current.hpp>
#include <wpi/units/voltage.hpp>

#include "yams/motorcontrollers/SimSupplier.hpp"

namespace yams::motorcontrollers {
class SmartMotorController;
}  // namespace yams::motorcontrollers

namespace yams::motorcontrollers::simulation {

/**
 * Simulation of a motor whose leader's simulation has its motor too, such as an A301 following
 * another, or a loosely coupled follower of a mechanism's motor controller.
 *
 * It reads the leader's simulation, with its rotor values from its own gearing and its share of the
 * current, and does not step a simulation of its own. Its current is in the leader's, so BatterySim
 * does not count it again.
 */
class FollowerSimSupplier : public SimSupplier {
 public:
  /**
   * Read the simulation of a leader whose motor model has this follower's motor.
   *
   * @param leader   Motor controller whose simulation has the follower's motor. Must outlive this.
   * @param follower Motor controller reading it. Must outlive this.
   * @param inverted Whether the follower is inverted from its leader.
   */
  FollowerSimSupplier(SmartMotorController& leader, SmartMotorController& follower, bool inverted);

  void UpdateSim() override {}
  bool GetUpdatedSim() override { return true; }
  void FeedUpdateSim() override {}
  void StarveUpdateSim() override {}
  bool IsInputFed() override { return true; }
  void FeedInput() override {}
  void StarveInput() override {}
  void SetMechanismStatorDutyCycle(double) override {}
  wpi::units::volt_t GetMechanismSupplyVoltage() override;
  wpi::units::volt_t GetMechanismStatorVoltage() override;
  void SetMechanismStatorVoltage(wpi::units::volt_t) override {}
  wpi::units::turn_t GetMechanismPosition() override;
  void SetMechanismPosition(wpi::units::turn_t) override {}
  wpi::units::turn_t GetRotorPosition() override;
  wpi::units::turns_per_second_t GetMechanismVelocity() override;
  void SetMechanismVelocity(wpi::units::turns_per_second_t) override {}
  wpi::units::turns_per_second_t GetRotorVelocity() override;
  wpi::units::turns_per_second_squared_t GetMechanismAcceleration() override;
  wpi::units::turns_per_second_squared_t GetRotorAcceleration() override;
  void SetRotorPosition(wpi::units::turn_t) override {}
  void SetRotorVelocity(wpi::units::turns_per_second_t) override {}
  wpi::units::ampere_t GetStatorCurrent() override;
  wpi::units::ampere_t GetSupplyCurrent() override;

 private:
  /** The leader's simulation, which a mechanism can replace with its own. */
  SimSupplier& Leader();
  /** Gearing from the follower's rotor to the mechanism. */
  double RotorRatio();
  /** The follower's share of the leader's current, by its share of the stall current. */
  wpi::units::ampere_t Share(wpi::units::ampere_t current);

  SmartMotorController& m_leader;
  SmartMotorController& m_follower;
  /** -1 when the follower is inverted from its leader, otherwise 1. */
  double m_sign;
};

}  // namespace yams::motorcontrollers::simulation
