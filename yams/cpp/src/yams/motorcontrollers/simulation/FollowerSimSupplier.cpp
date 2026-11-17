// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

#include "yams/motorcontrollers/simulation/FollowerSimSupplier.hpp"

#include "yams/gearing/MechanismGearing.hpp"
#include "yams/motorcontrollers/SmartMotorController.hpp"
#include "yams/motorcontrollers/SmartMotorControllerConfig.hpp"

namespace yams::motorcontrollers::simulation {

namespace {

/** Gearing from a motor controller's rotor to the mechanism. */
double MechanismToRotorRatio(SmartMotorController& smc) {
  return smc.GetConfig()
      .GetMotorGearing()
      .value_or(gearing::MechanismGearing::kOne)
      .GetMechanismToRotorRatio();
}

}  // namespace

FollowerSimSupplier::FollowerSimSupplier(SmartMotorController& leader,
                                         SmartMotorController& follower, bool inverted)
    : m_leader(leader), m_follower(follower), m_sign(inverted ? -1.0 : 1.0) {}

SimSupplier& FollowerSimSupplier::Leader() { return *m_leader.GetSimSupplier(); }

double FollowerSimSupplier::RotorRatio() { return MechanismToRotorRatio(m_follower); }

wpi::units::ampere_t FollowerSimSupplier::Share(wpi::units::ampere_t current) {
  return current * (m_follower.GetDCMotor().stallCurrent / m_leader.GetMechanismDCMotor().stallCurrent);
}

wpi::units::volt_t FollowerSimSupplier::GetMechanismSupplyVoltage() {
  return Leader().GetMechanismSupplyVoltage();
}

wpi::units::volt_t FollowerSimSupplier::GetMechanismStatorVoltage() {
  return Leader().GetMechanismStatorVoltage() * m_sign;
}

wpi::units::turn_t FollowerSimSupplier::GetMechanismPosition() {
  return Leader().GetMechanismPosition();
}

wpi::units::turn_t FollowerSimSupplier::GetRotorPosition() {
  return GetMechanismPosition() * RotorRatio();
}

wpi::units::turns_per_second_t FollowerSimSupplier::GetMechanismVelocity() {
  return Leader().GetMechanismVelocity();
}

wpi::units::turns_per_second_t FollowerSimSupplier::GetRotorVelocity() {
  return GetMechanismVelocity() * RotorRatio();
}

wpi::units::turns_per_second_squared_t FollowerSimSupplier::GetMechanismAcceleration() {
  return Leader().GetMechanismAcceleration();
}

wpi::units::turns_per_second_squared_t FollowerSimSupplier::GetRotorAcceleration() {
  return GetMechanismAcceleration() * RotorRatio();
}

wpi::units::ampere_t FollowerSimSupplier::GetStatorCurrent() {
  return Share(Leader().GetStatorCurrent());
}

wpi::units::ampere_t FollowerSimSupplier::GetSupplyCurrent() {
  return Share(Leader().GetSupplyCurrent());
}

}  // namespace yams::motorcontrollers::simulation
