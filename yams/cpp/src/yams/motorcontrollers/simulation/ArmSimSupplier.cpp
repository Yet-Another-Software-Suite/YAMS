// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

#include "yams/motorcontrollers/simulation/ArmSimSupplier.hpp"

#include <frc/simulation/RoboRioSim.h>

#include <utility>

namespace yams::motorcontrollers::simulation {

ArmSimSupplier::ArmSimSupplier(frc::sim::SingleJointedArmSim& sim,
                               std::function<double()> dutyCycleSupplier,
                               const gearing::MechanismGearing& gearing, units::second_t period)
    : m_sim(sim),
      m_dutyCycleSupplier(std::move(dutyCycleSupplier)),
      m_gearing(gearing),
      m_period(period) {}

void ArmSimSupplier::UpdateSim() {
  if (!m_inputFed) {
    m_lastInputVoltage = units::volt_t{m_dutyCycleSupplier() * GetMechanismSupplyVoltage().value()};
    m_sim.SetInputVoltage(m_lastInputVoltage);
  }
  m_inputFed = false;
  m_sim.Update(m_period);
  StarveWatchdog();
}

units::turn_t ArmSimSupplier::GetMechanismPosition() { return m_sim.GetAngle(); }

units::turns_per_second_t ArmSimSupplier::GetMechanismVelocity() { return m_sim.GetVelocity(); }

units::turns_per_second_squared_t ArmSimSupplier::GetMechanismAcceleration() {
  return units::turns_per_second_squared_t{0.0};
}

units::turn_t ArmSimSupplier::GetRotorPosition() {
  return GetMechanismPosition() * m_gearing.GetMechanismToRotorRatio();
}

units::turns_per_second_t ArmSimSupplier::GetRotorVelocity() {
  return GetMechanismVelocity() * m_gearing.GetMechanismToRotorRatio();
}

units::turns_per_second_squared_t ArmSimSupplier::GetRotorAcceleration() {
  return units::turns_per_second_squared_t{0.0};
}

void ArmSimSupplier::SetMechanismPosition(units::turn_t angle) {
  m_sim.SetState(units::radian_t{angle}, m_sim.GetVelocity());
}

void ArmSimSupplier::SetMechanismVelocity(units::turns_per_second_t velocity) {
  m_sim.SetState(m_sim.GetAngle(), units::radians_per_second_t{velocity});
}

void ArmSimSupplier::SetRotorPosition(units::turn_t angle) {
  SetMechanismPosition(angle / m_gearing.GetMechanismToRotorRatio());
}

void ArmSimSupplier::SetRotorVelocity(units::turns_per_second_t velocity) {
  SetMechanismVelocity(velocity / m_gearing.GetMechanismToRotorRatio());
}

bool ArmSimSupplier::IsWatchdogExpired() { return !m_watchdogFed; }

void ArmSimSupplier::FeedWatchdog() { m_watchdogFed = true; }

void ArmSimSupplier::StarveWatchdog() { m_watchdogFed = false; }

units::ampere_t ArmSimSupplier::GetCurrentDrawAmps() { return m_sim.GetCurrentDraw(); }

void ArmSimSupplier::SetInputVoltage(units::volt_t volts) {
  m_lastInputVoltage = volts;
  m_sim.SetInputVoltage(volts);
  m_inputFed = true;
  FeedWatchdog();
}

units::volt_t ArmSimSupplier::GetMechanismSupplyVoltage() {
  return frc::sim::RoboRioSim::GetVInVoltage();
}

units::volt_t ArmSimSupplier::GetMechanismStatorVoltage() { return m_lastInputVoltage; }

void ArmSimSupplier::SetMechanismStatorVoltage(units::volt_t volts) { SetInputVoltage(volts); }

}  // namespace yams::motorcontrollers::simulation
