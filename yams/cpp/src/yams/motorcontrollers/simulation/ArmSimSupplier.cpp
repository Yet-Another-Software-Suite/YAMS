// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

#include "yams/motorcontrollers/simulation/ArmSimSupplier.hpp"

#include <wpi/simulation/RoboRioSim.hpp>

#include <utility>

namespace yams::motorcontrollers::simulation {

ArmSimSupplier::ArmSimSupplier(wpi::sim::SingleJointedArmSim& sim,
                               std::function<double()> dutyCycleSupplier,
                               const gearing::MechanismGearing& gearing, wpi::units::second_t period)
    : m_sim(sim),
      m_dutyCycleSupplier(std::move(dutyCycleSupplier)),
      m_gearing(gearing),
      m_period(period) {}

void ArmSimSupplier::UpdateSim() {
  if (!m_inputFed) {
    m_lastInputVoltage = wpi::units::volt_t{m_dutyCycleSupplier() * GetMechanismSupplyVoltage().value()};
    m_sim.SetInputVoltage(m_lastInputVoltage);
  }
  m_inputFed = false;
  m_sim.Update(m_period);
  StarveWatchdog();
}

wpi::units::turn_t ArmSimSupplier::GetMechanismPosition() { return m_sim.GetAngle(); }

wpi::units::turns_per_second_t ArmSimSupplier::GetMechanismVelocity() { return m_sim.GetVelocity(); }

wpi::units::turns_per_second_squared_t ArmSimSupplier::GetMechanismAcceleration() {
  return wpi::units::turns_per_second_squared_t{0.0};
}

wpi::units::turn_t ArmSimSupplier::GetRotorPosition() {
  return GetMechanismPosition() * m_gearing.GetMechanismToRotorRatio();
}

wpi::units::turns_per_second_t ArmSimSupplier::GetRotorVelocity() {
  return GetMechanismVelocity() * m_gearing.GetMechanismToRotorRatio();
}

wpi::units::turns_per_second_squared_t ArmSimSupplier::GetRotorAcceleration() {
  return wpi::units::turns_per_second_squared_t{0.0};
}

void ArmSimSupplier::SetMechanismPosition(wpi::units::turn_t angle) {
  m_sim.SetState(wpi::units::radian_t{angle}, m_sim.GetVelocity());
}

void ArmSimSupplier::SetMechanismVelocity(wpi::units::turns_per_second_t velocity) {
  m_sim.SetState(m_sim.GetAngle(), wpi::units::radians_per_second_t{velocity});
}

void ArmSimSupplier::SetRotorPosition(wpi::units::turn_t angle) {
  SetMechanismPosition(angle / m_gearing.GetMechanismToRotorRatio());
}

void ArmSimSupplier::SetRotorVelocity(wpi::units::turns_per_second_t velocity) {
  SetMechanismVelocity(velocity / m_gearing.GetMechanismToRotorRatio());
}

bool ArmSimSupplier::IsWatchdogExpired() { return !m_watchdogFed; }

void ArmSimSupplier::FeedWatchdog() { m_watchdogFed = true; }

void ArmSimSupplier::StarveWatchdog() { m_watchdogFed = false; }

wpi::units::ampere_t ArmSimSupplier::GetCurrentDrawAmps() { return m_sim.GetCurrentDraw(); }

void ArmSimSupplier::SetInputVoltage(wpi::units::volt_t volts) {
  m_lastInputVoltage = volts;
  m_sim.SetInputVoltage(volts);
  m_inputFed = true;
  FeedWatchdog();
}

wpi::units::volt_t ArmSimSupplier::GetMechanismSupplyVoltage() {
  return wpi::sim::RoboRioSim::GetVInVoltage();
}

wpi::units::volt_t ArmSimSupplier::GetMechanismStatorVoltage() { return m_lastInputVoltage; }

void ArmSimSupplier::SetMechanismStatorVoltage(wpi::units::volt_t volts) { SetInputVoltage(volts); }

}  // namespace yams::motorcontrollers::simulation
