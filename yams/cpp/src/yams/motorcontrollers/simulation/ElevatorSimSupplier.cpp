// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

#include "yams/motorcontrollers/simulation/ElevatorSimSupplier.hpp"

#include <frc/simulation/RoboRioSim.h>

#include <utility>

namespace yams::motorcontrollers::simulation {

ElevatorSimSupplier::ElevatorSimSupplier(frc::sim::ElevatorSim& sim,
                                         std::function<double()> dutyCycleSupplier,
                                         const gearing::MechanismGearing& gearing,
                                         units::meter_t circumference, units::second_t period)
    : m_sim(sim),
      m_dutyCycleSupplier(std::move(dutyCycleSupplier)),
      m_gearing(gearing),
      m_circumference(circumference),
      m_period(period) {}

void ElevatorSimSupplier::UpdateSim() {
  if (!m_inputFed) {
    m_lastInputVoltage = units::volt_t{m_dutyCycleSupplier() * GetMechanismSupplyVoltage().value()};
    m_sim.SetInputVoltage(m_lastInputVoltage);
  }
  m_inputFed = false;
  m_sim.Update(m_period);
  StarveWatchdog();
}

units::turn_t ElevatorSimSupplier::GetMechanismPosition() {
  return LinearToMechanismAngle(m_sim.GetPosition());
}

units::turns_per_second_t ElevatorSimSupplier::GetMechanismVelocity() {
  return LinearToMechanismVelocity(m_sim.GetVelocity());
}

units::turns_per_second_squared_t ElevatorSimSupplier::GetMechanismAcceleration() {
  return units::turns_per_second_squared_t{0.0};
}

units::turn_t ElevatorSimSupplier::GetRotorPosition() {
  return GetMechanismPosition() * m_gearing.GetMechanismToRotorRatio();
}

units::turns_per_second_t ElevatorSimSupplier::GetRotorVelocity() {
  return GetMechanismVelocity() * m_gearing.GetMechanismToRotorRatio();
}

units::turns_per_second_squared_t ElevatorSimSupplier::GetRotorAcceleration() {
  return units::turns_per_second_squared_t{0.0};
}

void ElevatorSimSupplier::SetMechanismPosition(units::turn_t angle) {
  m_sim.SetState(MechanismAngleToLinear(angle), m_sim.GetVelocity());
}

void ElevatorSimSupplier::SetMechanismVelocity(units::turns_per_second_t velocity) {
  m_sim.SetState(m_sim.GetPosition(), MechanismVelocityToLinear(velocity));
}

void ElevatorSimSupplier::SetRotorPosition(units::turn_t angle) {
  SetMechanismPosition(angle / m_gearing.GetMechanismToRotorRatio());
}

void ElevatorSimSupplier::SetRotorVelocity(units::turns_per_second_t velocity) {
  SetMechanismVelocity(velocity / m_gearing.GetMechanismToRotorRatio());
}

bool ElevatorSimSupplier::IsWatchdogExpired() { return !m_watchdogFed; }

void ElevatorSimSupplier::FeedWatchdog() { m_watchdogFed = true; }

void ElevatorSimSupplier::StarveWatchdog() { m_watchdogFed = false; }

units::ampere_t ElevatorSimSupplier::GetCurrentDrawAmps() { return m_sim.GetCurrentDraw(); }

void ElevatorSimSupplier::SetInputVoltage(units::volt_t volts) {
  m_lastInputVoltage = volts;
  m_sim.SetInputVoltage(volts);
  m_inputFed = true;
  FeedWatchdog();
}

units::volt_t ElevatorSimSupplier::GetMechanismSupplyVoltage() {
  return frc::sim::RoboRioSim::GetVInVoltage();
}

units::volt_t ElevatorSimSupplier::GetMechanismStatorVoltage() { return m_lastInputVoltage; }

void ElevatorSimSupplier::SetMechanismStatorVoltage(units::volt_t volts) { SetInputVoltage(volts); }

// ---- Private helpers --------------------------------------------------------

units::turn_t ElevatorSimSupplier::LinearToMechanismAngle(units::meter_t position) const {
  return units::turn_t{position.value() / m_circumference.value()};
}

units::turns_per_second_t ElevatorSimSupplier::LinearToMechanismVelocity(
    units::meters_per_second_t velocity) const {
  return units::turns_per_second_t{velocity.value() / m_circumference.value()};
}

units::meter_t ElevatorSimSupplier::MechanismAngleToLinear(units::turn_t angle) const {
  return units::meter_t{angle.value() * m_circumference.value()};
}

units::meters_per_second_t ElevatorSimSupplier::MechanismVelocityToLinear(
    units::turns_per_second_t velocity) const {
  return units::meters_per_second_t{velocity.value() * m_circumference.value()};
}

}  // namespace yams::motorcontrollers::simulation
