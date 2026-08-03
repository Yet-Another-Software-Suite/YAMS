// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

#include "yams/motorcontrollers/simulation/ElevatorSimSupplier.hpp"

#include <wpi/simulation/RoboRioSim.hpp>

#include <utility>

namespace yams::motorcontrollers::simulation {

ElevatorSimSupplier::ElevatorSimSupplier(wpi::sim::ElevatorSim& sim,
                                         std::function<double()> dutyCycleSupplier,
                                         const gearing::MechanismGearing& gearing,
                                         wpi::units::meter_t circumference, wpi::units::second_t period)
    : m_sim(sim),
      m_dutyCycleSupplier(std::move(dutyCycleSupplier)),
      m_gearing(gearing),
      m_circumference(circumference),
      m_period(period) {}

void ElevatorSimSupplier::UpdateSim() {
  if (!m_inputFed) {
    m_lastInputVoltage = wpi::units::volt_t{m_dutyCycleSupplier() * GetMechanismSupplyVoltage().value()};
    m_sim.SetInputVoltage(m_lastInputVoltage);
  }
  m_inputFed = false;
  m_sim.Update(m_period);
  StarveWatchdog();
}

wpi::units::turn_t ElevatorSimSupplier::GetMechanismPosition() {
  return LinearToMechanismAngle(m_sim.GetPosition());
}

wpi::units::turns_per_second_t ElevatorSimSupplier::GetMechanismVelocity() {
  return LinearToMechanismVelocity(m_sim.GetVelocity());
}

wpi::units::turns_per_second_squared_t ElevatorSimSupplier::GetMechanismAcceleration() {
  return wpi::units::turns_per_second_squared_t{0.0};
}

wpi::units::turn_t ElevatorSimSupplier::GetRotorPosition() {
  return GetMechanismPosition() * m_gearing.GetMechanismToRotorRatio();
}

wpi::units::turns_per_second_t ElevatorSimSupplier::GetRotorVelocity() {
  return GetMechanismVelocity() * m_gearing.GetMechanismToRotorRatio();
}

wpi::units::turns_per_second_squared_t ElevatorSimSupplier::GetRotorAcceleration() {
  return wpi::units::turns_per_second_squared_t{0.0};
}

void ElevatorSimSupplier::SetMechanismPosition(wpi::units::turn_t angle) {
  m_sim.SetState(MechanismAngleToLinear(angle), m_sim.GetVelocity());
}

void ElevatorSimSupplier::SetMechanismVelocity(wpi::units::turns_per_second_t velocity) {
  m_sim.SetState(m_sim.GetPosition(), MechanismVelocityToLinear(velocity));
}

void ElevatorSimSupplier::SetRotorPosition(wpi::units::turn_t angle) {
  SetMechanismPosition(angle / m_gearing.GetMechanismToRotorRatio());
}

void ElevatorSimSupplier::SetRotorVelocity(wpi::units::turns_per_second_t velocity) {
  SetMechanismVelocity(velocity / m_gearing.GetMechanismToRotorRatio());
}

bool ElevatorSimSupplier::IsWatchdogExpired() { return !m_watchdogFed; }

void ElevatorSimSupplier::FeedWatchdog() { m_watchdogFed = true; }

void ElevatorSimSupplier::StarveWatchdog() { m_watchdogFed = false; }

wpi::units::ampere_t ElevatorSimSupplier::GetCurrentDrawAmps() { return m_sim.GetCurrentDraw(); }

void ElevatorSimSupplier::SetInputVoltage(wpi::units::volt_t volts) {
  m_lastInputVoltage = volts;
  m_sim.SetInputVoltage(volts);
  m_inputFed = true;
  FeedWatchdog();
}

wpi::units::volt_t ElevatorSimSupplier::GetMechanismSupplyVoltage() {
  return wpi::sim::RoboRioSim::GetVInVoltage();
}

wpi::units::volt_t ElevatorSimSupplier::GetMechanismStatorVoltage() { return m_lastInputVoltage; }

void ElevatorSimSupplier::SetMechanismStatorVoltage(wpi::units::volt_t volts) { SetInputVoltage(volts); }

// ---- Private helpers --------------------------------------------------------

wpi::units::turn_t ElevatorSimSupplier::LinearToMechanismAngle(wpi::units::meter_t position) const {
  return wpi::units::turn_t{position.value() / m_circumference.value()};
}

wpi::units::turns_per_second_t ElevatorSimSupplier::LinearToMechanismVelocity(
    wpi::units::meters_per_second_t velocity) const {
  return wpi::units::turns_per_second_t{velocity.value() / m_circumference.value()};
}

wpi::units::meter_t ElevatorSimSupplier::MechanismAngleToLinear(wpi::units::turn_t angle) const {
  return wpi::units::meter_t{angle.value() * m_circumference.value()};
}

wpi::units::meters_per_second_t ElevatorSimSupplier::MechanismVelocityToLinear(
    wpi::units::turns_per_second_t velocity) const {
  return wpi::units::meters_per_second_t{velocity.value() * m_circumference.value()};
}

}  // namespace yams::motorcontrollers::simulation
