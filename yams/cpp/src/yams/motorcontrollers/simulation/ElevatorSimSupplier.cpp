// Copyright (c) 2026 YAMS Contributors
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
  m_watchdogFed = false;
  if (!m_inputFed) {
    m_sim.SetInputVoltage(
        units::volt_t{m_dutyCycleSupplier() * frc::sim::RoboRioSim::GetVInVoltage().value()});
  }
  m_inputFed = false;
  m_sim.Update(m_period);
}

units::degree_t ElevatorSimSupplier::GetMechanismPosition() {
  return LinearToMechanismAngle(m_sim.GetPosition());
}

units::degrees_per_second_t ElevatorSimSupplier::GetMechanismVelocity() {
  return LinearToMechanismVelocity(m_sim.GetVelocity());
}

units::degrees_per_second_squared_t ElevatorSimSupplier::GetMechanismAcceleration() {
  return units::degrees_per_second_squared_t{0.0};
}

units::degree_t ElevatorSimSupplier::GetRotorPosition() {
  return GetMechanismPosition() * m_gearing.GetMechanismToRotorRatio();
}

units::degrees_per_second_t ElevatorSimSupplier::GetRotorVelocity() {
  return GetMechanismVelocity() * m_gearing.GetMechanismToRotorRatio();
}

units::degrees_per_second_squared_t ElevatorSimSupplier::GetRotorAcceleration() {
  return units::degrees_per_second_squared_t{0.0};
}

void ElevatorSimSupplier::SetMechanismPosition(units::degree_t angle) {
  m_sim.SetState(MechanismAngleToLinear(angle), m_sim.GetVelocity());
}

void ElevatorSimSupplier::SetMechanismVelocity(units::degrees_per_second_t velocity) {
  m_sim.SetState(m_sim.GetPosition(), MechanismVelocityToLinear(velocity));
}

void ElevatorSimSupplier::SetRotorPosition(units::degree_t angle) {
  SetMechanismPosition(angle / m_gearing.GetMechanismToRotorRatio());
}

void ElevatorSimSupplier::SetRotorVelocity(units::degrees_per_second_t velocity) {
  SetMechanismVelocity(velocity / m_gearing.GetMechanismToRotorRatio());
}

bool ElevatorSimSupplier::IsWatchdogExpired() { return !m_watchdogFed; }

void ElevatorSimSupplier::FeedWatchdog() { m_watchdogFed = true; }

void ElevatorSimSupplier::SetInputVoltage(units::volt_t volts) {
  m_sim.SetInputVoltage(volts);
  m_inputFed = true;
}

// ---- Private helpers --------------------------------------------------------

units::degree_t ElevatorSimSupplier::LinearToMechanismAngle(units::meter_t position) const {
  // (meters / circumference) * 360 degrees per revolution
  return units::degree_t{(position.value() / m_circumference.value()) * 360.0};
}

units::degrees_per_second_t ElevatorSimSupplier::LinearToMechanismVelocity(
    units::meters_per_second_t velocity) const {
  return units::degrees_per_second_t{(velocity.value() / m_circumference.value()) * 360.0};
}

units::meter_t ElevatorSimSupplier::MechanismAngleToLinear(units::degree_t angle) const {
  // (degrees / 360) * circumference
  return units::meter_t{(angle.value() / 360.0) * m_circumference.value()};
}

units::meters_per_second_t ElevatorSimSupplier::MechanismVelocityToLinear(
    units::degrees_per_second_t velocity) const {
  return units::meters_per_second_t{(velocity.value() / 360.0) * m_circumference.value()};
}

}  // namespace yams::motorcontrollers::simulation
