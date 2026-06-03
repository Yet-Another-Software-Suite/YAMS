// Copyright (c) 2026 YAMS Contributors
// SPDX-License-Identifier: LGPL-3.0-or-later

#include "yams/motorcontrollers/simulation/DCMotorSimSupplier.h"

#include <frc/simulation/RoboRioSim.h>

namespace yams::motorcontrollers::simulation {

DCMotorSimSupplier::DCMotorSimSupplier(frc::sim::DCMotorSim& sim,
                                       std::function<double()> dutyCycleSupplier,
                                       const gearing::MechanismGearing& gearing,
                                       units::second_t period)
    : m_sim(sim),
      m_dutyCycleSupplier(std::move(dutyCycleSupplier)),
      m_gearing(gearing),
      m_period(period) {}

void DCMotorSimSupplier::UpdateSim() {
  m_watchdogFed = false;
  if (!m_inputFed) {
    m_sim.SetInputVoltage(
        units::volt_t{m_dutyCycleSupplier() * frc::sim::RoboRioSim::GetVInVoltage().value()});
  }
  m_inputFed = false;
  m_sim.Update(m_period);
}

units::degree_t DCMotorSimSupplier::GetMechanismPosition() {
  return m_sim.GetAngularPosition();
}

units::degrees_per_second_t DCMotorSimSupplier::GetMechanismVelocity() {
  return m_sim.GetAngularVelocity();
}

units::degrees_per_second_squared_t DCMotorSimSupplier::GetMechanismAcceleration() {
  return m_sim.GetAngularAcceleration();
}

units::degree_t DCMotorSimSupplier::GetRotorPosition() {
  return GetMechanismPosition() * m_gearing.GetMechanismToRotorRatio();
}

units::degrees_per_second_t DCMotorSimSupplier::GetRotorVelocity() {
  return GetMechanismVelocity() * m_gearing.GetMechanismToRotorRatio();
}

units::degrees_per_second_squared_t DCMotorSimSupplier::GetRotorAcceleration() {
  return GetMechanismAcceleration() * m_gearing.GetMechanismToRotorRatio();
}

void DCMotorSimSupplier::SetMechanismPosition(units::degree_t angle) {
  m_sim.SetAngle(angle);
}

void DCMotorSimSupplier::SetMechanismVelocity(units::degrees_per_second_t velocity) {
  m_sim.SetAngularVelocity(velocity);
}

void DCMotorSimSupplier::SetRotorPosition(units::degree_t angle) {
  SetMechanismPosition(angle / m_gearing.GetMechanismToRotorRatio());
}

void DCMotorSimSupplier::SetRotorVelocity(units::degrees_per_second_t velocity) {
  SetMechanismVelocity(velocity / m_gearing.GetMechanismToRotorRatio());
}

bool DCMotorSimSupplier::IsWatchdogExpired() { return !m_watchdogFed; }

void DCMotorSimSupplier::FeedWatchdog() { m_watchdogFed = true; }

void DCMotorSimSupplier::SetInputVoltage(units::volt_t volts) {
  m_sim.SetInputVoltage(volts);
  m_inputFed = true;
}

}  // namespace yams::motorcontrollers::simulation
