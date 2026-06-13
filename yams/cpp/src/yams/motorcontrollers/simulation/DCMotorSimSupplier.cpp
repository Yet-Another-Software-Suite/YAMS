// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

#include "yams/motorcontrollers/simulation/DCMotorSimSupplier.hpp"

#include <frc/simulation/RoboRioSim.h>

#include <utility>

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
  if (!m_inputFed) {
    m_lastInputVoltage = units::volt_t{m_dutyCycleSupplier() * GetMechanismSupplyVoltage().value()};
    m_sim.SetInputVoltage(m_lastInputVoltage);
  }
  m_inputFed = false;
  m_sim.Update(m_period);
  StarveWatchdog();
}

units::turn_t DCMotorSimSupplier::GetMechanismPosition() { return m_sim.GetAngularPosition(); }

units::turns_per_second_t DCMotorSimSupplier::GetMechanismVelocity() {
  return m_sim.GetAngularVelocity();
}

units::turns_per_second_squared_t DCMotorSimSupplier::GetMechanismAcceleration() {
  return m_sim.GetAngularAcceleration();
}

units::turn_t DCMotorSimSupplier::GetRotorPosition() {
  return GetMechanismPosition() * m_gearing.GetMechanismToRotorRatio();
}

units::turns_per_second_t DCMotorSimSupplier::GetRotorVelocity() {
  return GetMechanismVelocity() * m_gearing.GetMechanismToRotorRatio();
}

units::turns_per_second_squared_t DCMotorSimSupplier::GetRotorAcceleration() {
  return GetMechanismAcceleration() * m_gearing.GetMechanismToRotorRatio();
}

void DCMotorSimSupplier::SetMechanismPosition(units::turn_t angle) { m_sim.SetAngle(angle); }

void DCMotorSimSupplier::SetMechanismVelocity(units::turns_per_second_t velocity) {
  m_sim.SetAngularVelocity(velocity);
}

void DCMotorSimSupplier::SetRotorPosition(units::turn_t angle) {
  SetMechanismPosition(angle / m_gearing.GetMechanismToRotorRatio());
}

void DCMotorSimSupplier::SetRotorVelocity(units::turns_per_second_t velocity) {
  SetMechanismVelocity(velocity / m_gearing.GetMechanismToRotorRatio());
}

bool DCMotorSimSupplier::IsWatchdogExpired() { return !m_watchdogFed; }

void DCMotorSimSupplier::FeedWatchdog() { m_watchdogFed = true; }

void DCMotorSimSupplier::StarveWatchdog() { m_watchdogFed = false; }

units::ampere_t DCMotorSimSupplier::GetCurrentDrawAmps() { return m_sim.GetCurrentDraw(); }

void DCMotorSimSupplier::SetInputVoltage(units::volt_t volts) {
  m_lastInputVoltage = volts;
  m_sim.SetInputVoltage(volts);
  m_inputFed = true;
  FeedWatchdog();
}

units::volt_t DCMotorSimSupplier::GetMechanismSupplyVoltage() {
  return frc::sim::RoboRioSim::GetVInVoltage();
}

units::volt_t DCMotorSimSupplier::GetMechanismStatorVoltage() { return m_lastInputVoltage; }

void DCMotorSimSupplier::SetMechanismStatorVoltage(units::volt_t volts) { SetInputVoltage(volts); }

}  // namespace yams::motorcontrollers::simulation
