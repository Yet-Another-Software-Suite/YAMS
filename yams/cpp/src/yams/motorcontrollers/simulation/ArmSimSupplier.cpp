// Copyright (c) 2026 YAMS Contributors
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
  m_watchdogFed = false;
  if (!m_inputFed) {
    m_sim.SetInputVoltage(
        units::volt_t{m_dutyCycleSupplier() * frc::sim::RoboRioSim::GetVInVoltage().value()});
  }
  m_inputFed = false;
  m_sim.Update(m_period);
}

units::degree_t ArmSimSupplier::GetMechanismPosition() { return m_sim.GetAngle(); }

units::degrees_per_second_t ArmSimSupplier::GetMechanismVelocity() { return m_sim.GetVelocity(); }

units::degrees_per_second_squared_t ArmSimSupplier::GetMechanismAcceleration() {
  return units::degrees_per_second_squared_t{0.0};
}

units::degree_t ArmSimSupplier::GetRotorPosition() {
  return GetMechanismPosition() * m_gearing.GetMechanismToRotorRatio();
}

units::degrees_per_second_t ArmSimSupplier::GetRotorVelocity() {
  return GetMechanismVelocity() * m_gearing.GetMechanismToRotorRatio();
}

units::degrees_per_second_squared_t ArmSimSupplier::GetRotorAcceleration() {
  return units::degrees_per_second_squared_t{0.0};
}

void ArmSimSupplier::SetMechanismPosition(units::degree_t angle) {
  m_sim.SetState(units::radian_t{angle}, m_sim.GetVelocity());
}

void ArmSimSupplier::SetMechanismVelocity(units::degrees_per_second_t velocity) {
  m_sim.SetState(m_sim.GetAngle(), units::radians_per_second_t{velocity});
}

void ArmSimSupplier::SetRotorPosition(units::degree_t angle) {
  SetMechanismPosition(angle / m_gearing.GetMechanismToRotorRatio());
}

void ArmSimSupplier::SetRotorVelocity(units::degrees_per_second_t velocity) {
  SetMechanismVelocity(velocity / m_gearing.GetMechanismToRotorRatio());
}

bool ArmSimSupplier::IsWatchdogExpired() { return !m_watchdogFed; }

void ArmSimSupplier::FeedWatchdog() { m_watchdogFed = true; }

void ArmSimSupplier::SetInputVoltage(units::volt_t volts) {
  m_sim.SetInputVoltage(volts);
  m_inputFed = true;
}

}  // namespace yams::motorcontrollers::simulation
