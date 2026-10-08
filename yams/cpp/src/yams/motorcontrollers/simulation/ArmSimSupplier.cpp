// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

#include "yams/motorcontrollers/simulation/ArmSimSupplier.hpp"

#include <wpi/simulation/RoboRioSim.hpp>

#include "yams/motorcontrollers/SmartMotorController.hpp"
#include "yams/motorcontrollers/simulation/BatterySim.hpp"

namespace yams::motorcontrollers::simulation {

ArmSimSupplier::ArmSimSupplier(wpi::sim::SingleJointedArmSim& sim, SmartMotorController& smc)
    : m_sim(sim),
      m_dutyCycleSupplier([&smc] { return smc.GetDutyCycle(); }),
      m_gearing(smc.GetConfig().GetMotorGearing().value_or(gearing::MechanismGearing::kOne)),
      m_period(smc.GetConfig().GetSimulationPeriod()),
      m_batteryKey(&smc),
      // Based off comment from https://github.com/wpilibsuite/allwpilib/issues/8691
      m_supplyCurrentFilter(wpi::math::LinearFilter<double>::SinglePoleIIR(0.1, m_period)),
      m_rotorAccelFilter(m_period) {}

void ArmSimSupplier::UpdateSim() {
  if (!IsInputFed()) {
    m_lastInputVoltage =
        wpi::units::volt_t{m_dutyCycleSupplier() * GetMechanismSupplyVoltage().value()};
    m_sim.SetInputVoltage(m_lastInputVoltage);
    wpi::sim::RoboRioSim::SetVInVoltage(BatterySim::CalculateVoltage(m_batteryKey, GetSupplyCurrent()));
  }
  if (!m_simUpdated) {
    StarveInput();
    m_sim.Update(m_period);
    FeedUpdateSim();
  }
}

bool ArmSimSupplier::GetUpdatedSim() { return m_simUpdated; }

void ArmSimSupplier::FeedUpdateSim() { m_simUpdated = true; }

void ArmSimSupplier::StarveUpdateSim() { m_simUpdated = false; }

bool ArmSimSupplier::IsInputFed() { return m_inputFed; }

void ArmSimSupplier::FeedInput() { m_inputFed = true; }

void ArmSimSupplier::StarveInput() { m_inputFed = false; }

void ArmSimSupplier::SetMechanismStatorDutyCycle(double dutyCycle) {
  SetMechanismStatorVoltage(wpi::units::volt_t{dutyCycle * GetMechanismSupplyVoltage().value()});
}

wpi::units::volt_t ArmSimSupplier::GetMechanismSupplyVoltage() {
  return wpi::sim::RoboRioSim::GetVInVoltage();
}

wpi::units::volt_t ArmSimSupplier::GetMechanismStatorVoltage() { return m_lastInputVoltage; }

void ArmSimSupplier::SetMechanismStatorVoltage(wpi::units::volt_t volts) {
  FeedInput();
  m_lastInputVoltage = volts;
  m_sim.SetInputVoltage(volts);
}

wpi::units::turn_t ArmSimSupplier::GetMechanismPosition() { return m_sim.GetAngle(); }

wpi::units::turns_per_second_t ArmSimSupplier::GetMechanismVelocity() { return m_sim.GetVelocity(); }

wpi::units::turns_per_second_squared_t ArmSimSupplier::GetMechanismAcceleration() {
  return GetRotorAcceleration() / m_gearing.GetMechanismToRotorRatio();
}

wpi::units::turn_t ArmSimSupplier::GetRotorPosition() {
  return GetMechanismPosition() * m_gearing.GetMechanismToRotorRatio();
}

wpi::units::turns_per_second_t ArmSimSupplier::GetRotorVelocity() {
  return GetMechanismVelocity() * m_gearing.GetMechanismToRotorRatio();
}

wpi::units::turns_per_second_squared_t ArmSimSupplier::GetRotorAcceleration() {
  return wpi::units::turns_per_second_squared_t{
      m_rotorAccelFilter.Derivative(GetRotorVelocity().value())};
}

void ArmSimSupplier::SetMechanismPosition(wpi::units::turn_t angle) { m_sim.SetState(wpi::units::radian_t{angle}, m_sim.GetVelocity()); }

void ArmSimSupplier::SetMechanismVelocity(wpi::units::turns_per_second_t velocity) {
  m_sim.SetState(m_sim.GetAngle(), wpi::units::radians_per_second_t{velocity});
}

void ArmSimSupplier::SetRotorPosition(wpi::units::turn_t angle) {
  SetMechanismPosition(angle / m_gearing.GetMechanismToRotorRatio());
}

void ArmSimSupplier::SetRotorVelocity(wpi::units::turns_per_second_t velocity) {
  SetMechanismVelocity(velocity / m_gearing.GetMechanismToRotorRatio());
}

wpi::units::ampere_t ArmSimSupplier::GetStatorCurrent() { return m_sim.GetCurrentDraw(); }

wpi::units::ampere_t ArmSimSupplier::GetSupplyCurrent() {
  // For a BLDC driven by a switching converter, power is conserved across the duty-cycle
  // transformation, so supplyCurrent = dutyCycle * statorCurrent.
  double dutyCycle = m_dutyCycleSupplier();
  return wpi::units::ampere_t{
      m_supplyCurrentFilter.Calculate(dutyCycle * m_sim.GetCurrentDraw().value())};
}

}  // namespace yams::motorcontrollers::simulation
