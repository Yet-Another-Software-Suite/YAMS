// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

#include "yams/motorcontrollers/simulation/DCMotorSimSupplier.hpp"

#include <wpi/driverstation/RobotState.hpp>
#include <wpi/simulation/RoboRioSim.hpp>

#include "yams/motorcontrollers/SmartMotorController.hpp"
#include "yams/motorcontrollers/simulation/BatterySim.hpp"

namespace yams::motorcontrollers::simulation {

DCMotorSimSupplier::DCMotorSimSupplier(wpi::sim::DCMotorSim& sim, SmartMotorController& smc)
    : m_sim(sim),
      // A mechanism's motors apply each motor controller's own output.
      m_dutyCycleSupplier([&smc] { return smc.GetMechanismDutyCycle(smc.GetDutyCycle()); }),
      m_mechanismDutyCycle([&smc](double dutyCycle) { return smc.GetMechanismDutyCycle(dutyCycle); }),
      m_gearing(smc.GetConfig().GetMotorGearing().value_or(gearing::MechanismGearing::kOne)),
      m_period(smc.GetConfig().GetSimulationPeriod()),
      m_batteryKey(&smc),
      // Based off comment from https://github.com/wpilibsuite/allwpilib/issues/8691
      m_supplyCurrentFilter(wpi::math::LinearFilter<double>::SinglePoleIIR(0.1, m_period)) {}

void DCMotorSimSupplier::UpdateSim() {
  if (!IsInputFed()) {
    m_lastInputVoltage =
        wpi::units::volt_t{m_dutyCycleSupplier() * GetMechanismSupplyVoltage().value()};
    m_sim.SetInputVoltage(m_lastInputVoltage);
    wpi::sim::RoboRioSim::SetVInVoltage(BatterySim::CalculateVoltage(m_batteryKey, GetSupplyCurrent()));
  }
  if (!m_simUpdated) {
    StarveInput();
    if (!wpi::RobotState::IsEnabled()) {
      // A disabled robot's motors get no power, so the mechanism coasts or falls.
      m_lastInputVoltage = wpi::units::volt_t{0};
      m_sim.SetInputVoltage(m_lastInputVoltage);
    }
    // Read before the step, which uses up the input voltage the current is drawn with.
    m_statorCurrent = m_sim.GetCurrentDraw();
    m_supplyCurrent = wpi::units::ampere_t{
        m_supplyCurrentFilter.Calculate(m_dutyCycleSupplier() * m_statorCurrent.value())};
    m_sim.Update(m_period);
    FeedUpdateSim();
  }
}

bool DCMotorSimSupplier::GetUpdatedSim() { return m_simUpdated; }

void DCMotorSimSupplier::FeedUpdateSim() { m_simUpdated = true; }

void DCMotorSimSupplier::StarveUpdateSim() { m_simUpdated = false; }

bool DCMotorSimSupplier::IsInputFed() { return m_inputFed; }

void DCMotorSimSupplier::FeedInput() { m_inputFed = true; }

void DCMotorSimSupplier::StarveInput() { m_inputFed = false; }

void DCMotorSimSupplier::SetMechanismStatorDutyCycle(double dutyCycle) {
  SetMechanismStatorVoltage(wpi::units::volt_t{dutyCycle * GetMechanismSupplyVoltage().value()});
}

wpi::units::volt_t DCMotorSimSupplier::GetMechanismSupplyVoltage() {
  return wpi::sim::RoboRioSim::GetVInVoltage();
}

wpi::units::volt_t DCMotorSimSupplier::GetMechanismStatorVoltage() { return m_lastInputVoltage; }

void DCMotorSimSupplier::SetMechanismStatorVoltage(wpi::units::volt_t volts) {
  FeedInput();
  // A mechanism's motors apply each motor controller's own output.
  const double supplyVolts = GetMechanismSupplyVoltage().value();
  if (supplyVolts != 0) volts = supplyVolts * wpi::units::volt_t{m_mechanismDutyCycle(volts.value() / supplyVolts)};
  m_lastInputVoltage = volts;
  m_sim.SetInputVoltage(volts);
}

wpi::units::turn_t DCMotorSimSupplier::GetMechanismPosition() { return m_sim.GetAngularPosition(); }

wpi::units::turns_per_second_t DCMotorSimSupplier::GetMechanismVelocity() { return m_sim.GetAngularVelocity(); }

wpi::units::turns_per_second_squared_t DCMotorSimSupplier::GetMechanismAcceleration() {
  return m_sim.GetAngularAcceleration();
}

wpi::units::turn_t DCMotorSimSupplier::GetRotorPosition() {
  return GetMechanismPosition() * m_gearing.GetMechanismToRotorRatio();
}

wpi::units::turns_per_second_t DCMotorSimSupplier::GetRotorVelocity() {
  return GetMechanismVelocity() * m_gearing.GetMechanismToRotorRatio();
}

wpi::units::turns_per_second_squared_t DCMotorSimSupplier::GetRotorAcceleration() {
  return GetMechanismAcceleration() * m_gearing.GetMechanismToRotorRatio();
}

void DCMotorSimSupplier::SetMechanismPosition(wpi::units::turn_t angle) { m_sim.SetAngle(angle); }

void DCMotorSimSupplier::SetMechanismVelocity(wpi::units::turns_per_second_t velocity) {
  m_sim.SetAngularVelocity(velocity);
}

void DCMotorSimSupplier::SetRotorPosition(wpi::units::turn_t angle) {
  SetMechanismPosition(angle / m_gearing.GetMechanismToRotorRatio());
}

void DCMotorSimSupplier::SetRotorVelocity(wpi::units::turns_per_second_t velocity) {
  SetMechanismVelocity(velocity / m_gearing.GetMechanismToRotorRatio());
}

wpi::units::ampere_t DCMotorSimSupplier::GetStatorCurrent() { return m_statorCurrent; }

wpi::units::ampere_t DCMotorSimSupplier::GetSupplyCurrent() {
  // For a BLDC driven by a switching converter, power is conserved across the duty-cycle
  // transformation, so supplyCurrent = dutyCycle * statorCurrent.
  // It is filtered once per step, in UpdateSim().
  return m_supplyCurrent;
}

}  // namespace yams::motorcontrollers::simulation
