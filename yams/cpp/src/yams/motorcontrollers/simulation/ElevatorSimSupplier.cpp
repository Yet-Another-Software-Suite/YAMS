// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

#include "yams/motorcontrollers/simulation/ElevatorSimSupplier.hpp"

#include <wpi/simulation/RoboRioSim.hpp>

#include "yams/motorcontrollers/SmartMotorController.hpp"
#include "yams/motorcontrollers/simulation/BatterySim.hpp"

namespace yams::motorcontrollers::simulation {

ElevatorSimSupplier::ElevatorSimSupplier(wpi::sim::ElevatorSim& sim, SmartMotorController& smc)
    : m_sim(sim),
      m_dutyCycleSupplier([&smc] { return smc.GetDutyCycle(); }),
      m_gearing(smc.GetConfig().GetMotorGearing().value_or(gearing::MechanismGearing::kOne)),
      m_period(smc.GetConfig().GetSimulationPeriod()),
      m_batteryKey(&smc),
      // Based off comment from https://github.com/wpilibsuite/allwpilib/issues/8691
      m_supplyCurrentFilter(wpi::math::LinearFilter<double>::SinglePoleIIR(0.1, m_period)),
      m_rotorAccelFilter(m_period),
      m_circumference(smc.GetConfig().ConvertFromMechanism(wpi::units::turn_t{1.0})),
      m_motor(smc.GetDCMotor()) {}

ElevatorSimSupplier::ElevatorSimSupplier(wpi::sim::ElevatorSim& sim, SmartMotorController& smc,
                                         wpi::units::kilogram_t carriageMass)
    : ElevatorSimSupplier(sim, smc) {
  m_carriageMass = carriageMass.value();
}

void ElevatorSimSupplier::UpdateSim() {
  if (!IsInputFed()) {
    m_lastInputVoltage =
        wpi::units::volt_t{m_dutyCycleSupplier() * GetMechanismSupplyVoltage().value()};
    m_sim.SetInputVoltage(m_lastInputVoltage);
    wpi::sim::RoboRioSim::SetVInVoltage(BatterySim::CalculateVoltage(m_batteryKey, GetSupplyCurrent()));
  }
  if (!m_simUpdated) {
    StarveInput();
    // Carrying game pieces, the simulation is given the voltage that moves the mechanism without them
    // as they move it, for this step, and then the motor's voltage again.
    const double volts = m_sim.GetInput(0);
    const double backEmf =
        wpi::units::radians_per_second_t{GetRotorVelocity()}.value() / m_motor.Kv.value();
    const double simulatedVolts = m_gamePieces.IsEmpty() ? volts : GamePieces::InertiaVoltage(
        volts, backEmf, *m_carriageMass, *m_carriageMass + m_gamePieces.Mass());
    m_sim.SetInput(0, simulatedVolts);
    m_sim.Update(m_period);
    // Unless a closed loop on another thread gave the motor a new voltage meanwhile.
    if (m_sim.GetInput(0) == simulatedVolts) m_sim.SetInput(0, volts);
    FeedUpdateSim();
  }
}

bool ElevatorSimSupplier::GetUpdatedSim() { return m_simUpdated; }

void ElevatorSimSupplier::FeedUpdateSim() { m_simUpdated = true; }

void ElevatorSimSupplier::StarveUpdateSim() { m_simUpdated = false; }

bool ElevatorSimSupplier::IsInputFed() { return m_inputFed; }

void ElevatorSimSupplier::FeedInput() { m_inputFed = true; }

void ElevatorSimSupplier::StarveInput() { m_inputFed = false; }

void ElevatorSimSupplier::SetMechanismStatorDutyCycle(double dutyCycle) {
  SetMechanismStatorVoltage(wpi::units::volt_t{dutyCycle * GetMechanismSupplyVoltage().value()});
}

wpi::units::volt_t ElevatorSimSupplier::GetMechanismSupplyVoltage() {
  return wpi::sim::RoboRioSim::GetVInVoltage();
}

wpi::units::volt_t ElevatorSimSupplier::GetMechanismStatorVoltage() { return m_lastInputVoltage; }

void ElevatorSimSupplier::SetMechanismStatorVoltage(wpi::units::volt_t volts) {
  FeedInput();
  m_lastInputVoltage = volts;
  m_sim.SetInputVoltage(volts);
}

wpi::units::turn_t ElevatorSimSupplier::GetMechanismPosition() { return wpi::units::turn_t{m_sim.GetPosition().value() / m_circumference.value()}; }

wpi::units::turns_per_second_t ElevatorSimSupplier::GetMechanismVelocity() { return wpi::units::turns_per_second_t{m_sim.GetVelocity().value() / m_circumference.value()}; }

wpi::units::turns_per_second_squared_t ElevatorSimSupplier::GetMechanismAcceleration() {
  return GetRotorAcceleration() / m_gearing.GetMechanismToRotorRatio();
}

wpi::units::turn_t ElevatorSimSupplier::GetRotorPosition() {
  return GetMechanismPosition() * m_gearing.GetMechanismToRotorRatio();
}

wpi::units::turns_per_second_t ElevatorSimSupplier::GetRotorVelocity() {
  return GetMechanismVelocity() * m_gearing.GetMechanismToRotorRatio();
}

wpi::units::turns_per_second_squared_t ElevatorSimSupplier::GetRotorAcceleration() {
  return wpi::units::turns_per_second_squared_t{
      m_rotorAccelFilter.Derivative(GetRotorVelocity().value())};
}

void ElevatorSimSupplier::SetMechanismPosition(wpi::units::turn_t angle) { m_sim.SetState(wpi::units::meter_t{angle.value() * m_circumference.value()}, m_sim.GetVelocity()); }

void ElevatorSimSupplier::SetMechanismVelocity(wpi::units::turns_per_second_t velocity) {
  m_sim.SetState(m_sim.GetPosition(),
                 wpi::units::meters_per_second_t{velocity.value() * m_circumference.value()});
}

void ElevatorSimSupplier::SetRotorPosition(wpi::units::turn_t angle) {
  SetMechanismPosition(angle / m_gearing.GetMechanismToRotorRatio());
}

void ElevatorSimSupplier::SetRotorVelocity(wpi::units::turns_per_second_t velocity) {
  SetMechanismVelocity(velocity / m_gearing.GetMechanismToRotorRatio());
}

wpi::units::ampere_t ElevatorSimSupplier::GetStatorCurrent() { return m_sim.GetCurrentDraw(); }

wpi::units::ampere_t ElevatorSimSupplier::GetSupplyCurrent() {
  // For a BLDC driven by a switching converter, power is conserved across the duty-cycle
  // transformation, so supplyCurrent = dutyCycle * statorCurrent.
  double dutyCycle = m_dutyCycleSupplier();
  return wpi::units::ampere_t{
      m_supplyCurrentFilter.Calculate(dutyCycle * m_sim.GetCurrentDraw().value())};
}

void ElevatorSimSupplier::AddGamePiece(wpi::units::kilogram_t mass) {
  // The game piece is on the carriage.
  AddGamePiece(mass, wpi::units::meter_t{0});
}

void ElevatorSimSupplier::AddGamePiece(wpi::units::kilogram_t mass, wpi::units::meter_t distance) {
  // The elevator lifts the game piece wherever it is on the carriage, so the distance does not matter.
  if (!m_carriageMass) {
    throw std::logic_error(
        "This ElevatorSimSupplier cannot simulate a game piece, as it was not given the carriage's "
        "mass.");
  }
  m_gamePieces.Add(mass, distance);
}

void ElevatorSimSupplier::RemoveGamePiece() { m_gamePieces.RemoveLast(); }

}  // namespace yams::motorcontrollers::simulation
