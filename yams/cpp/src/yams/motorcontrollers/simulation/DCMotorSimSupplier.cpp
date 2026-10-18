// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

#include "yams/motorcontrollers/simulation/DCMotorSimSupplier.hpp"

#include <wpi/simulation/RoboRioSim.hpp>

#include "yams/motorcontrollers/SmartMotorController.hpp"
#include "yams/motorcontrollers/simulation/BatterySim.hpp"

namespace yams::motorcontrollers::simulation {

DCMotorSimSupplier::DCMotorSimSupplier(wpi::sim::DCMotorSim& sim, SmartMotorController& smc)
    : m_sim(sim),
      m_dutyCycleSupplier([&smc] { return smc.GetDutyCycle(); }),
      m_gearing(smc.GetConfig().GetMotorGearing().value_or(gearing::MechanismGearing::kOne)),
      m_period(smc.GetConfig().GetSimulationPeriod()),
      m_batteryKey(&smc),
      // Based off comment from https://github.com/wpilibsuite/allwpilib/issues/8691
      m_supplyCurrentFilter(wpi::math::LinearFilter<double>::SinglePoleIIR(0.1, m_period)),
      m_motor(smc.GetDCMotor()) {}

void DCMotorSimSupplier::UpdateSim() {
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
        volts, backEmf, m_sim.GetJ().value(), m_sim.GetJ().value() + m_gamePieces.Moi());
    m_sim.SetInput(0, simulatedVolts);
    m_sim.Update(m_period);
    // Unless a closed loop on another thread gave the motor a new voltage meanwhile.
    if (m_sim.GetInput(0) == simulatedVolts) m_sim.SetInput(0, volts);
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

wpi::units::ampere_t DCMotorSimSupplier::GetStatorCurrent() { return m_sim.GetCurrentDraw(); }

wpi::units::ampere_t DCMotorSimSupplier::GetSupplyCurrent() {
  // For a BLDC driven by a switching converter, power is conserved across the duty-cycle
  // transformation, so supplyCurrent = dutyCycle * statorCurrent.
  double dutyCycle = m_dutyCycleSupplier();
  return wpi::units::ampere_t{
      m_supplyCurrentFilter.Calculate(dutyCycle * m_sim.GetCurrentDraw().value())};
}

void DCMotorSimSupplier::AddGamePiece(wpi::units::kilogram_t mass) {
  throw std::logic_error(
      "A game piece on a pivot or flywheel needs its distance from the axis: use "
      "AddGamePiece(kilogram_t, meter_t).");
}

void DCMotorSimSupplier::AddGamePiece(wpi::units::kilogram_t mass, wpi::units::meter_t distance) {
  // The game piece adds its mass times its distance from the axis, squared, to the moment of inertia.
  // A pivot or flywheel is not pulled by gravity, so the game piece's weight does not turn it.
  m_gamePieces.Add(mass, distance);
}

void DCMotorSimSupplier::RemoveGamePiece() { m_gamePieces.RemoveLast(); }

}  // namespace yams::motorcontrollers::simulation
