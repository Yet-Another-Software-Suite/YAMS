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
      m_rotorAccelFilter(m_period),
      m_motor(smc.GetDCMotor()) {}

ArmSimSupplier::ArmSimSupplier(wpi::sim::SingleJointedArmSim& sim, SmartMotorController& smc,
                               wpi::units::kilogram_square_meter_t moi, wpi::units::meter_t length,
                               bool simulateGravity)
    : ArmSimSupplier(sim, smc) {
  m_armMoi = moi.value();
  m_armLength = length.value();
  m_simulateGravity = simulateGravity;
}

void ArmSimSupplier::UpdateSim() {
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
    const double simulatedVolts = m_gamePieces.IsEmpty() ? volts : m_gamePieces.ArmVoltage(
        volts, backEmf, m_sim.GetAngle().value(), *m_armMoi, *m_armLength, m_simulateGravity, m_motor,
        m_gearing.GetMechanismToRotorRatio());
    m_sim.SetInput(0, simulatedVolts);
    m_sim.Update(m_period);
    // Unless a closed loop on another thread gave the motor a new voltage meanwhile.
    if (m_sim.GetInput(0) == simulatedVolts) m_sim.SetInput(0, volts);
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

void ArmSimSupplier::AddGamePiece(wpi::units::kilogram_t mass) {
  // The game piece is at the end of the arm.
  if (!m_armLength) {
    throw std::logic_error(
        "This ArmSimSupplier cannot simulate a game piece, as it was not given the arm's moment of "
        "inertia and length.");
  }
  AddGamePiece(mass, wpi::units::meter_t{*m_armLength});
}

void ArmSimSupplier::AddGamePiece(wpi::units::kilogram_t mass, wpi::units::meter_t distance) {
  if (!m_armMoi || !m_armLength) {
    throw std::logic_error(
        "This ArmSimSupplier cannot simulate a game piece, as it was not given the arm's moment of "
        "inertia and length.");
  }
  m_gamePieces.Add(mass, distance);
}

void ArmSimSupplier::RemoveGamePiece() { m_gamePieces.RemoveLast(); }

}  // namespace yams::motorcontrollers::simulation
