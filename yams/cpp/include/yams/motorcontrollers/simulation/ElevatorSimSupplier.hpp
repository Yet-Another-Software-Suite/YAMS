// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

#pragma once

#include <functional>
#include <wpi/math/filter/LinearFilter.hpp>
#include <wpi/simulation/ElevatorSim.hpp>
#include <wpi/units/angle.hpp>
#include <wpi/units/angular_acceleration.hpp>
#include <wpi/units/angular_velocity.hpp>
#include <wpi/units/current.hpp>
#include <wpi/units/length.hpp>
#include <wpi/units/time.hpp>
#include <wpi/units/velocity.hpp>
#include <wpi/units/voltage.hpp>

#include "yams/gearing/MechanismGearing.hpp"
#include "yams/math/DerivativeTimeFilter.hpp"
#include "yams/motorcontrollers/SimSupplier.hpp"

namespace yams::motorcontrollers {
class SmartMotorController;
}  // namespace yams::motorcontrollers

namespace yams::motorcontrollers::simulation {

/**
 * SimSupplier backed by a WPILib ElevatorSim.
 *
 * Translates between the linear physics (meters, m/s) of the ElevatorSim and the angular
 * mechanism representation expected by the SimSupplier interface, using the mechanism
 * circumference from the motor controller config.  The duty cycle is read from the motor
 * controller each iteration unless an input voltage has been fed this loop.
 */
class ElevatorSimSupplier : public SimSupplier {
 public:
  /**
   * Create a ElevatorSimSupplier.
   *
   * The duty cycle, gearing, simulation period and DC motor are read from @p smc and its
   * config.  The supplier's current draw is tracked in BatterySim under @p smc.
   *
   * @param sim WPILib simulation to advance each loop (must outlive this supplier).
   * @param smc SmartMotorController driving the simulation (must outlive this supplier).
   */
  ElevatorSimSupplier(wpi::sim::ElevatorSim& sim, SmartMotorController& smc);

  void UpdateSim() override;
  bool GetUpdatedSim() override;
  void FeedUpdateSim() override;
  void StarveUpdateSim() override;
  bool IsInputFed() override;
  void FeedInput() override;
  void StarveInput() override;
  void SetMechanismStatorDutyCycle(double dutyCycle) override;
  wpi::units::volt_t GetMechanismSupplyVoltage() override;
  wpi::units::volt_t GetMechanismStatorVoltage() override;
  void SetMechanismStatorVoltage(wpi::units::volt_t volts) override;

  wpi::units::turn_t GetMechanismPosition() override;
  wpi::units::turns_per_second_t GetMechanismVelocity() override;
  wpi::units::turns_per_second_squared_t GetMechanismAcceleration() override;
  wpi::units::turn_t GetRotorPosition() override;
  wpi::units::turns_per_second_t GetRotorVelocity() override;
  wpi::units::turns_per_second_squared_t GetRotorAcceleration() override;

  void SetMechanismPosition(wpi::units::turn_t angle) override;
  void SetMechanismVelocity(wpi::units::turns_per_second_t velocity) override;
  void SetRotorPosition(wpi::units::turn_t angle) override;
  void SetRotorVelocity(wpi::units::turns_per_second_t velocity) override;

  wpi::units::ampere_t GetStatorCurrent() override;
  wpi::units::ampere_t GetSupplyCurrent() override;

 private:
  wpi::sim::ElevatorSim& m_sim;
  std::function<double()> m_dutyCycleSupplier;
  gearing::MechanismGearing m_gearing;
  wpi::units::second_t m_period;
  const void* m_batteryKey;
  wpi::math::LinearFilter<double> m_supplyCurrentFilter;
  bool m_inputFed{false};
  bool m_simUpdated{false};
  wpi::units::volt_t m_lastInputVoltage{0};
  math::DerivativeTimeFilter m_rotorAccelFilter;
  wpi::units::meter_t m_circumference;
};

}  // namespace yams::motorcontrollers::simulation
