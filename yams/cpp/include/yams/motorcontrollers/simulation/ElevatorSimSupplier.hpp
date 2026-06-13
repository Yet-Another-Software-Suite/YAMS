// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

#pragma once

#include <frc/simulation/ElevatorSim.h>
#include <units/angle.h>
#include <units/angular_acceleration.h>
#include <units/angular_velocity.h>
#include <units/length.h>
#include <units/time.h>
#include <units/velocity.h>
#include <units/voltage.h>

#include <functional>

#include "yams/gearing/MechanismGearing.hpp"
#include "yams/motorcontrollers/SimSupplier.hpp"

namespace yams::motorcontrollers::simulation {

/**
 * SimSupplier backed by a WPILib ElevatorSim.
 *
 * Translates between the linear physics (metres, m/s) of the ElevatorSim and the
 * angular mechanism representation (degrees, dps) expected by the SimSupplier interface.
 * The circumference of the drive sprocket/drum is required to perform this conversion.
 *
 * The duty cycle is read from the motor controller each iteration unless an explicit
 * input voltage has been set.
 */
class ElevatorSimSupplier : public SimSupplier {
 public:
  /**
   * Create an ElevatorSimSupplier.
   *
   * @param sim               WPILib ElevatorSim to advance each loop.
   * @param dutyCycleSupplier Callable returning the current motor duty cycle in [-1, 1].
   * @param gearing           Mechanism gearing used to derive rotor position/velocity.
   * @param circumference     Drum/sprocket circumference (m) for linear↔angular conversion.
   * @param period            Simulation update period.
   */
  ElevatorSimSupplier(frc::sim::ElevatorSim& sim, std::function<double()> dutyCycleSupplier,
                      const gearing::MechanismGearing& gearing, units::meter_t circumference,
                      units::second_t period);

  void UpdateSim() override;
  void SetInputVoltage(units::volt_t volts) override;

  units::turn_t GetMechanismPosition() override;
  units::turns_per_second_t GetMechanismVelocity() override;
  units::turns_per_second_squared_t GetMechanismAcceleration() override;
  units::turn_t GetRotorPosition() override;
  units::turns_per_second_t GetRotorVelocity() override;
  units::turns_per_second_squared_t GetRotorAcceleration() override;

  void SetMechanismPosition(units::turn_t angle) override;
  void SetMechanismVelocity(units::turns_per_second_t velocity) override;
  void SetRotorPosition(units::turn_t angle) override;
  void SetRotorVelocity(units::turns_per_second_t velocity) override;

  bool IsWatchdogExpired() override;
  void FeedWatchdog() override;
  void StarveWatchdog() override;
  units::ampere_t GetCurrentDrawAmps() override;
  units::volt_t GetMechanismSupplyVoltage() override;
  units::volt_t GetMechanismStatorVoltage() override;
  void SetMechanismStatorVoltage(units::volt_t volts) override;

 private:
  frc::sim::ElevatorSim& m_sim;
  std::function<double()> m_dutyCycleSupplier;
  gearing::MechanismGearing m_gearing;
  units::meter_t m_circumference;
  units::second_t m_period;
  bool m_inputFed{false};
  bool m_watchdogFed{false};
  units::volt_t m_lastInputVoltage{0};

  units::turn_t LinearToMechanismAngle(units::meter_t position) const;
  units::turns_per_second_t LinearToMechanismVelocity(units::meters_per_second_t velocity) const;
  units::meter_t MechanismAngleToLinear(units::turn_t angle) const;
  units::meters_per_second_t MechanismVelocityToLinear(units::turns_per_second_t velocity) const;
};

}  // namespace yams::motorcontrollers::simulation
