// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

#pragma once

#include <frc/system/plant/DCMotor.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <units/angular_velocity.h>
#include <units/length.h>
#include <units/velocity.h>
#include <units/voltage.h>

#include <ctre/phoenix6/TalonFX.hpp>
#include <functional>
#include <optional>

#include "yams/gearing/GearBox.hpp"
#include "yams/gearing/MechanismGearing.hpp"
#include "yams/motorcontrollers/SmartMotorControllerConfig.hpp"
#include "yams/motorcontrollers/remote/TalonFXWrapper.hpp"

// Two independent Kraken X60 flywheels on TalonFX controllers (lower: CAN 4, upper: CAN 6).
// Both run through a 12:1 (3:1 x 4:1) reduction and are modeled with a 0.00029264 kg*m^2 MOI.
// Closed-loop velocity control via on-board TalonFX PID (kP=1, kI=0, kD=0) with a
// simple feedforward (kS=0, kV=0, kA=0 -- tune before match).  COAST idle mode so wheels
// spin down freely.  Neither wheel follows the other; setpoints are set independently on
// every command so you can spin them at different speeds (e.g. backspin vs topspin).
//
// Commands exposed:
//   SetDutyCycle(lower, upper)     -- open-loop percentages, useful for characterization
//   SetVoltage(lower, upper)       -- fixed-voltage open loop
//   SetVelocity(lower, upper)      -- closed-loop deg/s, static values
//   SetVelocity(lowerFn, upperFn)  -- closed-loop deg/s, live suppliers (e.g. from Dashboard)
class DoubleFlyWheelSubsystem : public frc2::SubsystemBase {
 public:
  DoubleFlyWheelSubsystem();

  frc2::CommandPtr SetDutyCycle(double lower, double upper);
  frc2::CommandPtr SetVoltage(units::volt_t lower, units::volt_t upper);
  // Static setpoints -- command runs until interrupted.
  frc2::CommandPtr SetVelocity(units::degrees_per_second_t lower,
                               units::degrees_per_second_t upper);
  // Supplier overload -- re-evaluates each loop tick; useful for tunable Dashboard targets.
  frc2::CommandPtr SetVelocity(std::function<units::degrees_per_second_t()> lower,
                               std::function<units::degrees_per_second_t()> upper);

  void Periodic() override;
  void SimulationPeriodic() override;

 private:
  ctre::phoenix6::hardware::TalonFX m_talonLower{4};  // CAN ID 4, Kraken X60
  ctre::phoenix6::hardware::TalonFX m_talonUpper{6};  // CAN ID 6, Kraken X60

  yams::motorcontrollers::SmartMotorControllerConfig m_lowerConfig;
  yams::motorcontrollers::SmartMotorControllerConfig m_upperConfig;

  // optional because TalonFXWrapper takes a raw pointer to the TalonFX object,
  // which must already be constructed; both are emplace()'d in the constructor.
  std::optional<yams::motorcontrollers::remote::TalonFXWrapper> m_lowerFlyWheel;
  std::optional<yams::motorcontrollers::remote::TalonFXWrapper> m_upperFlyWheel;
};
