// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

#pragma once

#include <wpi/math/system/DCMotor.hpp>
#include <wpi/commands2/CommandPtr.hpp>
#include <wpi/commands2/SubsystemBase.hpp>
#include <wpi/units/length.hpp>

#include <ctre/phoenix6/TalonFX.hpp>
#include <optional>

#include "yams/gearing/GearBox.hpp"
#include "yams/gearing/MechanismGearing.hpp"
#include "yams/mechanisms/config/ElevatorConfig.hpp"
#include "yams/mechanisms/positional/Elevator.hpp"
#include "yams/motorcontrollers/SmartMotorControllerConfig.hpp"
#include "yams/motorcontrollers/remote/TalonFXWrapper.hpp"

// Elevator driven by a single Kraken X44 (CAN 2) through a 22-tooth sprocket on
// #25 chain (0.25 in pitch). One output sprocket turn moves the carriage 5.5 in
// (0.1397 m); the 3:1 x 4:1 gearbox (12:1 total) sits between motor and sprocket.
// Height is in meters; WithMeasurementLimits clamps motion to [0, 2] m in the
// motor controller, and ElevatorConfig adds a second software ceiling at 3 m.
// The carriage starts at 0.5 m on deploy (seeds the encoder via WithStartingPosition).
//
// Commands:
//   ElevCmd(dutycycle)  -- open-loop percentage output for manual jogging
//   SetHeight(meter_t)  -- closed-loop move, runs continuously (use RunTo variant
//                          inside the Elevator mechanism if you want a finishing command)
class ElevatorSubsystem : public wpi::cmd::SubsystemBase {
 public:
  ElevatorSubsystem();

  wpi::cmd::CommandPtr ElevCmd(double dutycycle);
  wpi::cmd::CommandPtr SetHeight(wpi::units::meter_t height);

  void Periodic() override;
  void SimulationPeriodic() override;

 private:
  ctre::phoenix6::hardware::TalonFX m_elevatorMotor{2, ctre::phoenix6::CANBus{}};  // Kraken X44, CAN 2

  yams::motorcontrollers::SmartMotorControllerConfig m_motorConfig;
  std::optional<yams::motorcontrollers::remote::TalonFXWrapper> m_motor;

  yams::mechanisms::config::ElevatorConfig m_elevatorConfig;
  std::optional<yams::mechanisms::positional::Elevator> m_elevator;
};
