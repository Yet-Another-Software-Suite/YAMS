// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

#pragma once

#include <rev/SparkMax.h>
#include <wpi/commands2/CommandPtr.hpp>
#include <wpi/commands2/SubsystemBase.hpp>
#include <wpi/hardware/bus/CANPort.hpp>
#include <wpi/units/length.hpp>

#include <optional>

#include "yams/math/ADRCController.hpp"
#include "yams/mechanisms/config/ElevatorConfig.hpp"
#include "yams/mechanisms/positional/Elevator.hpp"
#include "yams/motorcontrollers/SmartMotorControllerConfig.hpp"
#include "yams/motorcontrollers/local/SparkWrapper.hpp"

// An elevator run by an active disturbance rejection controller (ADRC) instead of a PID controller: a NEO (CAN 31) on
// a 12:1 reduction, lifting a 16 lb carriage on a 22 tooth, 1/4 in pitch sprocket. The ADRC was written by FRC Team
// 8331.
//
// An elevator's speed is set by its motor's back-EMF more than by its inertia: each volt moves it at about the same
// speed. So its position is treated as a first order plant, with that speed per volt as b0. The ADRC's observer
// estimates gravity as part of the disturbance, so the elevator needs no kG.
//
// With HIGH telemetry, the ADRC's settings are tunable live in place of kP, kI and kD, under
// /Tuning/ADRCElevator/ElevatorMotor/closedloop/adrc.
class ADRCElevatorSubsystem : public wpi::cmd::SubsystemBase {
 public:
  ADRCElevatorSubsystem();

  // Move the elevator to a height and hold it there.
  wpi::cmd::CommandPtr SetHeight(wpi::units::meter_t height);

  // The elevator's height.
  wpi::units::meter_t GetHeight() const;

  // The elevator's ADRC, built from its ADRCConfig by the motor controller; nullptr until it is configured.
  yams::math::ADRCController* GetADRC();

  void Periodic() override;
  void SimulationPeriodic() override;

 private:
  rev::spark::SparkMax m_elevatorMotor{wpi::CANPort::CAN_S0, 31, rev::spark::SparkMax::MotorType::kBrushless};

  yams::motorcontrollers::SmartMotorControllerConfig m_motorConfig;
  std::optional<yams::motorcontrollers::local::SparkWrapper> m_motor;

  yams::mechanisms::config::ElevatorConfig m_elevatorConfig;
  std::optional<yams::mechanisms::positional::Elevator> m_elevator;
};
