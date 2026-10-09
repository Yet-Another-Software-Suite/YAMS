// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

#pragma once

#include <wpi/drive/DifferentialDrive.hpp>
#include <wpi/math/system/DCMotor.hpp>
#include <wpi/commands2/CommandPtr.hpp>
#include <wpi/commands2/SubsystemBase.hpp>
#include <rev/SparkMax.h>
#include <wpi/hardware/bus/CANPort.hpp>

#include <functional>
#include <optional>

#include "yams/gearing/GearBox.hpp"
#include "yams/gearing/MechanismGearing.hpp"
#include "yams/motorcontrollers/SmartMotorControllerConfig.hpp"
#include "yams/motorcontrollers/local/SparkWrapper.hpp"

// Two-side differential drivetrain using four NEO motors on REV SPARK Max controllers.
// Each side has a leader + follower (left: CAN 21/22, right: CAN 24/23) through a 12:1
// (3:1 x 4:1) reduction on 4-inch wheels.  Both sides run open-loop (duty cycle).
// Left is inverted; right is not.  COAST zero power mode so the robot can be pushed when disabled.
//
// SparkWrapper only wraps the leader on each side; follower mode is configured directly
// on the hardware objects (CAN 22 follows 21, CAN 23 follows 24).
//
// Stop() is the default command, so the drive halts when no other command is scheduled.
//
// Commands exposed:
//   Stop()                       -- kills output each loop tick
//   TankDrive(left, right)       -- independent left/right duty-cycle suppliers
//   ArcadeDrive(xSpeed, zRotation) -- combined translation + rotation suppliers
class DiffDriveSubsystem : public wpi::cmd::SubsystemBase {
 public:
  DiffDriveSubsystem();

  wpi::cmd::CommandPtr Stop();
  wpi::cmd::CommandPtr TankDrive(std::function<double()> left, std::function<double()> right);
  wpi::cmd::CommandPtr ArcadeDrive(std::function<double()> xSpeed, std::function<double()> zRotation);

  void Periodic() override;
  void SimulationPeriodic() override;

 private:
  rev::spark::SparkMax m_leftMotor{wpi::CANPort::CAN_S0, 21, rev::spark::SparkMax::MotorType::kBrushless};  // left leader
  rev::spark::SparkMax m_rightMotor{wpi::CANPort::CAN_S0, 24, rev::spark::SparkMax::MotorType::kBrushless};  // right leader
  rev::spark::SparkMax m_leftFollowerMotor{wpi::CANPort::CAN_S0, 22, rev::spark::SparkMax::MotorType::kBrushless};  // mirrors CAN 21
  rev::spark::SparkMax m_rightFollowerMotor{wpi::CANPort::CAN_S0, 23, rev::spark::SparkMax::MotorType::kBrushless};  // mirrors CAN 24

  yams::motorcontrollers::SmartMotorControllerConfig m_leftConfig;   // inverted=true
  yams::motorcontrollers::SmartMotorControllerConfig m_rightConfig;  // inverted=false

  // SparkWrapper wraps leaders only; duty-cycle callbacks are passed into DifferentialDrive.
  std::optional<yams::motorcontrollers::local::SparkWrapper> m_leftSMC;
  std::optional<yams::motorcontrollers::local::SparkWrapper> m_rightSMC;

  std::optional<wpi::DifferentialDrive> m_drive;
};
