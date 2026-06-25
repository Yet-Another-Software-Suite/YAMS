// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

#pragma once

#include <frc/drive/DifferentialDrive.h>
#include <frc/system/plant/DCMotor.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <rev/SparkMax.h>

#include <functional>
#include <optional>

#include "yams/gearing/GearBox.hpp"
#include "yams/gearing/MechanismGearing.hpp"
#include "yams/motorcontrollers/SmartMotorControllerConfig.hpp"
#include "yams/motorcontrollers/local/SparkWrapper.hpp"

// Two-side differential drivetrain using four NEO motors on REV SPARK Max controllers.
// Each side has a leader + follower (left: CAN 21/22, right: CAN 24/23) through a 12:1
// (3:1 x 4:1) reduction on 4-inch wheels.  Both sides run open-loop (duty cycle).
// Left is inverted; right is not.  COAST idle mode so the robot can be pushed when disabled.
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
class DiffDriveSubsystem : public frc2::SubsystemBase {
 public:
  DiffDriveSubsystem();

  frc2::CommandPtr Stop();
  frc2::CommandPtr TankDrive(std::function<double()> left, std::function<double()> right);
  frc2::CommandPtr ArcadeDrive(std::function<double()> xSpeed, std::function<double()> zRotation);

  void Periodic() override;
  void SimulationPeriodic() override;

 private:
  rev::spark::SparkMax m_leftMotor{21, rev::spark::SparkMax::MotorType::kBrushless};  // left leader
  rev::spark::SparkMax m_rightMotor{24,
                                    rev::spark::SparkMax::MotorType::kBrushless};  // right leader
  rev::spark::SparkMax m_leftFollowerMotor{
      22, rev::spark::SparkMax::MotorType::kBrushless};  // mirrors CAN 21
  rev::spark::SparkMax m_rightFollowerMotor{
      23, rev::spark::SparkMax::MotorType::kBrushless};  // mirrors CAN 24

  yams::motorcontrollers::SmartMotorControllerConfig m_leftConfig;   // inverted=true
  yams::motorcontrollers::SmartMotorControllerConfig m_rightConfig;  // inverted=false

  // SparkWrapper wraps leaders only; duty-cycle callbacks are passed into DifferentialDrive.
  std::optional<yams::motorcontrollers::local::SparkWrapper> m_leftSMC;
  std::optional<yams::motorcontrollers::local::SparkWrapper> m_rightSMC;

  std::optional<frc::DifferentialDrive> m_drive;
};
