// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

#pragma once

#include <wpi/commands2/CommandPtr.hpp>
#include <wpi/commands2/button/CommandXboxController.hpp>

#include "Constants.h"
#include "subsystems/TurretSubsystem.h"

// Uncomment to enable additional subsystems:
// #include "subsystems/ArmSubsystem.h"
#include "subsystems/ElevatorSubsystem.h"
// #include "subsystems/ShooterSubsystem.h"
// #include "subsystems/HoodSubsystem.h"
// #include "subsystems/SwerveSubsystem.h"
// #include "subsystems/DiffDriveSubsystem.h"
// #include "subsystems/doubleflywheel/DoubleFlyWheelSubsystem.h"

class RobotContainer {
 public:
  RobotContainer();

  wpi::cmd::CommandPtr GetAutonomousCommand();

 private:
  wpi::cmd::CommandXboxController m_xboxController{OperatorConstants::kDriverControllerPort};

  // TurretSubsystem m_turret;

  // ArmSubsystem m_arm;
  ElevatorSubsystem m_elevator;
  // ShooterSubsystem m_shooter;
  // HoodSubsystem m_hood;
  // SwerveSubsystem m_drive;
  // DiffDriveSubsystem m_diffDrive;
  // DoubleFlyWheelSubsystem m_flyWheelSubsystem;

  void ConfigureBindings();
};
