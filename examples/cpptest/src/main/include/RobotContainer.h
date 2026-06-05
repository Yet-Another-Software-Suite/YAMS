// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/button/CommandXboxController.h>

#include "Constants.h"
#include "subsystems/TurretSubsystem.h"

// Uncomment to enable additional subsystems:
// #include "subsystems/ArmSubsystem.h"
// #include "subsystems/ElevatorSubsystem.h"
// #include "subsystems/ShooterSubsystem.h"
// #include "subsystems/HoodSubsystem.h"
// #include "subsystems/SwerveSubsystem.h"
// #include "subsystems/DiffDriveSubsystem.h"
// #include "subsystems/doubleflywheel/DoubleFlyWheelSubsystem.h"

class RobotContainer {
 public:
  RobotContainer();

  frc2::CommandPtr GetAutonomousCommand();

 private:
  frc2::CommandXboxController m_xboxController{OperatorConstants::kDriverControllerPort};

  TurretSubsystem m_turret;

  // ArmSubsystem m_arm;
  // ElevatorSubsystem m_elevator;
  // ShooterSubsystem m_shooter;
  // HoodSubsystem m_hood;
  // SwerveSubsystem m_drive;
  // DiffDriveSubsystem m_diffDrive;
  // DoubleFlyWheelSubsystem m_flyWheelSubsystem;

  void ConfigureBindings();
};
