// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

#include "RobotContainer.h"

#include <frc/DriverStation.h>
#include <frc2/command/Commands.h>
#include <frc2/command/button/Trigger.h>
#include <units/angle.h>

RobotContainer::RobotContainer() {
  frc::DriverStation::SilenceJoystickConnectionWarning(true);

  // m_flyWheelSubsystem.SetDefaultCommand(m_flyWheelSubsystem.SetDutyCycle(0, 0));
  // m_arm.SetDefaultCommand(m_arm.ArmCmd(0));
  m_elevator.SetDefaultCommand(m_elevator.ElevCmd(0));
  // m_turret.SetDefaultCommand(m_turret.TurretCmd(0.0));
  //  m_drive.SetDefaultCommand(m_drive.SetRobotRelativeChassisSpeeds(frc::ChassisSpeeds{}));

  ConfigureBindings();
}

void RobotContainer::ConfigureBindings() {
  // Shooter bindings (uncomment with ShooterSubsystem):
  // m_xboxController.Button(1).WhileTrue(m_shooter.SetVelocity(units::degrees_per_second_t{6000}));
  // m_xboxController.Button(2).WhileTrue(m_shooter.SetVelocity(units::degrees_per_second_t{-6000}));
  // m_xboxController.Button(3).WhileTrue(m_shooter.Set(0.0));
  // m_xboxController.Button(4).WhileTrue(m_shooter.Set(0.5));

  // Swerve bindings (uncomment with SwerveSubsystem):
  // m_xboxController.Button(1).WhileTrue(m_drive.SetRobotRelativeChassisSpeeds({0.5_mps, 0_mps,
  // 0_rad_per_s})); m_xboxController.Button(5).WhileTrue(m_drive.DriveToPose(frc::Pose2d{3_m, 3_m,
  // frc::Rotation2d{30_deg}}));

  // Arm bindings (uncomment with ArmSubsystem):
  // m_xboxController.Button(1).WhileTrue(m_arm.ArmCmd(0.5));
  // m_xboxController.Button(2).WhileTrue(m_arm.ArmCmd(-0.5));
  // m_xboxController.Button(3).WhileTrue(m_arm.SetAngle(units::degree_t{30}));
  // m_xboxController.Button(4).WhileTrue(m_arm.SetAngle(units::degree_t{80}));

  // Elevator bindings (uncomment with ElevatorSubsystem):
  m_xboxController.Button(1).WhileTrue(m_elevator.SetHeight(1_m));
  m_xboxController.Button(2).WhileTrue(m_elevator.SetHeight(0_m));
  m_xboxController.Button(3).WhileTrue(m_elevator.ElevCmd(0.8));

  // Turret bindings:
  // m_xboxController.Button(1).WhileTrue(m_turret.TurretCmd(1));
  // m_xboxController.Button(2).WhileTrue(m_turret.TurretCmd(-1));
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  return frc2::cmd::Print("No autonomous command configured");
}
