// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

#include "RobotContainer.h"

#include <wpi/driverstation/internal/DriverStationBackend.hpp>
#include <wpi/commands2/Commands.hpp>
#include <wpi/commands2/button/Trigger.hpp>
#include <wpi/units/angle.hpp>

RobotContainer::RobotContainer() {
  wpi::internal::DriverStationBackend::SilenceJoystickConnectionAlert(true);

  // m_flyWheelSubsystem.SetDefaultCommand(m_flyWheelSubsystem.SetDutyCycle(0, 0));
  // m_arm.SetDefaultCommand(m_arm.ArmCmd(0));
  m_elevator.SetDefaultCommand(m_elevator.ElevCmd(0));
  // m_turret.SetDefaultCommand(m_turret.TurretCmd(0.0));
  //  m_drive.SetDefaultCommand(m_drive.SetRobotRelativeChassisSpeeds(wpi::math::ChassisVelocities{}));

  ConfigureBindings();
}

void RobotContainer::ConfigureBindings() {
  // Shooter bindings (uncomment with ShooterSubsystem):
  // m_xboxController.A().WhileTrue(m_shooter.SetVelocity(wpi::units::degrees_per_second_t{6000}));
  // m_xboxController.B().WhileTrue(m_shooter.SetVelocity(wpi::units::degrees_per_second_t{-6000}));
  // m_xboxController.X().WhileTrue(m_shooter.Set(0.0));
  // m_xboxController.Y().WhileTrue(m_shooter.Set(0.5));

  // Swerve bindings (uncomment with SwerveSubsystem):
  // m_xboxController.A().WhileTrue(m_drive.SetRobotRelativeChassisSpeeds({0.5_mps, 0_mps,
  // 0_rad_per_s})); m_xboxController.LeftBumper().WhileTrue(m_drive.DriveToPose(wpi::math::Pose2d{3_m, 3_m,
  // wpi::math::Rotation2d{30_deg}}));

  // Arm bindings (uncomment with ArmSubsystem):
  // m_xboxController.A().WhileTrue(m_arm.ArmCmd(0.5));
  // m_xboxController.B().WhileTrue(m_arm.ArmCmd(-0.5));
  // m_xboxController.X().WhileTrue(m_arm.SetAngle(wpi::units::degree_t{30}));
  // m_xboxController.Y().WhileTrue(m_arm.SetAngle(wpi::units::degree_t{80}));

  // Elevator bindings (uncomment with ElevatorSubsystem):
  m_xboxController.A().WhileTrue(m_elevator.SetHeight(1_m));
  m_xboxController.B().WhileTrue(m_elevator.SetHeight(0_m));
  m_xboxController.X().WhileTrue(m_elevator.ElevCmd(0.8));

  // Turret bindings:
  // m_xboxController.A().WhileTrue(m_turret.TurretCmd(1));
  // m_xboxController.B().WhileTrue(m_turret.TurretCmd(-1));
}

wpi::cmd::CommandPtr RobotContainer::GetAutonomousCommand() {
  return wpi::cmd::Print("No autonomous command configured");
}
