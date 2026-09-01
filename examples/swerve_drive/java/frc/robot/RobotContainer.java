// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static org.wpilib.units.Units.Meters;

import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.driverstation.internal.DriverStationBackend;
import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import org.wpilib.command2.button.CommandNiDsXboxController;
import frc.robot.subsystems.SwerveSubsystem;

public class RobotContainer
{
  private final SwerveSubsystem drive = new SwerveSubsystem();

  private final CommandNiDsXboxController xboxController = new CommandNiDsXboxController(0);

  public RobotContainer()
  {
    DriverStationBackend.silenceJoystickConnectionWarning(true);
    drive.setDefaultCommand(drive.drive(drive.getChassisSpeedsSupplier(xboxController::getLeftY,
                                                                       xboxController::getLeftX,
                                                                       xboxController::getRightX)));
    configureBindings();
  }

  private void configureBindings()
  {
//    xboxController.button(1).whileTrue(drive.setRobotRelativeChassisSpeeds(new ChassisVelocities(0.5, 0, 0)));
    xboxController.button(1).whileTrue(drive.azimuthSysId());
    xboxController.button(2).whileTrue(drive.driveSysId());
//    xboxController.button(2).whileTrue(drive.setRobotRelativeChassisSpeeds(new ChassisVelocities(-0.5, 0, 0)));
//    xboxController.button(3).whileTrue(drive.setRobotRelativeChassisSpeeds(new ChassisVelocities(0, 0.5, 0)));
//    xboxController.button(4).whileTrue(drive.setRobotRelativeChassisSpeeds(new ChassisVelocities(0, -0.5, 0)));
    xboxController.button(5).whileTrue(drive.driveToPose(new Pose2d(Meters.of(3),
                                                                    Meters.of(3),
                                                                    Rotation2d.fromDegrees(30))));
    xboxController.button(6).whileTrue(drive.driveToPose(new Pose2d(Meters.of(5),
                                                                    Meters.of(6),
                                                                    Rotation2d.fromDegrees(70))));

  }

  public Command getAutonomousCommand()
  {
    return Commands.print("No autonomous command configured");
  }
}
