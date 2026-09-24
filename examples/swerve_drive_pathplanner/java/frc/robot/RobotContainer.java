// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static org.wpilib.units.Units.Meters;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.events.EventTrigger;
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
    DriverStationBackend.silenceJoystickConnectionAlert(true);
    drive.setDefaultCommand(drive.drive(drive.getChassisSpeedsSupplier(xboxController::getLeftY,
                                                                       xboxController::getLeftX,
                                                                       xboxController::getRightX)));
    new EventTrigger("EventMarker").whileTrue(Commands.print("Something"));
    configureBindings();
  }

  private void configureBindings()
  {
    xboxController.getHID().button(1).whileTrue(drive.setRobotRelativeChassisSpeeds(new ChassisVelocities(0.5, 0, 0)));
    xboxController.getHID().button(2).whileTrue(drive.setRobotRelativeChassisSpeeds(new ChassisVelocities(-0.5, 0, 0)));
    xboxController.getHID().button(3).whileTrue(drive.setRobotRelativeChassisSpeeds(new ChassisVelocities(0, 0.5, 0)));
    xboxController.getHID().button(4).whileTrue(drive.setRobotRelativeChassisSpeeds(new ChassisVelocities(0, -0.5, 0)));
    xboxController.getHID().button(5).whileTrue(drive.driveToPose(new Pose2d(Meters.of(3),
                                                                    Meters.of(3),
                                                                    Rotation2d.fromDegrees(30))));
    xboxController.getHID().button(6).whileTrue(drive.driveToPose(new Pose2d(Meters.of(5),
                                                                    Meters.of(6),
                                                                    Rotation2d.fromDegrees(70))));

  }

  public Command getAutonomousCommand()
  {
    return new PathPlannerAuto("New Auto");
  }
}
