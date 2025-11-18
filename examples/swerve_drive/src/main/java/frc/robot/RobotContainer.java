// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.SwerveSubsystem;

public class RobotContainer
{

  private final SwerveSubsystem drive = new SwerveSubsystem();

  private final CommandXboxController xboxController = new CommandXboxController(0);

  public RobotContainer()
  {
    DriverStation.silenceJoystickConnectionWarning(true);
    drive.setDefaultCommand(drive.drive(drive.getChassisSpeedsSupplier(xboxController::getLeftY,
                                                                       xboxController::getLeftX,
                                                                       xboxController::getRightX)));
    configureBindings();
  }

  private void configureBindings()
  {

    xboxController.button(1).whileTrue(drive.setRobotRelativeChassisSpeeds(new ChassisSpeeds(0.5, 0, 0)));
    xboxController.button(2).whileTrue(drive.setRobotRelativeChassisSpeeds(new ChassisSpeeds(-0.5, 0, 0)));
    xboxController.button(3).whileTrue(drive.setRobotRelativeChassisSpeeds(new ChassisSpeeds(0, 0.5, 0)));
    xboxController.button(4).whileTrue(drive.setRobotRelativeChassisSpeeds(new ChassisSpeeds(0, -0.5, 0)));
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