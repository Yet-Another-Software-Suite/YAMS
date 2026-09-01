// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.wpilib.driverstation.internal.DriverStationBackend;
import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import org.wpilib.command2.button.CommandNiDsXboxController;
import frc.robot.subsystems.ArmSubsystem;

import static org.wpilib.units.Units.Degrees;


public class RobotContainer
{
  private ArmSubsystem arm = new ArmSubsystem();
  public CommandNiDsXboxController xboxController = new CommandNiDsXboxController(0);

  public RobotContainer()
  {
    DriverStationBackend.silenceJoystickConnectionWarning(true);
    arm.setDefaultCommand(arm.armCmd(0));
//    arm.setDefaultCommand(arm.setAngle(Degrees.of(0)));
    configureBindings();
  }


  private void configureBindings()
  {
    xboxController.button(1).whileTrue(arm.armCmd(0.5));
    xboxController.button(2).whileTrue(arm.armCmd(-0.5));
    xboxController.button(3).whileTrue(arm.setAngle(Degrees.of(30)));
    xboxController.button(4).whileTrue(arm.setAngle(Degrees.of(80)));
  }


  public Command getAutonomousCommand()
  {
    return Commands.print("No autonomous command configured");
  }
}
