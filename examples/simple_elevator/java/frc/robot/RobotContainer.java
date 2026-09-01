// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.wpilib.driverstation.internal.DriverStationBackend;
import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import org.wpilib.command2.button.CommandNiDsXboxController;
import frc.robot.subsystems.ElevatorSubsystem;

import static org.wpilib.units.Units.Meters;


public class RobotContainer
{
  public ElevatorSubsystem elevator = new ElevatorSubsystem();
  public CommandNiDsXboxController xboxController = new CommandNiDsXboxController(0);

  public RobotContainer()
  {
    DriverStationBackend.silenceJoystickConnectionWarning(true);
    elevator.setDefaultCommand(elevator.elevCmd(0));
    configureBindings();
  }


  private void configureBindings()
  {
    xboxController.button(1).whileTrue(elevator.setHeight(Meters.of(1)));
    xboxController.button(2).whileTrue(elevator.setHeight(Meters.of(0)));
    xboxController.button(4).whileTrue(elevator.elevCmd(-0.5));
    xboxController.button(5).whileTrue(elevator.elevCmd(0.5));

  }


  public Command getAutonomousCommand()
  {
    return Commands.print("No autonomous command configured");
  }
}
