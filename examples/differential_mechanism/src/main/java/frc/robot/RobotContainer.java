// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.DiffyMechSubsystem;

import static edu.wpi.first.units.Units.Degrees;


public class RobotContainer
{

  private DiffyMechSubsystem diffyMech = new DiffyMechSubsystem();
  public CommandXboxController xboxController = new CommandXboxController(0);

  public RobotContainer()
  {
    DriverStation.silenceJoystickConnectionWarning(true);
    diffyMech.setDefaultCommand(diffyMech.set(0, 0));
//    diffyMech.setDefaultCommand(diffyMech.setAngle(Degrees.of(0), Degrees.of(0)));
    configureBindings();
  }


  private void configureBindings()
  {
    xboxController.button(1).whileTrue(diffyMech.setAngle(Degrees.of(15), Degrees.of(15)));
    xboxController.button(2).whileTrue(diffyMech.setAngle(Degrees.of(30), Degrees.of(45)));
    xboxController.button(3).whileTrue(diffyMech.set(0,0));
    xboxController.button(4).whileTrue(diffyMech.set(0.5,0));
    xboxController.button(5).whileTrue(diffyMech.set(0,0.5));
  }


  public Command getAutonomousCommand()
  {
    return Commands.print("No autonomous command configured");
  }
}
