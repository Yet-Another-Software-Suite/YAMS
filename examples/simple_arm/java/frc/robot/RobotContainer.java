// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.ArmSubsystem;

import static edu.wpi.first.units.Units.Degrees;


public class RobotContainer
{

  private ArmSubsystem arm = new ArmSubsystem();
  public CommandXboxController xboxController = new CommandXboxController(0);

  public RobotContainer()
  {
    DriverStation.silenceJoystickConnectionWarning(true);
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
    xboxController.button(5).whileTrue(arm.sysId());
  }


  public Command getAutonomousCommand()
  {
    return Commands.print("No autonomous command configured");
  }
}
