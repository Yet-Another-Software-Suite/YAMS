// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.DoubleJointedArmSubsystem;

import static edu.wpi.first.units.Units.Degrees;


public class RobotContainer
{

  private DoubleJointedArmSubsystem jointedArm = new DoubleJointedArmSubsystem();
  public CommandXboxController xboxController = new CommandXboxController(0);

  public RobotContainer()
  {
    DriverStation.silenceJoystickConnectionWarning(true);
    jointedArm.setDefaultCommand(jointedArm.set(0.0, 0.0));
//    jointedArm.setDefaultCommand(jointedArm.setAngle(Degrees.of(0), Degrees.of(0)));
    configureBindings();
  }


  private void configureBindings()
  {
    xboxController.button(1).whileTrue(jointedArm.setAngle(Degrees.of(90), null));
    xboxController.button(2).whileTrue(jointedArm.set(null, 1.0));
    xboxController.button(3).whileTrue(jointedArm.setAngle(Degrees.of(15), Degrees.of(45)));
    xboxController.button(4).whileTrue(jointedArm.setAngle(Degrees.of(180), Degrees.of(90)));
    xboxController.button(5).whileTrue(jointedArm.setAngle(Degrees.of(135), Degrees.of(135)));
    //xboxController.button(6).whileTrue(jointedArm.sysId());
  }


  public Command getAutonomousCommand()
  {
    return Commands.print("No autonomous command configured");
  }
}
