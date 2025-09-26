// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.ShooterSubsystem;

import static edu.wpi.first.units.Units.Meters;


public class RobotContainer
{
  private ShooterSubsystem shooter = new ShooterSubsystem();
  public CommandXboxController xboxController = new CommandXboxController(0);

  public RobotContainer()
  {
    DriverStation.silenceJoystickConnectionWarning(true);
    shooter.setDefaultCommand(shooter.setDutyCycle(0));
    configureBindings();
  }


  private void configureBindings()
  {
    xboxController.button(1).whileTrue(shooter.setVelocity(RPM.of(60)));
    xboxController.button(2).whileTrue(shooter.setVelocity(RPM.of(0)));
    xboxController.button(3).whileTrue(shooter.sysId());
    xboxController.button(4).whileTrue(shooter.setDutyCycle(-0.5));
    xboxController.button(5).whileTrue(shooter.setDutyCycle(0.5));

  }


  public Command getAutonomousCommand()
  {
    return Commands.print("No autonomous command configured");
  }
}
