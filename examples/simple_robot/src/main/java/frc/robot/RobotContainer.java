// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.JointedArmSubsystem;

public class RobotContainer
{

  //  TurretSubsystem       turret         = new TurretSubsystem();
//  ArmSubsystem arm = new ArmSubsystem();
  JointedArmSubsystem dja = new  JointedArmSubsystem();
//  public ElevatorSubsystem elevator = new ElevatorSubsystem();
  public CommandXboxController xboxController = new CommandXboxController(0);


  public RobotContainer()
  {
    DriverStation.silenceJoystickConnectionWarning(true);
//    turret.setDefaultCommand(turret.turretCmd(0.0));
//    arm.setDefaultCommand(arm.armCmd(0));
//    arm.setDefaultCommand(arm.setAngle(Degrees.of(0)));
//    elevator.setDefaultCommand(elevator.elevCmd(0));
    configureBindings();
  }


  private void configureBindings()
  {
//    xboxController.button(1).whileTrue(elevator.setHeight(Meters.of(1)));
//    xboxController.button(2).whileTrue(elevator.setHeight(Meters.of(0)));
//    xboxController.button(3).whileTrue(elevator.sysId());
//    xboxController.button(4).whileTrue(elevator.elevCmd(-0.5));
//    xboxController.button(5).whileTrue(elevator.elevCmd(0.5));

//    xboxController.button(1).whileTrue(arm.armCmd(0.5));
//    xboxController.button(2).whileTrue(arm.armCmd(-0.5));
//    xboxController.button(3).whileTrue(arm.setAngle(Degrees.of(30)));
//    xboxController.button(4).whileTrue(arm.setAngle(Degrees.of(80)));

//    xboxController.button(1).whileTrue(arm.setAngle(Degrees.of(-30)));
//    xboxController.button(2).whileTrue(arm.setAngle(Degrees.of(30)));
//    xboxController.button(3).whileTrue(arm.sysId());

//    xboxController.button(1).whileTrue(turret.turretCmd(0.2));
//    xboxController.button(2).whileTrue(turret.turretCmd(-0.2));
//    xboxController.button(3).whileTrue(turret.sysId());
  }


  public Command getAutonomousCommand()
  {
    return Commands.print("No autonomous command configured");
  }
}
