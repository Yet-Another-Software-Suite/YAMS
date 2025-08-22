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
//  private final ArmSubsystem              arm = new ArmSubsystem();
//  private final DoubleJointedArmSubsystem jointedArm = new DoubleJointedArmSubsystem();
//  private final ElevatorSubsystem         elevator = new ElevatorSubsystem();
//  private final ShooterSubsystem          shooter = new ShooterSubsystem();
//  private final TurretSubsystem           turret         = new TurretSubsystem();

  private final CommandXboxController     xboxController = new CommandXboxController(0);

  public RobotContainer()
  {
    DriverStation.silenceJoystickConnectionWarning(true);
//    arm.setDefaultCommand(arm.armCmd(0));
//    arm.setDefaultCommand(arm.setAngle(Degrees.of(0)));
//    jointedArm.setDefaultCommand(jointedArm.setAngle(Degrees.of(90), Degrees.of(0)));
//    elevator.setDefaultCommand(elevator.elevCmd(0));
//    turret.setDefaultCommand(turret.turretCmd(0.0));
    configureBindings();
  }

  private void configureBindings()
  {
//    xboxController.button(1).whileTrue(elevator.setHeight(Meters.of(1)));
//    xboxController.button(2).whileTrue(elevator.setHeight(Meters.of(0)));
//    xboxController.button(3).whileTrue(elevator.sysId());
//    xboxController.button(4).whileTrue(elevator.elevCmd(-0.5));
//    xboxController.button(5).whileTrue(elevator.elevCmd(0.5));

//    xboxController.button(1).whileTrue(jointedArm.setAngle(Degrees.of(90), null));
//    xboxController.button(2).whileTrue(jointedArm.set(null, 1.0));
//    xboxController.button(3).whileTrue(jointedArm.setAngle(Degrees.of(15), Degrees.of(45)));
//    xboxController.button(4).whileTrue(jointedArm.setAngle(Degrees.of(180), Degrees.of(90)));
//    xboxController.button(5).whileTrue(jointedArm.setAngle(Degrees.of(135), Degrees.of(135)));

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