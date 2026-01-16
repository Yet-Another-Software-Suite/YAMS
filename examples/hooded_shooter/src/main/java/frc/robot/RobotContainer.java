// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class RobotContainer {
  public VisionSubsystem vision = new VisionSubsystem();
  public ShooterSubsystem shooter = new ShooterSubsystem(vision); // holds hood, flywheel and turret
  public CommandXboxController xboxController = new CommandXboxController(0);

  public RobotContainer() {
    DriverStation.silenceJoystickConnectionWarning(true);
    configureBindings();
  }

  private void configureBindings() {

    xboxController.rightBumper().whileTrue(shooter.runShooter()).whileFalse(shooter.stopShooter());
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
