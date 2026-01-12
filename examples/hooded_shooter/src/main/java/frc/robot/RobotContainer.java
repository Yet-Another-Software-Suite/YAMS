// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.RPM;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.FlywheelSubsystem;

public class RobotContainer {
  private FlywheelSubsystem flywheel = new FlywheelSubsystem();
  public CommandXboxController xboxController = new CommandXboxController(0);

  public RobotContainer() {
    DriverStation.silenceJoystickConnectionWarning(true);
    flywheel.setDefaultCommand(flywheel.setDutyCycle(0));
    configureBindings();
  }

  private void configureBindings() {
    xboxController.button(1).whileTrue(flywheel.setVelocity(RPM.of(300)));
    xboxController.button(2).whileTrue(flywheel.setVelocity(RPM.of(0)));
    xboxController.button(3).whileTrue(flywheel.sysId());
    xboxController.button(4).whileTrue(flywheel.setDutyCycle(-0.5));
    xboxController.button(5).whileTrue(flywheel.setDutyCycle(0.5));

  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
