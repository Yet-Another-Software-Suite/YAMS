// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.wpilib.driverstation.internal.DriverStationBackend;
import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import org.wpilib.command2.button.CommandNiDsXboxController;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class RobotContainer {
  public VisionSubsystem vision = new VisionSubsystem();
  public ShooterSubsystem shooter = new ShooterSubsystem(vision); // holds hood, flywheel and turret
  public CommandNiDsXboxController xboxController = new CommandNiDsXboxController(0);

  public RobotContainer() {
    DriverStationBackend.silenceJoystickConnectionWarning(true);
    configureBindings();
  }

  private void configureBindings() {
    xboxController.rightBumper().whileTrue(shooter.runShooter()).whileFalse(shooter.stopShooter());
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
