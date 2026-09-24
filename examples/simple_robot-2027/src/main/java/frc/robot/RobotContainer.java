// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.RPM;

import org.wpilib.driverstation.internal.DriverStationBackend;
import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import org.wpilib.command2.button.CommandNiDsXboxController;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class RobotContainer {
  private final SwerveSubsystem drive = new SwerveSubsystem();
  private final ArmSubsystem arm = new ArmSubsystem();
  private final ElevatorSubsystem elevator = new ElevatorSubsystem();
  private final ShooterSubsystem shooter = new ShooterSubsystem();

  private final CommandNiDsXboxController xboxController = new CommandNiDsXboxController(0);

  public RobotContainer() {
    DriverStationBackend.silenceJoystickConnectionAlert(true);
    drive.setDefaultCommand(drive.driveWithJoystick(xboxController));
    arm.setDefaultCommand(arm.setAngle(Degrees.of(0)));
    elevator.setDefaultCommand(elevator.setHeight(Meters.of(0)));
    shooter.setDefaultCommand(shooter.setVelocity(RPM.of(0)));
    configureBindings();
  }

  private void configureBindings() {
    xboxController.getHID().button(1).whileTrue(arm.setAngle(Degrees.of(30)));
    xboxController.getHID().button(2).whileTrue(arm.setAngle(Degrees.of(80)));

    xboxController.getHID().button(3).whileTrue(elevator.setHeight(Meters.of(0.5)));
    xboxController.getHID().button(4).whileTrue(elevator.setHeight(Meters.of(1.5)));

    xboxController.getHID().button(5).whileTrue(shooter.setVelocity(RPM.of(3500)));
    xboxController.getHID().button(6).whileTrue(shooter.setVelocity(RPM.of(2000)));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
