// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package first.robot;

import first.robot.Constants.OperatorConstants;
import first.robot.subsystems.SwerveSubsystem;
import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import org.wpilib.command2.button.CommandNiDsXboxController;
import yams.commands2.swerve.SwerveInputStream;

/** Wires the driver controller to the skid-monitored swerve drive. */
public class RobotContainer {
  private final SwerveSubsystem drive = new SwerveSubsystem();
  private final CommandNiDsXboxController driverController =
      new CommandNiDsXboxController(OperatorConstants.DRIVER_CONTROLLER_PORT);

  public RobotContainer() {
    SwerveInputStream input =
        drive.getChassisSpeedsSupplier(
            () -> -driverController.getLeftY(),
            () -> -driverController.getLeftX(),
            () -> -driverController.getRightX());
    drive.setDefaultCommand(drive.driveFieldRelative(input));
    driverController.y().onTrue(drive.zeroGyro());
  }

  public Command getAutonomousCommand() {
    return Commands.none();
  }
}
