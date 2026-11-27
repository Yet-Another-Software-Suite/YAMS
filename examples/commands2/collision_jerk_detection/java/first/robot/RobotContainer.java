// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package first.robot;

import static first.robot.Constants.OperatorConstants.DRIVER_CONTROLLER_PORT;

import first.robot.subsystems.SwerveSubsystem;
import org.wpilib.command2.button.CommandNiDsXboxController;

public class RobotContainer {
  public final SwerveSubsystem swerve = new SwerveSubsystem();
  private final CommandNiDsXboxController driverController =
      new CommandNiDsXboxController(DRIVER_CONTROLLER_PORT);

  public RobotContainer() {
    swerve.setDefaultCommand(
        swerve.driveFieldRelative(
            () -> -driverController.getLeftY(),
            () -> -driverController.getLeftX(),
            () -> -driverController.getRightX()));
  }
}
