// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package first.robot.opmodes.teleop;

import first.robot.Constants.OIConstants;
import first.robot.Robot;
import org.wpilib.command3.button.CommandNiDsXboxController;
import org.wpilib.opmode.OpMode;
import org.wpilib.opmode.Teleop;
import yams.commands3.swerve.SwerveInputStream;

/**
 * Driver controls for the REV ION Starter Bot, where the left stick translates and the X axis of the right stick
 * turns. The bindings and the drive default command only exist while this opmode is running.
 */
@Teleop
public class AngularVelocityTeleop implements OpMode
{
  /**
   * Creates the teleop opmode. This constructor is called automatically by the OpModeRobot
   * framework when the opmode is selected.
   *
   * @param robot The robot instance to control.
   */
  public AngularVelocityTeleop(Robot robot)
  {
    CommandNiDsXboxController controller = robot.driverController;
    robot.drive.setInputStream(SwerveInputStream.of(
            robot.drive.getSwerveDrive(),
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> -controller.getRightX())
        .withDeadband(OIConstants.kDriveDeadband));

    robot.drive.setDefaultCommand(robot.drive.driveInputStream());
    TeleopBindings.bind(robot);
  }
}
