// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package first.robot.opmodes.teleop;

import first.robot.Constants.OIConstants;
import first.robot.Robot;
import first.robot.commands.FuelCommands;
import org.wpilib.command3.button.CommandNiDsXboxController;
import org.wpilib.opmode.OpMode;
import org.wpilib.opmode.Teleop;

/**
 * Driver controls for the REV ION Starter Bot. The bindings and the drive default command only
 * exist while this opmode is running.
 */
@Teleop
public class DefaultTeleop implements OpMode
{
  /**
   * Creates the teleop opmode. This constructor is called automatically by the OpModeRobot
   * framework when the opmode is selected.
   *
   * @param robot The robot instance to control.
   */
  public DefaultTeleop(Robot robot)
  {
    CommandNiDsXboxController controller = robot.driverController;

    // The left stick controls translation of the robot.
    // Turning is controlled by the X axis of the right stick.
    robot.drive.setDefaultCommand(
        robot.drive.driveCommand(
            robot.drive.getInputStream(
                () -> -controller.getLeftY(),
                () -> -controller.getLeftX(),
                () -> -controller.getRightX(),
                OIConstants.kDriveDeadband),
            true));

    // Left Stick Button -> Set swerve to X
    controller.leftStick().whileTrue(robot.drive.setXCommand());

    // Start Button -> Zero swerve heading
    controller.start().onTrue(robot.drive.zeroHeadingCommand());

    // Right Trigger -> Run fuel intake
    controller
        .rightTrigger(OIConstants.kTriggerButtonThreshold)
        .whileTrue(FuelCommands.intake(robot.intake, robot.conveyor));

    // Left Trigger -> Run fuel intake in reverse
    controller
        .leftTrigger(OIConstants.kTriggerButtonThreshold)
        .whileTrue(FuelCommands.extake(robot.intake, robot.conveyor));

    // Y Button -> Run intake and run the shooter flywheel and feeder
    controller.y().toggleOnTrue(
        FuelCommands.shootAndIntake(robot.shooter, robot.feeder, robot.intake, robot.conveyor));
  }
}
