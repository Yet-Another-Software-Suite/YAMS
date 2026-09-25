// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package first.robot.opmodes.teleop;

import static first.robot.Constants.OperatorConstants.*;

import first.robot.Robot;
import org.wpilib.opmode.OpMode;
import org.wpilib.opmode.Teleop;

/**
 * Teleop for the 2026 FIRST KitBot. The bindings and the drive default command are created when
 * this opmode is selected and are removed when it exits.
 */
@Teleop(name = "KitBot Teleop")
public class KitBotTeleop implements OpMode {
  /**
   * Creates the teleop opmode. This constructor is called automatically by the OpModeRobot
   * framework when the opmode is selected on the driver station.
   *
   * @param robot The robot instance to control.
   */
  public KitBotTeleop(Robot robot) {
    // While the left bumper on operator controller is held, intake Fuel
    robot.operatorController.leftBumper()
        .whileTrue(robot.fuel.intake());
    // While the right bumper on the operator controller is held, spin up for 1
    // second, then launch fuel. When the button is released, stop.
    robot.operatorController.rightBumper()
        .whileTrue(robot.fuel.spinUpAndLaunch());
    // While the A button is held on the operator controller, eject fuel back out
    // the intake
    robot.operatorController.a()
        .whileTrue(robot.fuel.eject());

    // Set the default command for the drive mechanism to the command provided by
    // factory with the values provided by the joystick axes on the driver
    // controller. The Y axis of the controller is inverted so that pushing the
    // stick away from you (a negative value) drives the robot forwards (a positive
    // value). The X-axis is also inverted so a positive value (stick to the right)
    // results in clockwise rotation (front of the robot turning right). Both axes
    // are also scaled down so the rotation is more easily controllable.
    robot.drive.setDefaultCommand(
        robot.drive.driveArcade(
            () -> -robot.driverController.getLeftY() * DRIVE_SCALING,
            () -> -robot.driverController.getRightX() * ROTATION_SCALING));
  }
}
