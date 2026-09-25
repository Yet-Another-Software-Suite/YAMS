// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package first.robot.opmodes.teleop;

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

    // Set the default command for the drive mechanism to arcade drive from the driver
    // controller's joysticks.
    robot.drive.setDefaultCommand(robot.drive.driveArcade(robot.driverController));
  }
}
