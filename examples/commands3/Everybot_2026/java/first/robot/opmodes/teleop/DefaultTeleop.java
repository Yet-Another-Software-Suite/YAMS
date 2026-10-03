// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package first.robot.opmodes.teleop;

import first.robot.Robot;
import first.robot.commands.FuelCommands;
import org.wpilib.opmode.OpMode;
import org.wpilib.opmode.Teleop;

/** The Everybot's driver controls. All bindings are on the driver controller, as in the original. */
@Teleop
public class DefaultTeleop implements OpMode {
  /**
   * Creates the teleop opmode. The bindings and default command set here only exist while this
   * opmode is running.
   *
   * @param robot The robot instance to control.
   */
  public DefaultTeleop(Robot robot) {
    var controller = robot.driverController;

    // While the left bumper is held, intake Fuel
    controller.leftBumper().whileTrue(FuelCommands.intake(robot.intakeLauncher, robot.indexer));
    // While the right bumper is held, spin up, then launch fuel. When the button is released, stop.
    controller.rightBumper().whileTrue(FuelCommands.launchSequence(robot.intakeLauncher, robot.indexer));
    // While the A button is held, eject fuel back out the intake
    controller.a().whileTrue(FuelCommands.eject(robot.intakeLauncher, robot.indexer));
    // The D-pad triggers live on the generic HID in 2027.
    // While the down arrow on the directional pad is held it will unclimb the robot
    controller.getHID().povDown().whileTrue(robot.climber.climbDown());
    // While the up arrow on the directional pad is held it will climb the robot
    controller.getHID().povUp().whileTrue(robot.climber.climbUp());

    // Arcade drive from the joysticks.
    robot.drive.setDefaultCommand(robot.drive.arcadeDrive(controller));
  }
}
