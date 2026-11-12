// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package first.robot.opmodes.teleop;

import first.robot.Constants.OIConstants;
import first.robot.Robot;
import first.robot.commands.FuelCommands;
import org.wpilib.command3.button.CommandNiDsXboxController;

/** Bindings shared by every teleop opmode. Call from an opmode constructor so they are opmode-scoped. */
final class TeleopBindings
{
  private TeleopBindings()
  {
    throw new UnsupportedOperationException("This is a utility class!");
  }

  /**
   * Bind the X lock, zero heading, and fuel controls.
   *
   * @param robot The robot.
   */
  static void bind(Robot robot)
  {
    CommandNiDsXboxController controller = robot.driverController;

    // X Button -> Set swerve to X
    controller.x().whileTrue(robot.drive.lockWheels());

    // B Button -> Zero swerve heading
    controller.b().onTrue(robot.drive.zeroHeading());

    // A Button -> Run fuel intake
    controller.a().whileTrue(FuelCommands.intake(robot.intake, robot.conveyor));

    // POV Down -> Run fuel intake in reverse
    controller.getHID().povDown().whileTrue(FuelCommands.extake(robot.intake, robot.conveyor));

    // Y Button -> Run intake and run the shooter flywheel and feeder
    controller.y().toggleOnTrue(
        FuelCommands.shootAndIntake(robot.shooter, robot.feeder, robot.intake, robot.conveyor));
  }
}
