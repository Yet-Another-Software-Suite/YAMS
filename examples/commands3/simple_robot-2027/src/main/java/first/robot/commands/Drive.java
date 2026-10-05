// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package first.robot.commands;

import first.robot.mechanisms.SwerveMechanism;
import org.wpilib.command3.Command;
import org.wpilib.command3.button.CommandNiDsXboxController;
import yams.commands3.swerve.SwerveInputStream;

/**
 * Drive commands.
 */
public final class Drive {
  private Drive() {
    throw new UnsupportedOperationException("This is a utility class!");
  }

  /**
   * Drive the robot field-relative with the driver controller: the left stick translates and the X
   * axis of the right stick rotates.
   *
   * @param drive      The drivetrain.
   * @param controller Driver controller to read translation/rotation axes from.
   * @return {@link Command} that drives the robot while scheduled.
   */
  public static Command teleop(SwerveMechanism drive, CommandNiDsXboxController controller) {
    return drive.run(coroutine -> {
      SwerveInputStream input = drive.createDriverInput(
          () -> -controller.getLeftY(),
          () -> -controller.getLeftX(),
          () -> -controller.getRightX()
      );
      while (true) {
        drive.driveFromInput(input);
        coroutine.yield();
      }
    }).named("Swerve Drive With Joystick");
  }
}
