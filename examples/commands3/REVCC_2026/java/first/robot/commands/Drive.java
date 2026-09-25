// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package first.robot.commands;

import first.robot.mechanisms.DriveMechanism;
import org.wpilib.command3.Command;
import org.wpilib.command3.button.CommandNiDsXboxController;
import org.wpilib.driverstation.NiDsXboxController;

/**
 * Drive commands. {@link DriveMechanism} owns one YAMS {@code SwerveInputStream}; each command sets
 * its sticks and modes through {@link DriveMechanism} every loop and then drives from it.
 */
public final class Drive
{
  private Drive()
  {
    throw new UnsupportedOperationException("This is a utility class!");
  }

  /**
   * Drive with the driver controller. The left stick controls translation and the X axis of the
   * right stick controls turning. Holding the left stick button sets the wheels into an X formation
   * to prevent movement, and pressing Start zeroes the heading.
   *
   * @param drive         The drivetrain.
   * @param controller    Driver controller.
   * @param fieldRelative Whether the left stick drives relative to the field.
   * @return A command that drives from the controller until canceled.
   */
  public static Command teleop(DriveMechanism drive, CommandNiDsXboxController controller, boolean fieldRelative)
  {
    final NiDsXboxController hid = controller.getNiDsXboxController();
    return drive.run(coroutine -> {
      drive.resetDriveInput();
      drive.setFieldRelative(fieldRelative);
      // Drop a Start press from before this command started.
      hid.getStartButtonPressed();
      while (true)
      {
        if (hid.getStartButtonPressed())
        {
          drive.zeroHeading();
        }
        if (hid.getLeftStickButton())
        {
          drive.lockWheels();
        } else
        {
          drive.setDriveInput(-hid.getLeftY(), -hid.getLeftX(), -hid.getRightX());
          drive.driveFromInput();
        }
        coroutine.yield();
      }
    }).named(fieldRelative ? "Drive.FieldRelative" : "Drive.RobotRelative");
  }
}
