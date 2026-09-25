// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package first.robot.commands;

import first.robot.mechanisms.SwerveMechanism;
import java.util.Optional;
import org.wpilib.command3.Command;
import org.wpilib.command3.button.CommandNiDsXboxController;
import org.wpilib.driverstation.NiDsXboxController;
import org.wpilib.math.geometry.Pose2d;

/**
 * Drive commands. {@link SwerveMechanism} owns one YAMS {@code SwerveInputStream}; each command sets
 * its sticks through {@link SwerveMechanism} every loop and then drives from it.
 */
public final class Drive
{
  private Drive()
  {
    throw new UnsupportedOperationException("This is a utility class!");
  }

  /**
   * Drive with "standard" swerve drive controls from the driver controller: the left stick
   * translates and the X axis of the right stick rotates. While the left or right bumper is held,
   * the robot PID-drives to that bumper's pose instead. The controller is read every loop.
   *
   * @param drive           The drivetrain.
   * @param controller      Driver controller.
   * @param leftBumperPose  Field-relative pose to drive to while the left bumper is held.
   * @param rightBumperPose Field-relative pose to drive to while the right bumper is held.
   * @return {@link Command} that drives until canceled.
   */
  public static Command teleop(SwerveMechanism drive, CommandNiDsXboxController controller, Pose2d leftBumperPose,
                               Pose2d rightBumperPose)
  {
    final NiDsXboxController hid = controller.getNiDsXboxController();
    return drive.run(coroutine -> {
      drive.resetDriveInput();
      Optional<Pose2d> lastTarget = Optional.empty();
      while (true)
      {
        Optional<Pose2d> target = hid.getLeftBumperButton() ? Optional.of(leftBumperPose)
                                  : hid.getRightBumperButton() ? Optional.of(rightBumperPose)
                                  : Optional.empty();
        if (target.isPresent())
        {
          // Start each drive to pose with fresh PID state.
          if (!target.equals(lastTarget))
          {
            drive.startDriveToPose();
          }
          drive.driveTowardPose(target.get());
        } else
        {
          drive.setDriveInput(hid.getLeftY(), hid.getLeftX(), hid.getRightX());
          drive.driveFromInput();
        }
        lastTarget = target;
        coroutine.yield();
      }
    }).named("Drive With Controller");
  }
}
