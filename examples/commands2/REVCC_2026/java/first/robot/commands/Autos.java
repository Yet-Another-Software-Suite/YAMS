// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package first.robot.commands;

import first.robot.subsystems.DriveSubsystem;
import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;

public final class Autos
{
  /**
   * Drives the same 's' curve as the REV example auto. WPILib 2027 no longer ships
   * SwerveControllerCommand, so instead of following a generated trajectory the robot drives to each
   * waypoint in turn with the YAMS drive-to-pose controller.
   */
  public static Command exampleAuto(DriveSubsystem drive)
  {
    // Start at the origin facing the +X direction
    Pose2d startPose = new Pose2d(0, 0, Rotation2d.ZERO);

    return Commands.sequence(
        // Reset odometry to the starting pose of the path.
        Commands.runOnce(() -> drive.resetOdometry(startPose), drive),
        // Pass through these two interior waypoints, making an 's' curve path
        drive.driveToPoseCommand(new Pose2d(1, 1, Rotation2d.ZERO)),
        drive.driveToPoseCommand(new Pose2d(2, -1, Rotation2d.ZERO)),
        // End 3 meters straight ahead of where we started, facing forward
        drive.driveToPoseCommand(new Pose2d(3, 0, Rotation2d.ZERO)));
  }

  private Autos()
  {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
