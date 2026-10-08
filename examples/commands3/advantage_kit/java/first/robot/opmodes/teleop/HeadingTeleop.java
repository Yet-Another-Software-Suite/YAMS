// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package first.robot.opmodes.teleop;

import first.robot.Robot;
import yams.commands3.swerve.SwerveInputStream;

/**
 * Teleop where the left stick translates and the robot faces the direction the right stick is pushed. Created by
 * {@link Robot#teleopInit()}, since a LoggedRobot does not run WPILib opmodes.
 */
public class HeadingTeleop {
  public HeadingTeleop(Robot robot) {
    var controller = robot.xboxController;
    robot.drive.setInputStream(SwerveInputStream.of(
            robot.drive.getSwerveDrive(),
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX())
        // Heading axes are (left, forward), so pushing the right stick away from the driver faces the robot forward.
        .withControllerHeadingAxis(() -> -controller.getRightX(), () -> -controller.getRightY())
        .withHeadingControl(() -> true)
        // 0.01 deadband eliminates stick drift without adding noticeable dead zone.
        .withDeadband(0.01)
        .withCubeTranslationControllerAxis()
        // Alliance-relative: forward on the stick always moves toward the opposing
        // alliance wall regardless of which side the robot started on.
        .withAllianceRelativeControl());

    robot.drive.setDefaultCommand(robot.drive.driveInputStream());
  }
}
