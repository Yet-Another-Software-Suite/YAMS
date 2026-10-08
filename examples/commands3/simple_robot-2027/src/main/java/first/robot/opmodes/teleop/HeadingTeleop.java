// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package first.robot.opmodes.teleop;

import first.robot.Robot;
import org.wpilib.opmode.OpMode;
import org.wpilib.opmode.Teleop;
import yams.commands3.swerve.SwerveInputStream;
import yams.core.telemetry.enums.TelemetryVerbosity;

/** Teleop where the left stick translates and the robot faces the direction the right stick is pushed. */
@Teleop
public class HeadingTeleop implements OpMode {
  public HeadingTeleop(Robot robot) {
    var controller = robot.xboxController;
    robot.drive.setInputStream(SwerveInputStream.of(
            robot.drive.getSwerveDrive(),
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX())
        // Heading axes are (left, forward), so pushing the right stick away from the driver faces the robot forward.
        .withControllerHeadingAxis(() -> -controller.getRightX(), () -> -controller.getRightY())
        .withHeadingControl(() -> true)
        .withDeadband(0.05)
        .withCubeTranslationControllerAxis()
        .withAllianceRelativeControl()
        .withTelemetry("Driver", TelemetryVerbosity.HIGH));

    // Opmode-scoped default command and bindings: they only exist while this teleop runs.
    robot.drive.setDefaultCommand(robot.drive.driveInputStream());
    TeleopBindings.bindMechanisms(robot);
  }
}
