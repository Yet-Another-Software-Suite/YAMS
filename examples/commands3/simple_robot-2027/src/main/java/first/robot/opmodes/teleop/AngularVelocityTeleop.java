// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package first.robot.opmodes.teleop;

import first.robot.Robot;
import org.wpilib.opmode.OpMode;
import org.wpilib.opmode.Teleop;
import yams.commands3.swerve.SwerveInputStream;
import yams.core.telemetry.enums.TelemetryVerbosity;

/** Teleop where the left stick translates and the X axis of the right stick sets the rotation rate. */
@Teleop
public class AngularVelocityTeleop implements OpMode {
  public AngularVelocityTeleop(Robot robot) {
    var controller = robot.xboxController;
    robot.drive.setInputStream(SwerveInputStream.of(
            robot.drive.getSwerveDrive(),
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> -controller.getRightX())
        .withDeadband(0.05)
        .withCubeTranslationControllerAxis()
        .withAllianceRelativeControl()
        .withTelemetry("Driver", TelemetryVerbosity.HIGH));

    // Opmode-scoped default command and bindings: they only exist while this teleop runs.
    robot.drive.setDefaultCommand(robot.drive.driveInputStream());
    TeleopBindings.bindMechanisms(robot);
  }
}
