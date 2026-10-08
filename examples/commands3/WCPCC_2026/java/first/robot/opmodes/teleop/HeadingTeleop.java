// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later
// Ported from the WCP 2026 Competitive Concept (MIT, see LICENSE-WCP).

package first.robot.opmodes.teleop;

import first.robot.Constants.Driving;
import first.robot.Landmarks;
import first.robot.Robot;
import first.robot.mechanisms.Swerve;
import org.wpilib.command3.button.CommandNiDsXboxController;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.opmode.OpMode;
import org.wpilib.opmode.Teleop;
import yams.commands3.swerve.SwerveInputStream;
import yams.core.telemetry.enums.TelemetryVerbosity;

/**
 * Teleop where the left stick translates field centric and the robot faces the direction the right stick is pushed.
 * While the right trigger is held (shoot) the driver still translates, but the robot faces the hub.
 */
@Teleop(name = "Heading Teleop")
public class HeadingTeleop implements OpMode {
    /**
     * Creates the teleop opmode. The OpModeRobot framework calls this when the opmode is selected on
     * the driver station.
     *
     * @param robot The robot instance to control.
     */
    public HeadingTeleop(Robot robot) {
        final CommandNiDsXboxController driver = robot.driver;
        final Swerve swerve = robot.swerve;

        final SwerveInputStream inputStream = SwerveInputStream.of(
                swerve.getSwerveDrive(),
                () -> -driver.getLeftY(),
                () -> -driver.getLeftX())
            // Heading axes are (left, forward), so pushing the right stick away from the driver faces the robot
            // forward.
            .withControllerHeadingAxis(() -> -driver.getRightX(), () -> -driver.getRightY())
            .withHeadingControl(() -> true)
            .withMaximumLinearVelocity(Driving.kMaxSpeed)
            .withMaximumAngularVelocity(Driving.kMaxRotationalRate)
            .withDeadband(Driving.kJoystickDeadband)
            .withCubeTranslationControllerAxis()
            .withAllianceRelativeControl()
            .withAim(() -> new Pose2d(Landmarks.hubPosition(), Rotation2d.ZERO), driver.rightTrigger())
            .withTelemetry("Driver", TelemetryVerbosity.HIGH);

        // Opmode-scoped input stream, default command, and bindings: they only exist while this teleop runs.
        swerve.setInputStream(inputStream);
        swerve.setDefaultCommand(swerve.driveInputStream());
        TeleopBindings.bindMechanisms(robot);
    }
}
