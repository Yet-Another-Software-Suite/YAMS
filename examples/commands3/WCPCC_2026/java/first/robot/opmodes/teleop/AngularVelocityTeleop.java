// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later
// Ported from the WCP 2026 Competitive Concept (MIT, see LICENSE-WCP).

package first.robot.opmodes.teleop;

import static org.wpilib.units.Units.Seconds;

import first.robot.Constants.Driving;
import first.robot.Landmarks;
import first.robot.Robot;
import first.robot.mechanisms.Swerve;
import java.util.Optional;
import org.wpilib.command3.Command;
import org.wpilib.command3.Trigger;
import org.wpilib.command3.button.CommandNiDsXboxController;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.opmode.OpMode;
import org.wpilib.opmode.Teleop;
import yams.commands3.swerve.SwerveInputStream;
import yams.core.telemetry.enums.TelemetryVerbosity;

/**
 * Teleop with the original WCP drive controls. The left stick translates field centric and the X axis of the right
 * stick sets the rotation rate. The current heading is held once the rotation stick has been idle for a short delay,
 * and A/B/X/Y turn to a heading until the driver rotates manually. While the right trigger is held (shoot) the driver
 * still translates, but the robot faces the hub.
 */
@Teleop(name = "Angular Velocity Teleop")
public class AngularVelocityTeleop implements OpMode {
    /** Heading picked with the face buttons, from the operator's perspective. */
    private Optional<Rotation2d> snapHeading = Optional.empty();

    /**
     * Creates the teleop opmode. The OpModeRobot framework calls this when the opmode is selected on
     * the driver station.
     *
     * @param robot The robot instance to control.
     */
    public AngularVelocityTeleop(Robot robot) {
        final CommandNiDsXboxController driver = robot.driver;
        final Swerve swerve = robot.swerve;

        final Trigger aimAtHub = driver.rightTrigger();
        final Trigger rotating = new Trigger(() -> Math.abs(driver.getRightX()) > Driving.kJoystickDeadband);
        // Rotation stick must be idle for the heading lock delay before the heading is held. Aiming restarts the delay.
        final Trigger holdHeading = rotating.or(aimAtHub).negate().debounce(Seconds.of(Driving.kHeadingLockDelaySeconds));

        final SwerveInputStream inputStream = SwerveInputStream.of(
                swerve.getSwerveDrive(),
                () -> -driver.getLeftY(),
                () -> -driver.getLeftX(),
                () -> -driver.getRightX())
            .withMaximumLinearVelocity(Driving.kMaxSpeed)
            .withMaximumAngularVelocity(Driving.kMaxRotationalRate)
            .withDeadband(Driving.kJoystickDeadband)
            .withCubeTranslationControllerAxis()
            .withCubeRotationControllerAxis()
            .withAllianceRelativeControl()
            .withAim(() -> new Pose2d(Landmarks.hubPosition(), Rotation2d.ZERO), aimAtHub)
            .withHeading(() -> snapHeading.orElse(Rotation2d.ZERO).plus(swerve.getOperatorForwardDirection())
                .getMeasure())
            .withHeadingControl(() -> snapHeading.isPresent() && !aimAtHub.getAsBoolean())
            .withTranslationOnly(() -> holdHeading.getAsBoolean() && snapHeading.isEmpty() && !aimAtHub.getAsBoolean())
            .withTelemetry("Driver", TelemetryVerbosity.HIGH);

        // Opmode-scoped input stream, default command, and bindings: they only exist while this teleop runs.
        swerve.setInputStream(inputStream);
        swerve.setDefaultCommand(swerve.driveInputStream());

        driver.a().and(aimAtHub.negate()).onTrue(snapTo(Rotation2d.k180deg));
        driver.b().and(aimAtHub.negate()).onTrue(snapTo(Rotation2d.CW_90DEG));
        driver.x().and(aimAtHub.negate()).onTrue(snapTo(Rotation2d.CCW_90DEG));
        driver.y().and(aimAtHub.negate()).onTrue(snapTo(Rotation2d.ZERO));
        // Rotating manually, aiming, or resetting field centric cancels a picked heading.
        rotating.or(aimAtHub).or(driver.back()).onTrue(clearSnapHeading());

        TeleopBindings.bindMechanisms(robot);
    }

    /**
     * Turn to a heading from the operator's perspective until the driver rotates manually.
     *
     * @param heading Heading from the operator's perspective.
     * @return {@link Command} that picks the heading and ends.
     */
    private Command snapTo(Rotation2d heading) {
        return Command.noRequirements(coroutine -> snapHeading = Optional.of(heading))
            .named("Snap To " + heading.getDegrees() + " Degrees");
    }

    /**
     * Stop turning to a picked heading.
     *
     * @return {@link Command} that clears the picked heading and ends.
     */
    private Command clearSnapHeading() {
        return Command.noRequirements(coroutine -> snapHeading = Optional.empty()).named("Clear Snap Heading");
    }
}
