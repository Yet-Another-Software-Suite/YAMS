// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later
// Ported from the WCP 2026 Competitive Concept (MIT, see LICENSE-WCP).

package first.robot.commands;

import first.robot.Constants.Driving;
import first.robot.subsystems.Swerve;
import java.util.Optional;
import java.util.function.DoubleSupplier;
import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import org.wpilib.command2.button.Trigger;
import org.wpilib.math.geometry.Rotation2d;
import yams.core.mechanisms.swerve.utility.SwerveInputStream;

/**
 * Teleop manual driving for the swerve drivetrain, built on a YAMS {@link SwerveInputStream}.
 *
 * <p>The stream drives field centric with manual rotation, holds the current heading once the
 * rotation stick has been idle for a short delay (translation only mode), and turns to a heading
 * picked with {@link #setLockedHeading} (heading mode) until the driver rotates manually.
 */
public class ManualDrive {
    private final Swerve swerve;
    private final Command command;

    // Heading picked with the face buttons, from the operator's perspective.
    private Optional<Rotation2d> snapHeading = Optional.empty();

    public ManualDrive(
        Swerve swerve,
        DoubleSupplier forwardInput,
        DoubleSupplier leftInput,
        DoubleSupplier rotationInput
    ) {
        this.swerve = swerve;

        final Trigger rotating = new Trigger(() -> Math.abs(rotationInput.getAsDouble()) > Driving.kJoystickDeadband);
        // Rotating manually cancels a picked heading.
        rotating.onTrue(Commands.runOnce(() -> snapHeading = Optional.empty()));

        // SwerveInputStream turns the heading axes into a target with atan2(x, y), so a heading of
        // theta is given as (sin(theta), cos(theta)), in the field frame.
        final SwerveInputStream input = swerve.createDriverInput(forwardInput, leftInput, rotationInput)
            .withTranslationOnly(rotating.negate().debounce(Driving.kHeadingLockDelaySeconds).and(() -> snapHeading.isEmpty()))
            .withControllerHeadingAxis(() -> snapHeadingInField().getSin(), () -> snapHeadingInField().getCos())
            .withHeadingControl(() -> snapHeading.isPresent());

        command = swerve.driveCommand(input).beforeStarting(() -> snapHeading = Optional.empty());
    }

    /** Default drive command for the drivetrain. */
    public Command command() {
        return command;
    }

    /** Turn to and hold a heading from the operator's perspective until the driver rotates. */
    public void setLockedHeading(Rotation2d heading) {
        snapHeading = Optional.of(heading);
    }

    /** Make the direction the robot is facing "forward" and drop any picked heading. */
    public void seedFieldCentric() {
        snapHeading = Optional.empty();
        swerve.seedFieldCentric();
    }

    private Rotation2d snapHeadingInField() {
        return snapHeading.orElse(Rotation2d.ZERO).plus(swerve.getOperatorForwardDirection());
    }
}
