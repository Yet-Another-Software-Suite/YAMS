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
import yams.commands2.swerve.SwerveInputStream;

/**
 * Teleop manual driving for the swerve drivetrain, built on a YAMS {@link SwerveInputStream}.
 *
 * <p>The stream drives field centric with manual rotation, holds the current heading once the
 * rotation stick has been idle for a short delay (translation only mode), and turns to a heading
 * picked with {@link #setLockedHeading} (heading mode) until the driver rotates manually. Like
 * WCP's {@code ManualDriveCommand}, it is the drivetrain's default command.
 */
public class ManualDrive extends Command {
    private final Swerve swerve;
    private final SwerveInputStream input;

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

        input = swerve.createDriverInput(forwardInput, leftInput, rotationInput)
            .withTranslationOnly(rotating.negate().debounce(Driving.kHeadingLockDelaySeconds).and(() -> snapHeading.isEmpty()))
            // Face the picked heading, converted to the field frame, until it is cleared.
            .withHeading(() -> snapHeadingInField().getMeasure())
            .withHeadingControl(() -> snapHeading.isPresent());

        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        snapHeading = Optional.empty();
    }

    @Override
    public void execute() {
        swerve.driveFieldRelative(input.get());
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
