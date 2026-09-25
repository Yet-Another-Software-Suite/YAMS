// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later
// Ported from the WCP 2026 Competitive Concept (MIT, see LICENSE-WCP).

package first.robot.commands;

import first.robot.Constants.Driving;
import first.robot.mechanisms.Swerve;
import java.util.Optional;
import java.util.function.DoubleSupplier;
import org.wpilib.command3.Command;
import org.wpilib.math.filter.Debouncer;
import org.wpilib.math.geometry.Rotation2d;
import yams.core.mechanisms.swerve.utility.SwerveInputStream;

/**
 * Teleop manual driving for the swerve drivetrain, built on a YAMS {@link SwerveInputStream}.
 *
 * <p>The stream drives field centric with manual rotation, holds the current heading once the
 * rotation stick has been idle for a short delay (translation only mode), and turns to a heading
 * picked with {@link #setLockedHeading} (heading mode) until the driver rotates manually. The v2
 * port used triggers for the rotation edge and the debounce; here the drive coroutine tracks both
 * itself each loop.
 */
public class ManualDrive {
    private final Swerve swerve;
    private final DoubleSupplier rotationInput;
    private final SwerveInputStream input;
    private final Command command;

    // Rotation stick must be idle for the heading lock delay before the heading is held.
    private final Debouncer headingHoldDebouncer = new Debouncer(Driving.kHeadingLockDelaySeconds);
    private boolean holdHeading = false;

    // Heading picked with the face buttons, from the operator's perspective.
    private Optional<Rotation2d> snapHeading = Optional.empty();

    public ManualDrive(
        Swerve swerve,
        DoubleSupplier forwardInput,
        DoubleSupplier leftInput,
        DoubleSupplier rotationInput
    ) {
        this.swerve = swerve;
        this.rotationInput = rotationInput;

        input = swerve.createDriverInput(forwardInput, leftInput, rotationInput)
            .withTranslationOnly(() -> holdHeading && snapHeading.isEmpty())
            // Face the picked heading, converted to the field frame, until it is cleared.
            .withHeading(() -> snapHeadingInField().getMeasure())
            .withHeadingControl(() -> snapHeading.isPresent());

        command = swerve.run(coroutine -> {
            snapHeading = Optional.empty();
            boolean wasRotating = false;
            while (true) {
                final boolean rotating = isRotating();
                // Rotating manually cancels a picked heading.
                if (rotating && !wasRotating) {
                    snapHeading = Optional.empty();
                }
                wasRotating = rotating;
                holdHeading = headingHoldDebouncer.calculate(!rotating);

                swerve.driveFieldRelative(input.get());
                coroutine.yield();
            }
        }).named("Manual Drive");
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

    private boolean isRotating() {
        return Math.abs(rotationInput.getAsDouble()) > Driving.kJoystickDeadband;
    }

    private Rotation2d snapHeadingInField() {
        return snapHeading.orElse(Rotation2d.ZERO).plus(swerve.getOperatorForwardDirection());
    }
}
