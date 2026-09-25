// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later
// Ported from the WCP 2026 Competitive Concept (MIT, see LICENSE-WCP).

package first.robot.commands;

import first.robot.Constants.Driving;
import first.robot.subsystems.Swerve;
import java.util.Optional;
import java.util.function.DoubleSupplier;
import org.wpilib.command2.Command;
import org.wpilib.math.filter.Debouncer;
import org.wpilib.math.filter.Debouncer.DebounceType;
import org.wpilib.math.geometry.Rotation2d;
import yams.core.mechanisms.swerve.utility.SwerveInputStream;

/**
 * Teleop manual drive command for the swerve drivetrain.
 *
 * Handles field-centric driving with manual rotation input and
 * heading-hold behavior after a short delay once rotation input
 * returns to zero.
 *
 * <p>The driving itself is a YAMS {@link SwerveInputStream}: its translation only mode holds the
 * current heading, and its heading mode snaps to a heading picked with {@link #setLockedHeading}.
 */
public class ManualDriveCommand extends Command {
    private final Swerve swerve;
    private final DoubleSupplier rotationInput;
    private final SwerveInputStream input;

    // True once the rotation stick has been idle for the heading lock delay.
    private final Debouncer rotationIdle = new Debouncer(Driving.kHeadingLockDelaySeconds, DebounceType.RISING);
    // Heading picked with the face buttons, from the operator's perspective.
    private Optional<Rotation2d> snapHeading = Optional.empty();

    public ManualDriveCommand(
        Swerve swerve,
        DoubleSupplier forwardInput,
        DoubleSupplier leftInput,
        DoubleSupplier rotationInput
    ) {
        this.swerve = swerve;
        this.rotationInput = rotationInput;
        this.input = swerve.createDriverInput(forwardInput, leftInput, rotationInput)
            // Hold the current heading once rotation input stops.
            .withTranslationOnly(this::isHoldingHeading)
            // Turn to the snap heading until the driver rotates manually.
            .withControllerHeadingAxis(this::snapHeadingX, this::snapHeadingY)
            .withHeadingControl(this::isSnappingHeading);
        addRequirements(swerve);
    }

    public void seedFieldCentric() {
        initialize();
        swerve.seedFieldCentric();
    }

    /** Turn to and hold a heading from the operator's perspective until the driver rotates. */
    public void setLockedHeading(Rotation2d heading) {
        snapHeading = Optional.of(heading);
    }

    private boolean hasRotationInput() {
        return Math.abs(rotationInput.getAsDouble()) > Driving.kJoystickDeadband;
    }

    private boolean isHoldingHeading() {
        final boolean rotationStopped = rotationIdle.calculate(!hasRotationInput());
        return rotationStopped && snapHeading.isEmpty();
    }

    private boolean isSnappingHeading() {
        if (hasRotationInput()) {
            snapHeading = Optional.empty();
        }
        return snapHeading.isPresent();
    }

    // SwerveInputStream turns the heading axes into a target with atan2(x, y), so a heading of theta
    // is given as (sin(theta), cos(theta)). The snap heading is converted to the field frame first.
    private Rotation2d snapHeadingInField() {
        return snapHeading.orElse(Rotation2d.ZERO).plus(swerve.getOperatorForwardDirection());
    }

    private double snapHeadingX() {
        return snapHeadingInField().getSin();
    }

    private double snapHeadingY() {
        return snapHeadingInField().getCos();
    }

    @Override
    public void initialize() {
        snapHeading = Optional.empty();
    }

    @Override
    public void execute() {
        swerve.driveFieldRelative(input.get());
    }

    @Override
    public void end(boolean interrupted) {
        swerve.stop();
    }

    @Override
    public boolean isFinished() {
        // Default drive command: runs until interrupted
        return false;
    }
}
