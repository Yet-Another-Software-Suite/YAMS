// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later
// Ported from the WCP 2026 Competitive Concept (MIT, see LICENSE-WCP).

package first.robot.commands;

import first.robot.Constants.Driving;
import first.robot.Landmarks;
import first.robot.mechanisms.Swerve;
import java.util.Optional;
import org.wpilib.command3.Command;
import org.wpilib.command3.button.CommandNiDsXboxController;
import org.wpilib.driverstation.NiDsXboxController;
import org.wpilib.math.filter.Debouncer;
import org.wpilib.math.geometry.Rotation2d;

/**
 * Drive commands for the swerve drivetrain. {@link Swerve} owns one YAMS
 * {@code SwerveInputStream}; each command sets its sticks and modes through {@link Swerve} every
 * loop and then drives from it.
 *
 * <p>{@link #teleop} reads the driver controller. It drives field centric with manual rotation,
 * holds the current heading once the rotation stick has been idle for a short delay, and turns to a
 * heading picked with the A/B/X/Y buttons until the driver rotates manually. Back makes the
 * direction the robot is facing "forward". While the right trigger is held (shoot) the driver still
 * translates, but the heading faces the hub.
 *
 * <p>{@link #autoAim} is the autonomous drive loop: it holds position and faces the hub.
 */
public final class Drive {
    // Matches the right trigger binding for shooting.
    private static final double kAimTriggerThreshold = 0.5;

    private Drive() {
        throw new UnsupportedOperationException("This is a utility class!");
    }

    /**
     * Teleop drive command for the drivetrain.
     *
     * @param swerve     The drivetrain.
     * @param controller The driver's controller.
     * @return A command that drives from the controller until canceled.
     */
    public static Command teleop(Swerve swerve, CommandNiDsXboxController controller) {
        final NiDsXboxController hid = controller.getNiDsXboxController();
        return swerve.run(coroutine -> {
            swerve.resetDriveInput();
            // Rotation stick must be idle for the heading lock delay before the heading is held.
            final Debouncer headingHoldDebouncer = new Debouncer(Driving.kHeadingLockDelaySeconds);
            // Heading picked with the face buttons, from the operator's perspective.
            Optional<Rotation2d> snapHeading = Optional.empty();
            boolean wasRotating = false;

            // Drop button presses from before this command started.
            hid.getAButtonPressed();
            hid.getBButtonPressed();
            hid.getXButtonPressed();
            hid.getYButtonPressed();
            hid.getBackButtonPressed();

            while (true) {
                swerve.setDriveInput(-hid.getLeftY(), -hid.getLeftX(), -hid.getRightX());

                final boolean aimAtHub = hid.getRightTriggerAxis() > kAimTriggerThreshold;
                if (aimAtHub) {
                    // Aiming replaces manual rotation; start fresh once the trigger is released.
                    snapHeading = Optional.empty();
                    wasRotating = false;
                    headingHoldDebouncer.calculate(false);
                    swerve.setAimTarget(Optional.of(Landmarks.hubPosition()));
                    swerve.setHeadingLock(Optional.empty());
                    swerve.setHoldHeading(false);
                } else {
                    if (hid.getAButtonPressed()) {
                        snapHeading = Optional.of(Rotation2d.k180deg);
                    }
                    if (hid.getBButtonPressed()) {
                        snapHeading = Optional.of(Rotation2d.CW_90DEG);
                    }
                    if (hid.getXButtonPressed()) {
                        snapHeading = Optional.of(Rotation2d.CCW_90DEG);
                    }
                    if (hid.getYButtonPressed()) {
                        snapHeading = Optional.of(Rotation2d.ZERO);
                    }
                    if (hid.getBackButtonPressed()) {
                        snapHeading = Optional.empty();
                        swerve.seedFieldCentric();
                    }

                    final boolean rotating = Math.abs(hid.getRightX()) > Driving.kJoystickDeadband;
                    // Rotating manually cancels a picked heading.
                    if (rotating && !wasRotating) {
                        snapHeading = Optional.empty();
                    }
                    wasRotating = rotating;
                    final boolean holdHeading = headingHoldDebouncer.calculate(!rotating);

                    // Face the picked heading, converted to the field frame, or hold the current
                    // heading once the rotation stick is idle.
                    swerve.setAimTarget(Optional.empty());
                    swerve.setHeadingLock(snapHeading.map(heading -> heading.plus(swerve.getOperatorForwardDirection())));
                    swerve.setHoldHeading(holdHeading && snapHeading.isEmpty());
                }

                swerve.driveFromInput();
                coroutine.yield();
            }
        }).named("Teleop Drive");
    }

    /**
     * Autonomous drive command that holds position and faces the hub until canceled.
     *
     * @param swerve The drivetrain.
     * @return A command that aims at the hub until canceled.
     */
    public static Command autoAim(Swerve swerve) {
        return swerve.run(coroutine -> {
            swerve.resetDriveInput();
            while (true) {
                swerve.setAimTarget(Optional.of(Landmarks.hubPosition()));
                swerve.driveFromInput();
                coroutine.yield();
            }
        }).named("Auto Aim");
    }
}
