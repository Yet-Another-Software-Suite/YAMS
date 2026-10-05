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
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import yams.commands3.swerve.SwerveInputStream;

/**
 * Drive commands for the swerve drivetrain.
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
            // Rotation stick must be idle for the heading lock delay before the heading is held.
            final Debouncer headingHoldDebouncer = new Debouncer(Driving.kHeadingLockDelaySeconds);
            // Heading picked with the face buttons, from the operator's perspective.
            final Optional<Rotation2d>[] snapHeading = new Optional[]{Optional.empty()};
            final boolean[] wasRotating = new boolean[]{false};
            final boolean[] holdHeading = new boolean[]{false};
            final boolean[] aimAtHub = new boolean[]{false};

            // Drop button presses from before this command started.
            hid.getAButtonPressed();
            hid.getBButtonPressed();
            hid.getXButtonPressed();
            hid.getYButtonPressed();
            hid.getBackButtonPressed();

            SwerveInputStream stream = swerve.createDriverInput(
                () -> -hid.getLeftY(),
                () -> -hid.getLeftX(),
                () -> -hid.getRightX()
            )
            .withAimTarget(() -> new Pose2d(Landmarks.hubPosition(), Rotation2d.ZERO))
            .setAim(() -> aimAtHub[0])
            .withHeading(() -> {
                if (snapHeading[0].isPresent()) {
                    return snapHeading[0].get().plus(swerve.getOperatorForwardDirection()).getMeasure();
                }
                return Rotation2d.ZERO.getMeasure();
            })
            .setHeadingControl(() -> snapHeading[0].isPresent() && !aimAtHub[0])
            .setTranslationOnly(() -> holdHeading[0] && snapHeading[0].isEmpty() && !aimAtHub[0]);

            while (true) {
                aimAtHub[0] = hid.getRightTriggerAxis() > kAimTriggerThreshold;
                if (aimAtHub[0]) {
                    // Aiming replaces manual rotation; start fresh once the trigger is released.
                    snapHeading[0] = Optional.empty();
                    wasRotating[0] = false;
                    headingHoldDebouncer.calculate(false);
                    holdHeading[0] = false;
                } else {
                    if (hid.getAButtonPressed()) {
                        snapHeading[0] = Optional.of(Rotation2d.k180deg);
                    }
                    if (hid.getBButtonPressed()) {
                        snapHeading[0] = Optional.of(Rotation2d.CW_90DEG);
                    }
                    if (hid.getXButtonPressed()) {
                        snapHeading[0] = Optional.of(Rotation2d.CCW_90DEG);
                    }
                    if (hid.getYButtonPressed()) {
                        snapHeading[0] = Optional.of(Rotation2d.ZERO);
                    }
                    if (hid.getBackButtonPressed()) {
                        snapHeading[0] = Optional.empty();
                        swerve.seedFieldCentric();
                    }

                    final boolean rotating = Math.abs(hid.getRightX()) > Driving.kJoystickDeadband;
                    // Rotating manually cancels a picked heading.
                    if (rotating && !wasRotating[0]) {
                        snapHeading[0] = Optional.empty();
                    }
                    wasRotating[0] = rotating;
                    holdHeading[0] = headingHoldDebouncer.calculate(!rotating);
                }

                swerve.driveFieldRelative(stream.get());
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
            SwerveInputStream stream = swerve.createDriverInput(() -> 0, () -> 0, () -> 0)
                .withAimTarget(() -> new Pose2d(Landmarks.hubPosition(), Rotation2d.ZERO))
                .setAim(true);
            while (true) {
                swerve.driveFieldRelative(stream.get());
                coroutine.yield();
            }
        }).named("Auto Aim");
    }
}
