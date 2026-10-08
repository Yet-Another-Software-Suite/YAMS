// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later
// Ported from the WCP 2026 Competitive Concept (MIT, see LICENSE-WCP).

package first.robot.commands;

import first.robot.Landmarks;
import first.robot.mechanisms.Swerve;
import org.wpilib.command3.Command;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import yams.commands3.swerve.SwerveInputStream;

/**
 * Drive commands for the swerve drivetrain that do not read the driver controller. Teleop driving is built by the
 * teleop opmodes.
 *
 * <p>{@link #holdHeading} keeps the robot still and holding its heading, e.g. between autonomous trajectories.
 * {@link #autoAim} holds position and faces the hub.
 */
public final class Drive {
    private Drive() {
        throw new UnsupportedOperationException("This is a utility class!");
    }

    /**
     * Hold position and the heading the robot had when this command started, until canceled.
     *
     * @param swerve The drivetrain.
     * @return A command that holds the heading until canceled.
     */
    public static Command holdHeading(Swerve swerve) {
        return swerve.run(coroutine -> {
            final SwerveInputStream stream = SwerveInputStream.of(swerve.getSwerveDrive(), () -> 0, () -> 0, () -> 0)
                .withTranslationOnly(() -> true);
            coroutine.await(swerve.driveFieldRelative(stream));
        }).named("Hold Heading");
    }

    /**
     * Autonomous drive command that holds position and faces the hub until canceled.
     *
     * @param swerve The drivetrain.
     * @return A command that aims at the hub until canceled.
     */
    public static Command autoAim(Swerve swerve) {
        return swerve.run(coroutine -> {
            final SwerveInputStream stream = SwerveInputStream.of(swerve.getSwerveDrive(), () -> 0, () -> 0, () -> 0)
                .withAim(() -> new Pose2d(Landmarks.hubPosition(), Rotation2d.ZERO), () -> true);
            coroutine.await(swerve.driveFieldRelative(stream));
        }).named("Auto Aim");
    }
}
