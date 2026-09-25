// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later
// Ported from the WCP 2026 Competitive Concept (MIT, see LICENSE-WCP).

package first.robot.commands;

import static org.wpilib.units.Units.Degrees;

import first.robot.Landmarks;
import first.robot.mechanisms.Swerve;
import java.util.function.DoubleSupplier;
import org.wpilib.command3.Command;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.units.measure.Angle;
import yams.core.mechanisms.swerve.utility.SwerveInputStream;

/** Drive with the translation sticks while a YAMS {@link SwerveInputStream} aims at the hub. */
public class AimAndDrive {
    private static final Angle kAimTolerance = Degrees.of(5);

    private final Swerve swerve;
    private final Command command;

    public AimAndDrive(
        Swerve swerve,
        DoubleSupplier forwardInput,
        DoubleSupplier leftInput
    ) {
        this.swerve = swerve;
        final SwerveInputStream input = swerve.createDriverInput(forwardInput, leftInput, () -> 0)
            .withAim(() -> new Pose2d(Landmarks.hubPosition(), Rotation2d.ZERO), () -> true);
        this.command = swerve.driveCommand("Aim And Drive", input);
    }

    public AimAndDrive(Swerve swerve) {
        this(swerve, () -> 0, () -> 0);
    }

    /** Drives and aims until canceled. */
    public Command command() {
        return command;
    }

    public boolean isAimed() {
        return swerve.isFacing(Landmarks.hubPosition(), kAimTolerance);
    }
}
