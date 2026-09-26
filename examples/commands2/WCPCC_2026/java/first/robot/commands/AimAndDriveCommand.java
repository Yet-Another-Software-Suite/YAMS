// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later
// Ported from the WCP 2026 Competitive Concept (MIT, see LICENSE-WCP).

package first.robot.commands;

import static org.wpilib.units.Units.Degrees;

import first.robot.Landmarks;
import first.robot.subsystems.Swerve;
import java.util.function.DoubleSupplier;
import org.wpilib.command2.Command;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.units.measure.Angle;
import yams.commands2.swerve.SwerveInputStream;

/** Drive with the translation sticks while a YAMS {@link SwerveInputStream} aims at the hub. */
public class AimAndDriveCommand extends Command {
    private static final Angle kAimTolerance = Degrees.of(5);

    private final Swerve swerve;
    private final SwerveInputStream input;

    public AimAndDriveCommand(
        Swerve swerve,
        DoubleSupplier forwardInput,
        DoubleSupplier leftInput
    ) {
        this.swerve = swerve;
        this.input = swerve.createDriverInput(forwardInput, leftInput, () -> 0)
            .withAim(() -> new Pose2d(Landmarks.hubPosition(), Rotation2d.ZERO), () -> true);
        addRequirements(swerve);
    }

    public AimAndDriveCommand(Swerve swerve) {
        this(swerve, () -> 0, () -> 0);
    }

    public boolean isAimed() {
        return swerve.isFacing(Landmarks.hubPosition(), kAimTolerance);
    }

    @Override
    public void execute() {
        swerve.driveFieldRelative(input.get());
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
