// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later
// Ported from the WCP 2026 Competitive Concept (MIT, see LICENSE-WCP).

package first.robot.commands;

import static org.wpilib.units.Units.Degrees;

import first.robot.Constants.Driving;
import first.robot.Landmarks;
import first.robot.subsystems.Swerve;
import first.robot.util.DriveInputSmoother;
import first.robot.util.GeometryUtil;
import first.robot.util.ManualDriveInput;
import java.util.function.DoubleSupplier;
import org.wpilib.command2.Command;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.units.measure.Angle;

public class AimAndDriveCommand extends Command {
    private static final Angle kAimTolerance = Degrees.of(5);

    private final Swerve swerve;
    private final DriveInputSmoother inputSmoother;

    public AimAndDriveCommand(
        Swerve swerve,
        DoubleSupplier forwardInput,
        DoubleSupplier leftInput
    ) {
        this.swerve = swerve;
        this.inputSmoother = new DriveInputSmoother(forwardInput, leftInput);
        addRequirements(swerve);
    }

    public AimAndDriveCommand(Swerve swerve) {
        this(swerve, () -> 0, () -> 0);
    }

    public boolean isAimed() {
        return GeometryUtil.isNear(getDirectionToHub(), swerve.getHeadingInOperatorPerspective(), kAimTolerance);
    }

    /** Direction from the robot to the hub, from the operator's perspective. */
    private Rotation2d getDirectionToHub() {
        final Translation2d hubPosition = Landmarks.hubPosition();
        final Translation2d robotPosition = swerve.getPose().getTranslation();
        final Rotation2d hubDirectionInBlueAlliancePerspective = hubPosition.minus(robotPosition).getAngle().orElse(Rotation2d.ZERO);
        return hubDirectionInBlueAlliancePerspective.minus(swerve.getOperatorForwardDirection());
    }

    @Override
    public void execute() {
        final ManualDriveInput input = inputSmoother.getSmoothedInput();
        swerve.driveFacingAngle(
            Driving.kMaxSpeed.times(input.forward),
            Driving.kMaxSpeed.times(input.left),
            getDirectionToHub()
        );
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
