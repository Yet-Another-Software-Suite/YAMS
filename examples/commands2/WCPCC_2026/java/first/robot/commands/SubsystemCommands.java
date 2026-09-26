// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later
// Ported from the WCP 2026 Competitive Concept (MIT, see LICENSE-WCP).

package first.robot.commands;

import java.util.function.DoubleSupplier;

import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import first.robot.subsystems.Feeder;
import first.robot.subsystems.Floor;
import first.robot.subsystems.Hanger;
import first.robot.subsystems.Hood;
import first.robot.subsystems.IntakePivot;
import first.robot.subsystems.IntakeRollers;
import first.robot.subsystems.Shooter;
import first.robot.subsystems.Swerve;

public final class SubsystemCommands {
    private final Swerve swerve;
    private final IntakePivot intakePivot;
    private final IntakeRollers intakeRollers;
    private final Floor floor;
    private final Feeder feeder;
    private final Shooter shooter;
    private final Hood hood;
    private final Hanger hanger;

    private final DoubleSupplier forwardInput;
    private final DoubleSupplier leftInput;

    public SubsystemCommands(
        Swerve swerve,
        IntakePivot intakePivot,
        IntakeRollers intakeRollers,
        Floor floor,
        Feeder feeder,
        Shooter shooter,
        Hood hood,
        Hanger hanger,
        DoubleSupplier forwardInput,
        DoubleSupplier leftInput
    ) {
        this.swerve = swerve;
        this.intakePivot = intakePivot;
        this.intakeRollers = intakeRollers;
        this.floor = floor;
        this.feeder = feeder;
        this.shooter = shooter;
        this.hood = hood;
        this.hanger = hanger;

        this.forwardInput = forwardInput;
        this.leftInput = leftInput;
    }

    public SubsystemCommands(
        Swerve swerve,
        IntakePivot intakePivot,
        IntakeRollers intakeRollers,
        Floor floor,
        Feeder feeder,
        Shooter shooter,
        Hood hood,
        Hanger hanger
    ) {
        this(
            swerve,
            intakePivot,
            intakeRollers,
            floor,
            feeder,
            shooter,
            hood,
            hanger,
            () -> 0,
            () -> 0
        );
    }

    public Command aimAndShoot() {
        final AimAndDriveCommand aimAndDriveCommand = new AimAndDriveCommand(swerve, forwardInput, leftInput);
        final PrepareShotCommand prepareShotCommand = new PrepareShotCommand(shooter, hood, () -> swerve.getPose());
        return Commands.parallel(
            aimAndDriveCommand,
            Commands.waitSeconds(0.25)
                .andThen(prepareShotCommand),
            Commands.waitUntil(() -> aimAndDriveCommand.isAimed() && prepareShotCommand.isReadyToShoot())
                .andThen(feed())
        );
    }

    public Command shootManually() {
        return shooter.dashboardSpinUpCommand()
            .andThen(feed())
            .handleInterrupt(() -> shooter.stop());
    }

    /** Swing the intake out and run the rollers; the rollers stop when the command ends. */
    public Command intake() {
        return Commands.startEnd(
            () -> {
                intakePivot.set(IntakePivot.Position.INTAKE);
                intakeRollers.set(IntakeRollers.Speed.INTAKE);
            },
            () -> intakeRollers.set(IntakeRollers.Speed.STOP),
            intakePivot,
            intakeRollers
        );
    }

    /** Run the rollers while rocking the intake to push fuel toward the floor rollers. */
    public Command agitate() {
        return intakeRollers.runOnce(() -> intakeRollers.set(IntakeRollers.Speed.INTAKE))
            .andThen(
                Commands.sequence(
                    intakePivot.runOnce(() -> intakePivot.set(IntakePivot.Position.AGITATE)),
                    Commands.waitUntil(intakePivot::isPositionWithinTolerance),
                    intakePivot.runOnce(() -> intakePivot.set(IntakePivot.Position.INTAKE)),
                    Commands.waitUntil(intakePivot::isPositionWithinTolerance)
                )
                .repeatedly()
            )
            .handleInterrupt(() -> {
                intakePivot.set(IntakePivot.Position.INTAKE);
                intakeRollers.set(IntakeRollers.Speed.STOP);
            });
    }

    private Command feed() {
        return Commands.sequence(
            Commands.waitSeconds(0.25),
            Commands.parallel(
                feeder.feedCommand(),
                Commands.waitSeconds(0.125)
                    .andThen(floor.feedCommand().alongWith(agitate()))
            )
        );
    }
}
