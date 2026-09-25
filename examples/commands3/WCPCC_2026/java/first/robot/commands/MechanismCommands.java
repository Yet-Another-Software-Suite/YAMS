// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later
// Ported from the WCP 2026 Competitive Concept (MIT, see LICENSE-WCP).

package first.robot.commands;

import static org.wpilib.units.Units.Seconds;

import java.util.function.DoubleSupplier;

import org.wpilib.command3.Command;
import first.robot.mechanisms.Feeder;
import first.robot.mechanisms.Floor;
import first.robot.mechanisms.Hanger;
import first.robot.mechanisms.Hood;
import first.robot.mechanisms.IntakePivot;
import first.robot.mechanisms.IntakeRollers;
import first.robot.mechanisms.Shooter;
import first.robot.mechanisms.Swerve;

/**
 * Commands that use several mechanisms. Each is a coroutine that requires every mechanism its
 * children use, like the v2 port's command groups, so starting one interrupts whatever else is using
 * those mechanisms.
 */
public final class MechanismCommands {
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

    public MechanismCommands(
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

    public MechanismCommands(
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

    /**
     * Aim at the hub while driving, start tracking the shot after 0.25 s, and feed once the robot is
     * aimed and the shot is ready. Runs until canceled.
     */
    public Command aimAndShoot() {
        final AimAndDrive aimAndDrive = new AimAndDrive(swerve, forwardInput, leftInput);
        final PrepareShot prepareShot = new PrepareShot(shooter, hood, () -> swerve.getPose());
        final Command feed = feed();
        return Command.requiring(swerve, shooter, hood, feeder, floor, intakePivot, intakeRollers)
            .executing(coroutine -> {
                coroutine.fork(aimAndDrive.command());
                coroutine.wait(Seconds.of(0.25));
                coroutine.fork(prepareShot.command());
                // The shot cannot be ready before PrepareShot has set a velocity, so waiting from here
                // matches the v2 port's waitUntil that started at the same time as the aim.
                coroutine.waitUntil(() -> aimAndDrive.isAimed() && prepareShot.isReadyToShoot());
                coroutine.await(feed);
            })
            .named("Aim And Shoot");
    }

    /** Spin up to the dashboard RPM, then feed. The shooter stops when the command is canceled. */
    public Command shootManually() {
        final Command spinUp = shooter.dashboardSpinUpCommand();
        final Command feed = feed();
        return Command.requiring(shooter, feeder, floor, intakePivot, intakeRollers)
            .executing(coroutine -> {
                coroutine.await(spinUp);
                coroutine.await(feed);
            })
            .whenCanceled(shooter::stop)
            .named("Shoot Manually");
    }

    /** Swing the intake out and run the rollers; the rollers stop when the command is canceled. */
    public Command intake() {
        return Command.requiring(intakePivot, intakeRollers)
            .executing(coroutine -> {
                intakePivot.set(IntakePivot.Position.INTAKE);
                intakeRollers.set(IntakeRollers.Speed.INTAKE);
                coroutine.park();
            })
            .whenCanceled(() -> intakeRollers.set(IntakeRollers.Speed.STOP))
            .named("Intake");
    }

    /** Run the rollers while rocking the intake to push fuel toward the floor rollers. */
    public Command agitate() {
        return Command.requiring(intakePivot, intakeRollers)
            .executing(coroutine -> {
                intakeRollers.set(IntakeRollers.Speed.INTAKE);
                while (true) {
                    intakePivot.set(IntakePivot.Position.AGITATE);
                    coroutine.waitUntil(intakePivot::isPositionWithinTolerance);
                    intakePivot.set(IntakePivot.Position.INTAKE);
                    coroutine.waitUntil(intakePivot::isPositionWithinTolerance);
                }
            })
            .whenCanceled(() -> {
                intakePivot.set(IntakePivot.Position.INTAKE);
                intakeRollers.set(IntakeRollers.Speed.STOP);
            })
            .named("Agitate");
    }

    private Command feed() {
        final Command feederFeed = feeder.feedCommand();
        final Command floorFeed = floor.feedCommand();
        final Command agitate = agitate();
        return Command.requiring(feeder, floor, intakePivot, intakeRollers)
            .executing(coroutine -> {
                coroutine.wait(Seconds.of(0.25));
                coroutine.fork(feederFeed);
                coroutine.wait(Seconds.of(0.125));
                coroutine.awaitAll(floorFeed, agitate);
            })
            .named("Feed");
    }
}
