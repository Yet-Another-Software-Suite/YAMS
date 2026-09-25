// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later
// Ported from the WCP 2026 Competitive Concept (MIT, see LICENSE-WCP).

package first.robot.commands;

import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Seconds;

import first.robot.Landmarks;
import org.wpilib.command3.Command;
import org.wpilib.command3.Coroutine;
import org.wpilib.units.measure.Angle;
import first.robot.mechanisms.Feeder;
import first.robot.mechanisms.Floor;
import first.robot.mechanisms.Hanger;
import first.robot.mechanisms.Hood;
import first.robot.mechanisms.IntakePivot;
import first.robot.mechanisms.IntakeRollers;
import first.robot.mechanisms.Shooter;
import first.robot.mechanisms.Swerve;

/**
 * Commands that use several mechanisms. Each is one coroutine that requires every mechanism it
 * drives, like the v2 port's command groups, so starting one interrupts whatever else is using those
 * mechanisms. Feeding is plain sequential code shared by both shooting commands.
 */
public final class MechanismCommands {
    private static final Angle kAimTolerance = Degrees.of(5);

    private final Swerve swerve;
    private final IntakePivot intakePivot;
    private final IntakeRollers intakeRollers;
    private final Floor floor;
    private final Feeder feeder;
    private final Shooter shooter;
    private final Hood hood;
    private final Hanger hanger;

    // Set once feeding starts rocking the intake, so canceling before then leaves the pivot alone.
    private boolean isAgitating = false;

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
        this.swerve = swerve;
        this.intakePivot = intakePivot;
        this.intakeRollers = intakeRollers;
        this.floor = floor;
        this.feeder = feeder;
        this.shooter = shooter;
        this.hood = hood;
        this.hanger = hanger;

    }

    /**
     * Start tracking the shot after 0.25 s, and feed once the robot is facing the hub and the shot is
     * ready. Runs until canceled. The drive command running alongside it does the aiming:
     * {@link Drive#teleop} aims while the right trigger is held, and the autonomous routine runs
     * {@link Drive#autoAim}.
     */
    public Command shootWhenAimed() {
        final PrepareShot prepareShot = new PrepareShot(shooter, hood, () -> swerve.getPose());
        return Command.requiring(shooter, hood, feeder, floor, intakePivot, intakeRollers)
            .executing(coroutine -> {
                coroutine.wait(Seconds.of(0.25));
                coroutine.fork(prepareShot.command());
                // The shot cannot be ready before PrepareShot has set a velocity, so waiting from here
                // matches the v2 port's waitUntil that started at the same time as the aim.
                coroutine.waitUntil(() -> swerve.isFacing(Landmarks.hubPosition(), kAimTolerance) && prepareShot.isReadyToShoot());
                feed(coroutine);
            })
            .whenCanceled(this::stopFeeding)
            .named("Shoot When Aimed");
    }

    /** Spin up to the dashboard RPM, then feed. The shooter stops when the command is canceled. */
    public Command shootManually() {
        return Command.requiring(shooter, feeder, floor, intakePivot, intakeRollers)
            .executing(coroutine -> {
                shooter.setRPM(shooter.getDashboardTargetRPM());
                coroutine.waitUntil(shooter::isVelocityWithinTolerance);
                feed(coroutine);
            })
            .whenCanceled(() -> {
                stopFeeding();
                shooter.stop();
            })
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

    /**
     * Feed fuel into the shooter until canceled: start the feeder after 0.25 s, then 0.125 s later run
     * the floor rollers and intake rollers while rocking the intake to push fuel toward the floor.
     */
    private void feed(Coroutine coroutine) {
        coroutine.wait(Seconds.of(0.25));
        feeder.set(Feeder.Speed.FEED);
        coroutine.wait(Seconds.of(0.125));
        floor.set(Floor.Speed.FEED);
        intakeRollers.set(IntakeRollers.Speed.INTAKE);
        isAgitating = true;
        while (true) {
            intakePivot.set(IntakePivot.Position.AGITATE);
            coroutine.waitUntil(intakePivot::isPositionWithinTolerance);
            intakePivot.set(IntakePivot.Position.INTAKE);
            coroutine.waitUntil(intakePivot::isPositionWithinTolerance);
        }
    }

    /** Stop the rollers, and return the intake to its intake position if it was being rocked. */
    private void stopFeeding() {
        feeder.setPercentOutput(0);
        floor.set(Floor.Speed.STOP);
        intakeRollers.set(IntakeRollers.Speed.STOP);
        if (isAgitating) {
            intakePivot.set(IntakePivot.Position.INTAKE);
            isAgitating = false;
        }
    }
}
