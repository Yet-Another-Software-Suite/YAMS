// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later
// Ported from the WCP 2026 Competitive Concept (MIT, see LICENSE-WCP).

package first.robot.commands;

import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Inches;
import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.Seconds;

import first.robot.Landmarks;
import org.wpilib.command3.Command;
import org.wpilib.networktables.DoublePublisher;
import org.wpilib.networktables.NetworkTableInstance;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.Distance;
import first.robot.mechanisms.Feeder;
import first.robot.mechanisms.Floor;
import first.robot.mechanisms.Hanger;
import first.robot.mechanisms.Hood;
import first.robot.mechanisms.IntakePivot;
import first.robot.mechanisms.IntakeRollers;
import first.robot.mechanisms.Shooter;
import first.robot.mechanisms.Swerve;

/**
 * Commands that use several mechanisms. Each is a coroutine without requirements of its own that
 * runs the mechanisms' own commands with {@code fork}, {@code await} and {@code awaitAll}, so a
 * mechanism is only owned while its command runs, and its default command (stopping the rollers)
 * takes over again once that command ends. Canceling one of these commands cancels the mechanism
 * commands it started.
 */
public final class MechanismCommands {
    private static final Angle kAimTolerance = Degrees.of(5);
    private static final DoublePublisher distanceToHubPublisher = NetworkTableInstance.getDefault()
        .getDoubleTopic("SmartDashboard/Distance to Hub (inches)")
        .publish();

    private final Swerve swerve;
    private final IntakePivot intakePivot;
    private final IntakeRollers intakeRollers;
    private final Floor floor;
    private final Feeder feeder;
    private final Shooter shooter;
    private final Hood hood;
    private final Hanger hanger;

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
        final Command prepareShot = prepareShot();
        final Command feed = feed();
        return Command.noRequirements(coroutine -> {
            coroutine.wait(Seconds.of(0.25));
            coroutine.fork(prepareShot);
            // The shot cannot be ready before prepareShot has set a velocity, so waiting from here
            // matches the v2 port's waitUntil that started at the same time as the aim.
            coroutine.waitUntil(() -> swerve.isFacing(Landmarks.hubPosition(), kAimTolerance)
                && shooter.isVelocityWithinTolerance() && hood.isPositionWithinTolerance());
            coroutine.await(feed);
        }).named("Shoot When Aimed");
    }

    /**
     * Set the shooter speed and hood position from the distance to the hub every loop, using
     * {@link ShotMap}. The shooter stops when the command is canceled.
     */
    private Command prepareShot() {
        return Command.requiring(shooter, hood)
            .executing(coroutine -> {
                while (true) {
                    final Distance distanceToHub =
                        Meters.of(swerve.getPose().getTranslation().getDistance(Landmarks.hubPosition()));
                    final ShotMap.Shot shot = ShotMap.forDistance(distanceToHub);
                    shooter.setRPM(shot.shooterRPM);
                    hood.setPosition(shot.hoodPosition);
                    distanceToHubPublisher.set(distanceToHub.in(Inches));
                    coroutine.yield();
                }
            })
            .whenCanceled(shooter::stop)
            .named("Prepare Shot");
    }

    /**
     * Spin up to the dashboard RPM, then feed. The shooter is held at that speed until the command
     * is canceled, and then stops.
     */
    public Command shootManually() {
        final Command feed = feed();
        return Command.noRequirements(coroutine -> {
            coroutine.fork(shooter.runAt(shooter.getDashboardTargetRPM()));
            coroutine.waitUntil(shooter::isVelocityWithinTolerance);
            coroutine.await(feed);
        }).named("Shoot Manually");
    }

    /**
     * Swing the intake out and run the rollers until canceled. The pivot holds its intake position
     * afterwards, and the rollers stop.
     */
    public Command intake() {
        final Command pivotOut = intakePivot.holdAt(IntakePivot.Position.INTAKE);
        final Command rollersIn = intakeRollers.intake();
        return Command.noRequirements(coroutine -> {
            coroutine.awaitAll(pivotOut, rollersIn);
        }).named("Intake");
    }

    /**
     * Feed fuel into the shooter until canceled: start the feeder after 0.25 s, then 0.125 s later run
     * the floor rollers and intake rollers while rocking the intake to push fuel toward the floor.
     */
    private Command feed() {
        final Command feederIn = feeder.feed();
        final Command floorIn = floor.feed();
        final Command rollersIn = intakeRollers.intake();
        final Command agitate = intakePivot.agitate();
        return Command.noRequirements(coroutine -> {
            coroutine.wait(Seconds.of(0.25));
            coroutine.fork(feederIn);
            coroutine.wait(Seconds.of(0.125));
            // These run until feeding is canceled; forked commands end with their parent, so park.
            coroutine.fork(floorIn, rollersIn, agitate);
            coroutine.park();
        }).named("Feed");
    }

    /**
     * Home the intake pivot and the hanger together. Each homing command runs above the default
     * priority and does nothing once its mechanism is homed.
     */
    public Command home() {
        final Command homePivot = intakePivot.home();
        final Command homeHanger = hanger.home();
        return Command.noRequirements(coroutine -> {
            coroutine.awaitAll(homePivot, homeHanger);
        }).named("Home");
    }
}
