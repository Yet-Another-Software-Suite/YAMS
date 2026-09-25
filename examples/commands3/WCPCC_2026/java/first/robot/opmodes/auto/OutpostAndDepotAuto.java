// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later
// Ported from the WCP 2026 Competitive Concept (MIT, see LICENSE-WCP).

package first.robot.opmodes.auto;

import static first.robot.generated.ChoreoTraj.OutpostAndDepotTrajectory$0;
import static first.robot.generated.ChoreoTraj.OutpostAndDepotTrajectory$1;
import static first.robot.generated.ChoreoTraj.OutpostAndDepotTrajectory$2;
import static first.robot.generated.ChoreoTraj.OutpostAndDepotTrajectory$3;
import static org.wpilib.units.Units.Seconds;

import choreo.Choreo;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import first.robot.Robot;
import first.robot.generated.ChoreoTraj;
import first.robot.mechanisms.Hanger;
import first.robot.mechanisms.IntakePivot;
import java.util.List;
import java.util.Optional;
import org.wpilib.command3.Command;
import org.wpilib.command3.Coroutine;
import org.wpilib.command3.Scheduler;
import org.wpilib.driverstation.DriverStationErrors;
import org.wpilib.opmode.Autonomous;
import org.wpilib.opmode.OpMode;

/**
 * WCP's "Outpost and Depot" routine. ChoreoLib's AutoFactory, AutoRoutine and AutoTrajectory are
 * built on commands v2, so the trajectories are loaded with {@link Choreo#loadTrajectory} and
 * followed with {@code Swerve.followTrajectory}, and the v2 port's trajectory triggers ({@code
 * done()}, {@code atTime()}, {@code active()}) become waits in one coroutine. Timings are unchanged.
 */
@Autonomous(name = "Outpost and Depot")
public class OutpostAndDepotAuto implements OpMode {
    private final Robot robot;
    private final Scheduler scheduler = Scheduler.getDefault();

    private final Trajectory<SwerveSample> startToOutpost;
    private final Trajectory<SwerveSample> outpostToDepot;
    private final Trajectory<SwerveSample> depotToShootingPose;
    private final Trajectory<SwerveSample> shootingPoseToTower;

    private final Command routine;

    /**
     * Creates the autonomous opmode and loads its trajectories. The OpModeRobot framework calls this
     * when the opmode is selected on the driver station, so loading happens while disabled.
     *
     * @param robot The robot instance to control.
     */
    public OutpostAndDepotAuto(Robot robot) {
        this.robot = robot;
        final Optional<Trajectory<SwerveSample>> fullTrajectory = Choreo.loadTrajectory(ChoreoTraj.OutpostAndDepotTrajectory.name());
        startToOutpost = segment(fullTrajectory, OutpostAndDepotTrajectory$0);
        outpostToDepot = segment(fullTrajectory, OutpostAndDepotTrajectory$1);
        depotToShootingPose = segment(fullTrajectory, OutpostAndDepotTrajectory$2);
        shootingPoseToTower = segment(fullTrajectory, OutpostAndDepotTrajectory$3);
        routine = outpostAndDepotRoutine();
    }

    /** Homing and the routine start together when autonomous is enabled. */
    @Override
    public void start() {
        robot.scheduleHoming();
        scheduler.schedule(routine);
    }

    /** Stop the routine and homing when autonomous is disabled, as the v2 port did. */
    @Override
    public void end() {
        scheduler.cancel(routine);
        robot.cancelHoming();
    }

    /**
     * Picks one split of a Choreo trajectory. A missing trajectory becomes an empty one that is
     * skipped, as ChoreoLib's AutoRoutine did.
     */
    private static Trajectory<SwerveSample> segment(Optional<Trajectory<SwerveSample>> fullTrajectory, ChoreoTraj traj) {
        final Optional<Trajectory<SwerveSample>> split = fullTrajectory.flatMap(full -> full.getSplit(traj.segment().orElse(0)));
        if (split.isEmpty()) {
            DriverStationErrors.reportError("Could not load Choreo trajectory " + traj.name() + " segment " + traj.segment(), false);
            return new Trajectory<SwerveSample>(traj.name(), List.<SwerveSample>of(), List.of(), List.of());
        }
        return split.get();
    }

    private Command outpostAndDepotRoutine() {
        final Command startToOutpostCmd = robot.swerve.followTrajectory(startToOutpost);
        final Command outpostToDepotCmd = robot.swerve.followTrajectory(outpostToDepot);
        final Command depotToShootingPoseCmd = robot.swerve.followTrajectory(depotToShootingPose);
        final Command shootingPoseToTowerCmd = robot.swerve.followTrajectory(shootingPoseToTower);

        // Deploy the intake half a second after the hanger has homed.
        final Command deployIntake = Command.noRequirements(coroutine -> {
            coroutine.waitUntil(robot.hanger::isHomed);
            coroutine.wait(Seconds.of(0.5));
            coroutine.await(robot.intakePivot.positionCommand(IntakePivot.Position.INTAKE));
        }).named("Deploy Intake Once Homed");

        final Command intake = robot.autoCommands.intake();
        final Command spinUp = robot.shooter.spinUpCommand(2600);
        final Command hoodUp = robot.hood.positionCommand(0.32);
        final Command aimAndShoot = robot.autoCommands.aimAndShoot().withTimeout(Seconds.of(5));
        final Command extendHanger = robot.hanger.positionCommand(Hanger.Position.HANGING);
        final Command hang = robot.hanger.positionCommand(Hanger.Position.HUNG);

        return Command.noRequirements(coroutine -> {
            // Like the v2 trigger bindings, a command that cannot start (e.g. while homing holds
            // the pivot) is skipped instead of ending the whole routine.
            coroutine.setCancelOnForkFailure(false);
            coroutine.fork(deployIntake);

            coroutine.await(robot.swerve.resetOdometryCommand(startToOutpost));
            coroutine.await(startToOutpostCmd);
            coroutine.wait(Seconds.of(1));

            // Start intaking one second before reaching the depot.
            coroutine.fork(outpostToDepotCmd);
            coroutine.wait(Seconds.of(outpostToDepot.getTotalTime() - 1));
            coroutine.fork(intake);
            awaitFinished(coroutine, outpostToDepotCmd);
            coroutine.wait(Seconds.of(0.1));

            // Vision is paused on the way to the shooting pose; spin up half a second in.
            final Command pauseVisionToShootingPose = robot.limelight.idle();
            coroutine.fork(depotToShootingPoseCmd, pauseVisionToShootingPose);
            coroutine.wait(Seconds.of(0.5));
            coroutine.fork(spinUp, hoodUp);
            awaitFinished(coroutine, depotToShootingPoseCmd);
            scheduler.cancel(pauseVisionToShootingPose);

            coroutine.await(aimAndShoot);

            // Vision is paused on the way to the tower, and the hanger extends while driving there.
            final Command pauseVisionToTower = robot.limelight.idle();
            coroutine.fork(shootingPoseToTowerCmd, pauseVisionToTower, extendHanger);
            awaitFinished(coroutine, shootingPoseToTowerCmd);
            scheduler.cancel(pauseVisionToTower);
            coroutine.fork(hang);

            // Stay alive so the forked commands are not canceled before autonomous ends.
            coroutine.park();
        }).named("Outpost and Depot");
    }

    /** Yield until a forked command has finished. */
    private void awaitFinished(Coroutine coroutine, Command command) {
        coroutine.waitUntil(() -> !scheduler.isScheduledOrRunning(command));
    }
}
