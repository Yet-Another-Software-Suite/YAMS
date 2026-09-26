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
import first.robot.commands.Drive;
import first.robot.generated.ChoreoTraj;
import first.robot.mechanisms.Hanger;
import first.robot.mechanisms.IntakePivot;
import java.util.List;
import java.util.Optional;
import org.wpilib.command3.Command;
import org.wpilib.command3.Scheduler;
import org.wpilib.command3.Trigger;
import org.wpilib.command3.button.RobotModeTriggers;
import org.wpilib.driverstation.DriverStationErrors;
import org.wpilib.opmode.Autonomous;
import org.wpilib.opmode.OpMode;

/**
 * WCP's "Outpost and Depot" routine. ChoreoLib's AutoFactory, AutoRoutine and AutoTrajectory are
 * built on commands v2, so the trajectories are loaded with {@link Choreo#loadTrajectory} and
 * followed with {@code Swerve.followTrajectory}. The routine coroutine awaits the trajectories in
 * order, replacing the v2 port's {@code done()} chaining, and the v2 port's {@code active()} and
 * {@code atTime()} events become triggers bound inside the routine, so they only exist while it
 * runs. Timings are unchanged.
 */
@Autonomous(name = "Outpost and Depot")
public class OutpostAndDepotAuto implements OpMode {
    private final Robot robot;
    private final Scheduler scheduler = Scheduler.getDefault();

    private final Trajectory<SwerveSample> startToOutpost;
    private final Trajectory<SwerveSample> outpostToDepot;
    private final Trajectory<SwerveSample> depotToShootingPose;
    private final Trajectory<SwerveSample> shootingPoseToTower;

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

        // Created in the opmode, so this binding only exists while the opmode is selected. The
        // routine starts when autonomous is enabled and is canceled when it is disabled. Homing is
        // bound in Robot.
        RobotModeTriggers.autonomous().whileTrue(outpostAndDepotRoutine());
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
            coroutine.wait(Seconds.of(0.5));
            coroutine.await(robot.intakePivot.moveTo(IntakePivot.Position.INTAKE));
        }).named("Deploy Intake");

        final Command intake = robot.mechanismCommands.intake();
        final Command spinUp = robot.shooter.spinUp(2600);
        final Command hoodUp = robot.hood.moveTo(0.32);
        final Command pauseVisionToShootingPose = robot.limelight.idle();
        final Command shootWhenAimed = robot.mechanismCommands.shootWhenAimed();
        final Command aim = Drive.autoAim(robot.swerve);
        final Command shotTimeout = Command.waitFor(Seconds.of(5)).named("Shot Timeout");
        final Command pauseVisionToTower = robot.limelight.idle();
        final Command extendHanger = robot.hanger.moveTo(Hanger.Position.HANGING);
        final Command hang = robot.hanger.moveTo(Hanger.Position.HUNG);

        return Command.noRequirements(coroutine -> {
            // Like the v2 trigger bindings, a command that cannot start (e.g. the hanger while it is
            // still homing) is skipped instead of ending the whole routine.
            coroutine.setCancelOnForkFailure(false);

            // Events along the way, like the v2 port's routine triggers. They are bound inside this
            // command, so they stop firing and their commands are canceled when the routine ends.
            // Commands they start are not children of the routine, so a later step that needs the
            // same mechanism (feeding takes the intake, the shot takes the shooter and hood) just
            // interrupts them.
            new Trigger(robot.hanger::isHomed).onTrue(deployIntake);
            // Start intaking one second before reaching the depot.
            following(outpostToDepotCmd)
                .debounce(Seconds.of(Math.max(outpostToDepot.getTotalTime() - 1, 0)))
                .onTrue(intake);
            // Vision is paused on the way to the shooting pose; spin up half a second in.
            final Trigger toShootingPose = following(depotToShootingPoseCmd);
            toShootingPose.whileTrue(pauseVisionToShootingPose);
            toShootingPose.debounce(Seconds.of(0.5))
                .onTrue(spinUp)
                .onTrue(hoodUp);
            // Vision is paused on the way to the tower, and the hanger extends while driving there.
            following(shootingPoseToTowerCmd)
                .whileTrue(pauseVisionToTower)
                .onTrue(extendHanger);

            robot.swerve.resetOdometry(startToOutpost);
            coroutine.await(startToOutpostCmd);
            coroutine.wait(Seconds.of(1));
            coroutine.await(outpostToDepotCmd);
            coroutine.wait(Seconds.of(0.1));
            coroutine.await(depotToShootingPoseCmd);

            // Aim and shoot for five seconds; the timeout ending cancels the other two.
            coroutine.awaitAny(aim, shootWhenAimed, shotTimeout);

            coroutine.await(shootingPoseToTowerCmd);
            coroutine.await(hang);
        }).named("Outpost and Depot");
    }

    /** A trigger that is true while a trajectory command is running, like {@code active()} in v2. */
    private Trigger following(Command trajectoryCommand) {
        return new Trigger(() -> scheduler.isRunning(trajectoryCommand));
    }
}
