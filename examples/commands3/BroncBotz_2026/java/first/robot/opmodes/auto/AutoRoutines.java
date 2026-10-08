// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later
// Ported from BroncBotz 3481 FRC2026.

package first.robot.opmodes.auto;

import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.Seconds;

import first.robot.Constants.ShooterConstants;
import first.robot.Field;
import first.robot.Robot;
import first.robot.opmodes.auto.AutoPaths.AutoPath;
import first.robot.opmodes.auto.AutoPaths.MarkerAt;
import java.util.List;
import org.wpilib.command3.Command;
import org.wpilib.command3.Coroutine;
import org.wpilib.command3.Scheduler;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.Distance;
import org.wpilib.units.measure.Time;

/**
 * The steps the autos are built from, standing in for the original PathPlanner autos and named commands. Each auto
 * opmode makes one of these and runs the steps in order from a single routine command's coroutine.
 *
 * <p>Poses are given for the blue alliance. When an auto starts they are mirrored across the field's long centerline
 * if the auto is mirrored, then flipped for the red alliance.
 */
final class AutoRoutines {
    /** Tolerances at a waypoint the robot drives through. */
    private static final Distance kWaypointTolerance = Meters.of(0.3);
    private static final Angle kWaypointHeadingTolerance = Degrees.of(15);
    /** Tolerances at the end of a path, where the robot stops. */
    private static final Distance kEndTolerance = Meters.of(0.05);
    private static final Angle kEndHeadingTolerance = Degrees.of(3);

    private final Robot robot;
    private final boolean mirror;
    /** The event marker command started last, which keeps running across paths until it is replaced or a shot. */
    private Command markerCommand;

    /**
     * Create the steps for an auto.
     *
     * @param robot  The robot.
     * @param mirror Mirror the auto across the field's long centerline, to run it on the other side.
     */
    AutoRoutines(Robot robot, boolean mirror) {
        this.robot = robot;
        this.mirror = mirror;
    }

    /** A blue pose mirrored when the auto is mirrored, still blue alliance origin. */
    private Pose2d mirrored(Pose2d bluePose) {
        return mirror ? Field.mirror(bluePose) : bluePose;
    }

    /**
     * Reset the robot's pose to the start of a path, as the PathPlanner autos did.
     *
     * @param path First path of the auto.
     * @return {@link Command} that resets the pose.
     */
    Command resetPose(AutoPath path) {
        return robot.swerve.resetOdometryCommand(mirrored(path.start()));
    }

    /**
     * Drive a path: drive to pose through each waypoint after the first, starting the path's event markers at their
     * waypoints. The marker commands are forked on the auto's routine, so they keep running after the path ends.
     *
     * @param coroutine The auto routine's coroutine.
     * @param path      Path to drive.
     */
    void followPath(Coroutine coroutine, AutoPath path) {
        final List<Pose2d> waypoints = path.waypoints();
        startMarkers(coroutine, path, 0);
        for (int i = 1; i < waypoints.size(); i++) {
            final boolean last = i == waypoints.size() - 1;
            coroutine.await(robot.swerve.driveToPose(
                Field.forAlliance(mirrored(waypoints.get(i))),
                last ? kEndTolerance : kWaypointTolerance,
                last ? kEndHeadingTolerance : kWaypointHeadingTolerance));
            startMarkers(coroutine, path, i);
        }
    }

    private void startMarkers(Coroutine coroutine, AutoPath path, int waypoint) {
        for (MarkerAt marker : path.markers()) {
            if (marker.waypoint() == waypoint) {
                markerCommand = switch (marker.marker()) {
                    case INTAKE_START -> robot.superstructure.startIntake();
                    case INTAKE_STOP -> robot.superstructure.stopIntake();
                };
                // Both use the agitator: a stop forked while a start runs replaces it, as they share this routine as parent.
                coroutine.fork(markerCommand);
            }
        }
    }

    /**
     * End the running event marker command, so a shot can take over the agitator. The shot runs inside another
     * command, so taking over a marker command forked on the routine would cancel the whole routine instead.
     */
    private void endMarkers() {
        if (markerCommand != null) {
            Scheduler.getDefault().cancel(markerCommand);
            markerCommand = null;
        }
    }

    /**
     * The "ShootCommand" named command: shoot at the hub from the robot's distance, for at most the auto shot timeout.
     *
     * @param coroutine The auto routine's coroutine.
     */
    void shoot(Coroutine coroutine) {
        endMarkers();
        coroutine.await(shot());
    }

    /**
     * Shoot while turning to face the hub, rocking the intake arm to push fuel toward the indexer. Ends with the shot.
     *
     * @param coroutine The auto routine's coroutine.
     * @param wiggles   How many times to raise and lower the intake arm.
     * @param delay     Wait before the first wiggle.
     */
    void shootWhileAiming(Coroutine coroutine, int wiggles, Time delay) {
        endMarkers();
        final Command aim = robot.swerve.aimAtHubInPlace();
        final Command wiggle = Command.noRequirements(wiggleCoroutine -> {
            wiggleCoroutine.wait(delay);
            for (int i = 0; i < wiggles; i++) {
                wiggleCoroutine.await(robot.intakeArm.wiggleUp());
                wiggleCoroutine.await(robot.intakeArm.wiggleDown());
            }
        }).named("Wiggle Intake Arm");
        final Command shot = shot();
        // Aiming and wiggling are forked, so they end with the shot.
        coroutine.await(Command.noRequirements(shotCoroutine -> {
            shotCoroutine.fork(aim);
            shotCoroutine.fork(wiggle);
            shotCoroutine.await(shot);
        }).named("Shoot While Aiming"));
    }

    /**
     * Shoot while aiming at the hub, rocking the intake arm right away.
     *
     * @param coroutine The auto routine's coroutine.
     * @param wiggles   How many times to raise and lower the intake arm.
     */
    void shootWhileAiming(Coroutine coroutine, int wiggles) {
        shootWhileAiming(coroutine, wiggles, Seconds.of(0));
    }

    /** The "ArmUp" named command. */
    Command armUp() {
        return robot.intakeArm.wiggleUp();
    }

    /** The "ArmDown" named command. */
    Command armDown() {
        return robot.intakeArm.wiggleDown();
    }

    private Command shot() {
        return robot.superstructure.shootFromDistance().withTimeout(ShooterConstants.kAutoShotTimeout);
    }
}
