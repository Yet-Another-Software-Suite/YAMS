// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later
// Ported from BroncBotz 3481 FRC2026.

package first.robot.opmodes.auto;

import static org.wpilib.units.Units.Seconds;

import first.robot.Constants.ShooterConstants;
import first.robot.Field;
import first.robot.Robot;
import org.wpilib.command3.Command;
import org.wpilib.command3.Scheduler;
import org.wpilib.command3.button.RobotModeTriggers;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.opmode.Autonomous;
import org.wpilib.opmode.OpMode;
import org.wpilib.units.measure.Time;

/**
 * Auto Three Advanced on the left side, with its own second trip.
 *
 * <p>Converted from the PathPlanner auto: the robot drives straight between each path's anchor points with drive to
 * pose, passing through intermediate waypoints and stopping at each path's end. Event markers start at the anchor point
 * before them.
 */
@Autonomous(name = "Auto Three Advanced Mirror")
public class AutoThreeAdvancedMirrorAuto implements OpMode {
    private final Robot robot;
    private final boolean mirror;

    /**
     * Creates the autonomous opmode. The OpModeRobot framework calls this when the opmode is selected on the driver
     * station.
     *
     * @param robot The robot instance to control.
     */
    public AutoThreeAdvancedMirrorAuto(Robot robot) {
        this(robot, false);
    }

    /**
     * Creates the autonomous opmode, optionally mirrored. {@link Robot} adds the mirrored copy.
     *
     * @param robot  The robot instance to control.
     * @param mirror Mirror the auto across the field's long centerline, to run it on the other side.
     */
    public AutoThreeAdvancedMirrorAuto(Robot robot, boolean mirror) {
        this.robot = robot;
        this.mirror = mirror;
        final Command routine = Command.noRequirements(coroutine -> {
            coroutine.await(robot.swerve.resetOdometryCommand(waypoint(3.56, 7.406, 0)));
            coroutine.await(robot.intakeArm.wiggleDown());
            // Auto Three Advanced Path 1 Mirror
            // IntakeStart event marker. Forked, so it keeps running across paths.
            Command intake = robot.superstructure.startIntake();
            coroutine.fork(intake);
            coroutine.await(robot.swerve.driveThroughPose(waypoint(5.494, 7.406, 0)));
            coroutine.await(robot.swerve.driveThroughPose(waypoint(7.429, 7.04, -90)));
            coroutine.await(robot.swerve.driveThroughPose(waypoint(7.341, 6.319, -90)));
            // IntakeStop event marker: takes the agitator from IntakeStart, which stops the rollers.
            intake = robot.superstructure.stopIntake();
            coroutine.fork(intake);
            coroutine.await(robot.swerve.driveThroughPose(waypoint(6.642, 6.811, -90)));
            coroutine.await(robot.swerve.driveThroughPose(waypoint(5.757, 7.401, 0)));
            coroutine.await(robot.swerve.driveThroughPose(waypoint(3.549, 7.406, 0)));
            coroutine.await(robot.swerve.driveToPose(waypoint(3, 7, -54.69)));
            // End the intake marker so the shot can take the agitator: the shot runs inside another command,
            // and taking the agitator from a command forked on this routine would cancel the whole routine.
            Scheduler.getDefault().cancel(intake);
            coroutine.await(shootWhileAiming(4, Seconds.of(0)));
            // Auto Three Advanced Path 2 Mirror
            coroutine.await(robot.swerve.driveThroughPose(waypoint(3.5, 7.406, 0)));
            // IntakeStart event marker. Forked, so it keeps running across paths.
            intake = robot.superstructure.startIntake();
            coroutine.fork(intake);
            coroutine.await(robot.swerve.driveThroughPose(waypoint(5.494, 7.406, 0)));
            coroutine.await(robot.swerve.driveThroughPose(waypoint(6.412, 6.876, -90)));
            coroutine.await(robot.swerve.driveThroughPose(waypoint(6.915, 5.423, -90)));
            // IntakeStop event marker: takes the agitator from IntakeStart, which stops the rollers.
            intake = robot.superstructure.stopIntake();
            coroutine.fork(intake);
            coroutine.await(robot.swerve.driveThroughPose(waypoint(7.254, 6.985, -90)));
            coroutine.await(robot.swerve.driveThroughPose(waypoint(5.6, 7.406, 0)));
            coroutine.await(robot.swerve.driveThroughPose(waypoint(3.549, 7.406, 0)));
            coroutine.await(robot.swerve.driveToPose(waypoint(3.3, 7.306, -54.69)));
            // End the intake marker so the shot can take the agitator: the shot runs inside another command,
            // and taking the agitator from a command forked on this routine would cancel the whole routine.
            Scheduler.getDefault().cancel(intake);
            coroutine.await(shootWhileAiming(4, Seconds.of(0)));
        }).named(mirror ? "Auto Three Advanced Mirror (Mirrored)" : "Auto Three Advanced Mirror");

        // Created in the opmode, so this binding only exists while the opmode is selected. The routine starts when
        // autonomous is enabled and is canceled when it is disabled.
        RobotModeTriggers.autonomous().whileTrue(routine);
    }

    /**
     * Shoot while turning to face the hub, rocking the intake arm to push fuel toward the indexer. Ends with the shot.
     *
     * @param wiggles How many times to raise and lower the intake arm.
     * @param delay   Wait before the first wiggle.
     */
    private Command shootWhileAiming(int wiggles, Time delay) {
        return Command.noRequirements(coroutine -> {
            // Aiming and wiggling are forked, so they end with the shot.
            coroutine.fork(robot.swerve.aimAtHubInPlace());
            coroutine.fork(robot.intakeArm.wiggle(wiggles, delay));
            coroutine.await(shot());
        }).named("Shoot While Aiming");
    }

    /** The "ShootCommand" named command: shoot at the hub from the robot's distance, for at most the auto shot timeout. */
    private Command shot() {
        return robot.superstructure.shootFromDistance().withTimeout(ShooterConstants.kAutoShotTimeout);
    }

    /**
     * A waypoint from the PathPlanner path, blue alliance origin, mirrored across the field's long centerline when this
     * auto is mirrored. The drive commands flip it for the red alliance.
     */
    private Pose2d waypoint(double x, double y, double headingDegrees) {
        Pose2d pose = new Pose2d(x, y, Rotation2d.fromDegrees(headingDegrees));
        return mirror ? Field.mirror(pose) : pose;
    }
}
