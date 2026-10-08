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
 * Collect fuel from the center line on the right side and shoot.
 *
 * <p>Converted from the PathPlanner auto: the robot drives straight between each path's anchor points with drive to
 * pose, passing through intermediate waypoints and stopping at each path's end. Event markers start at the anchor point
 * before them.
 */
@Autonomous(name = "Auto Three")
public class AutoThreeAuto implements OpMode {
    private final Robot robot;
    private final boolean mirror;

    /**
     * Creates the autonomous opmode. The OpModeRobot framework calls this when the opmode is selected on the driver
     * station.
     *
     * @param robot The robot instance to control.
     */
    public AutoThreeAuto(Robot robot) {
        this(robot, false);
    }

    /**
     * Creates the autonomous opmode, optionally mirrored. {@link Robot} adds the mirrored copy.
     *
     * @param robot  The robot instance to control.
     * @param mirror Mirror the auto across the field's long centerline, to run it on the other side.
     */
    public AutoThreeAuto(Robot robot, boolean mirror) {
        this.robot = robot;
        this.mirror = mirror;
        final Command routine = Command.noRequirements(coroutine -> {
            coroutine.await(robot.swerve.resetOdometryCommand(waypoint(3.56, 0.639, 0)));
            // Auto Three Path Three
            // IntakeStart event marker. Forked, so it keeps running across paths.
            Command intake = robot.superstructure.startIntake();
            coroutine.fork(intake);
            coroutine.await(robot.swerve.driveThroughPose(waypoint(6.666, 0.742, 0)));
            coroutine.await(robot.swerve.driveThroughPose(waypoint(7.235, 1.053, 90)));
            coroutine.await(robot.swerve.driveThroughPose(waypoint(7.649, 2.618, 90)));
            coroutine.await(robot.swerve.driveThroughPose(waypoint(7.52, 2.411, 90)));
            coroutine.await(robot.swerve.driveThroughPose(waypoint(5.968, 0.639, 0)));
            coroutine.await(robot.swerve.driveThroughPose(waypoint(3.56, 0.639, 0)));
            coroutine.await(robot.swerve.driveToPose(waypoint(2.228, 1.273, 47.816)));
            // End the intake marker so the shot can take the agitator: the shot runs inside another command,
            // and taking the agitator from a command forked on this routine would cancel the whole routine.
            Scheduler.getDefault().cancel(intake);
            coroutine.await(shootWhileAiming(0, Seconds.of(0)));
            coroutine.await(robot.intakeArm.wiggleUp());
        }).named(mirror ? "Auto Three (Mirrored)" : "Auto Three");

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
