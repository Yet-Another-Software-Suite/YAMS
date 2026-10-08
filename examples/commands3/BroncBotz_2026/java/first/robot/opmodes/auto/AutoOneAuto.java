// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later
// Ported from BroncBotz 3481 FRC2026.

package first.robot.opmodes.auto;

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

/**
 * Collect fuel from the center line and shoot.
 *
 * <p>Converted from the PathPlanner auto: the robot drives straight between each path's anchor points with drive to
 * pose, passing through intermediate waypoints and stopping at each path's end. Event markers start at the anchor point
 * before them.
 */
@Autonomous(name = "Auto One")
public class AutoOneAuto implements OpMode {
    private final Robot robot;
    private final boolean mirror;

    /**
     * Creates the autonomous opmode. The OpModeRobot framework calls this when the opmode is selected on the driver
     * station.
     *
     * @param robot The robot instance to control.
     */
    public AutoOneAuto(Robot robot) {
        this(robot, false);
    }

    /**
     * Creates the autonomous opmode, optionally mirrored. {@link Robot} adds the mirrored copy.
     *
     * @param robot  The robot instance to control.
     * @param mirror Mirror the auto across the field's long centerline, to run it on the other side.
     */
    public AutoOneAuto(Robot robot, boolean mirror) {
        this.robot = robot;
        this.mirror = mirror;
        final Command routine = Command.noRequirements(coroutine -> {
            coroutine.await(robot.swerve.resetOdometryCommand(waypoint(3.56, 7.375, 0)));
            // Auto One Path One
            coroutine.await(robot.swerve.driveThroughPose(waypoint(7.08, 7.095, -90)));
            // IntakeStart event marker. Forked, so it keeps running across paths.
            Command intake = robot.superstructure.startIntake();
            coroutine.fork(intake);
            coroutine.await(robot.swerve.driveThroughPose(waypoint(7.584, 4.857, -90)));
            coroutine.await(robot.swerve.driveThroughPose(waypoint(7.3, 7.211, 0)));
            coroutine.await(robot.swerve.driveThroughPose(waypoint(2.422, 7.16, 0)));
            coroutine.await(robot.swerve.driveToPose(waypoint(2.085, 6.254, -45)));
            // End the intake marker so the shot can take the agitator: the shot runs inside another command,
            // and taking the agitator from a command forked on this routine would cancel the whole routine.
            Scheduler.getDefault().cancel(intake);
            coroutine.await(shot());
        }).named(mirror ? "Auto One (Mirrored)" : "Auto One");

        // Created in the opmode, so this binding only exists while the opmode is selected. The routine starts when
        // autonomous is enabled and is canceled when it is disabled.
        RobotModeTriggers.autonomous().whileTrue(routine);
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
