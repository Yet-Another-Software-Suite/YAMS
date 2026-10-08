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
 * Collect fuel from the center line twice, shooting after each trip.
 *
 * <p>Converted from the PathPlanner auto: the robot drives straight between each path's anchor points with drive to
 * pose, passing through intermediate waypoints and stopping at each path's end. Event markers start at the anchor point
 * before them.
 */
@Autonomous(name = "Auto One Advanced")
public class AutoOneAdvancedAuto implements OpMode {
    private final Robot robot;
    private final boolean mirror;

    /**
     * Creates the autonomous opmode. The OpModeRobot framework calls this when the opmode is selected on the driver
     * station.
     *
     * @param robot The robot instance to control.
     */
    public AutoOneAdvancedAuto(Robot robot) {
        this(robot, false);
    }

    /**
     * Creates the autonomous opmode, optionally mirrored. {@link Robot} adds the mirrored copy.
     *
     * @param robot  The robot instance to control.
     * @param mirror Mirror the auto across the field's long centerline, to run it on the other side.
     */
    public AutoOneAdvancedAuto(Robot robot, boolean mirror) {
        this.robot = robot;
        this.mirror = mirror;
        final Command routine = Command.noRequirements(coroutine -> {
            coroutine.await(robot.swerve.resetOdometryCommand(waypoint(3.549, 7.425, 0)));
            coroutine.await(robot.intakeArm.wiggleDown());
            // Auto One Advanced Path 1
            // IntakeStart event marker. Forked, so it keeps running across paths.
            Command intake = robot.superstructure.startIntake();
            coroutine.fork(intake);
            coroutine.await(robot.swerve.driveThroughPose(waypoint(5.494, 7.425, 0)));
            coroutine.await(robot.swerve.driveThroughPose(waypoint(7.33, 7.226, -90)));
            coroutine.await(robot.swerve.driveThroughPose(waypoint(7.724, 5.193, -90)));
            // IntakeStop event marker: takes the agitator from IntakeStart, which stops the rollers.
            intake = robot.superstructure.stopIntake();
            coroutine.fork(intake);
            coroutine.await(robot.swerve.driveThroughPose(waypoint(6.849, 6.078, -90)));
            coroutine.await(robot.swerve.driveThroughPose(waypoint(5.243, 7.357, 0)));
            coroutine.await(robot.swerve.driveThroughPose(waypoint(3.538, 7.423, 0)));
            coroutine.await(robot.swerve.driveToPose(waypoint(2.5, 6.308, -47.726)));
            // End the intake marker so the shot can take the agitator: the shot runs inside another command,
            // and taking the agitator from a command forked on this routine would cancel the whole routine.
            Scheduler.getDefault().cancel(intake);
            coroutine.await(shootWhileAiming(3, Seconds.of(0)));
            // Auto One Advanced Path Follow
            coroutine.await(robot.swerve.driveToPose(waypoint(3.565, 7.434, 0)));
            // Auto One Advanced Path 2
            // IntakeStart event marker. Forked, so it keeps running across paths.
            intake = robot.superstructure.startIntake();
            coroutine.fork(intake);
            coroutine.await(robot.swerve.driveThroughPose(waypoint(6.583, 7.045, -90)));
            coroutine.await(robot.swerve.driveThroughPose(waypoint(6.777, 4.887, -90)));
            // IntakeStop event marker: takes the agitator from IntakeStart, which stops the rollers.
            intake = robot.superstructure.stopIntake();
            coroutine.fork(intake);
            coroutine.await(robot.swerve.driveThroughPose(waypoint(6.777, 7.223, 0)));
            coroutine.await(robot.swerve.driveThroughPose(waypoint(3.56, 7.426, 0)));
            coroutine.await(robot.swerve.driveToPose(waypoint(2.429, 6.363, -47.291)));
            // End the intake marker so the shot can take the agitator: the shot runs inside another command,
            // and taking the agitator from a command forked on this routine would cancel the whole routine.
            Scheduler.getDefault().cancel(intake);
            coroutine.await(shootWhileAiming(3, Seconds.of(0)));
        }).named(mirror ? "Auto One Advanced (Mirrored)" : "Auto One Advanced");

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
