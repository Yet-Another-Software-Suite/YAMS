// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later
// Ported from BroncBotz 3481 FRC2026.

package first.robot.opmodes.auto;

import static org.wpilib.units.Units.Seconds;

import first.robot.Constants.ShooterConstants;
import first.robot.Field;
import first.robot.Robot;
import org.wpilib.command3.Command;
import org.wpilib.command3.button.RobotModeTriggers;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.opmode.Autonomous;
import org.wpilib.opmode.OpMode;
import org.wpilib.units.measure.Time;

/**
 * Drive back from the start line and shoot the preload while aiming.
 *
 * <p>Converted from the PathPlanner auto: the robot drives straight between each path's anchor points with drive to
 * pose, passing through intermediate waypoints and stopping at each path's end. Event markers start at the anchor point
 * before them.
 */
@Autonomous(name = "Auto 2 Preload")
public class Auto2PreloadAuto implements OpMode {
    private final Robot robot;
    private final boolean mirror;

    /**
     * Creates the autonomous opmode. The OpModeRobot framework calls this when the opmode is selected on the driver
     * station.
     *
     * @param robot The robot instance to control.
     */
    public Auto2PreloadAuto(Robot robot) {
        this(robot, false);
    }

    /**
     * Creates the autonomous opmode, optionally mirrored. {@link Robot} adds the mirrored copy.
     *
     * @param robot  The robot instance to control.
     * @param mirror Mirror the auto across the field's long centerline, to run it on the other side.
     */
    public Auto2PreloadAuto(Robot robot, boolean mirror) {
        this.robot = robot;
        this.mirror = mirror;
        final Command routine = Command.noRequirements(coroutine -> {
            coroutine.await(robot.swerve.resetOdometryCommand(waypoint(3.534, 4.054, 0)));
            // Auto 2 Preload Path
            coroutine.await(robot.swerve.driveToPose(waypoint(2.141, 4.054, 0)));
            coroutine.await(robot.intakeArm.wiggleDown());
            coroutine.await(shootWhileAiming(3, Seconds.of(0)));
        }).named(mirror ? "Auto 2 Preload (Mirrored)" : "Auto 2 Preload");

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
