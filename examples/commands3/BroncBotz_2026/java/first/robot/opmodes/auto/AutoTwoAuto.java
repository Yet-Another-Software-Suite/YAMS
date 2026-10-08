// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later
// Ported from BroncBotz 3481 FRC2026.

package first.robot.opmodes.auto;

import first.robot.Robot;
import org.wpilib.command3.Command;
import org.wpilib.command3.button.RobotModeTriggers;
import org.wpilib.opmode.Autonomous;
import org.wpilib.opmode.OpMode;

/** The "Auto Two" PathPlanner auto, driven with drive to pose. See {@link AutoPaths} for how the paths were converted. */
@Autonomous(name = "Auto Two")
public class AutoTwoAuto implements OpMode {
    /**
     * Creates the autonomous opmode. The OpModeRobot framework calls this when the opmode is selected on the driver
     * station.
     *
     * @param robot The robot instance to control.
     */
    public AutoTwoAuto(Robot robot) {
        this(robot, false);
    }

    /**
     * Creates the autonomous opmode, optionally mirrored. {@link Robot} adds the mirrored copy.
     *
     * @param robot  The robot instance to control.
     * @param mirror Mirror the auto across the field's long centerline, to run it on the other side.
     */
    public AutoTwoAuto(Robot robot, boolean mirror) {
        final AutoRoutines auto = new AutoRoutines(robot, mirror);
        final Command routine = Command.noRequirements(coroutine -> {
            coroutine.await(auto.resetPose(AutoPaths.AUTO_TWO_PATH_ONE));
            coroutine.await(auto.armDown());
            auto.followPath(coroutine, AutoPaths.AUTO_TWO_PATH_ONE);
            coroutine.await(auto.armUp());
            auto.followPath(coroutine, AutoPaths.AUTO_TWO_PATH_TWO);
            coroutine.await(auto.armDown());
            auto.shootWhileAiming(coroutine, 3);
        }).named(mirror ? "Auto Two (Mirrored)" : "Auto Two");

        // Created in the opmode, so this binding only exists while the opmode is selected. The routine starts when
        // autonomous is enabled and is canceled when it is disabled.
        RobotModeTriggers.autonomous().whileTrue(routine);
    }
}
