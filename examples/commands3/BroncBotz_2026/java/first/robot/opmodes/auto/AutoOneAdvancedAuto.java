// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later
// Ported from BroncBotz 3481 FRC2026.

package first.robot.opmodes.auto;

import first.robot.Robot;
import org.wpilib.command3.Command;
import org.wpilib.command3.button.RobotModeTriggers;
import org.wpilib.opmode.Autonomous;
import org.wpilib.opmode.OpMode;

/** The "Auto One Advanced" PathPlanner auto, driven with drive to pose. See {@link AutoPaths} for how the paths were converted. */
@Autonomous(name = "Auto One Advanced")
public class AutoOneAdvancedAuto implements OpMode {
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
        final AutoRoutines auto = new AutoRoutines(robot, mirror);
        final Command routine = Command.noRequirements(coroutine -> {
            coroutine.await(auto.resetPose(AutoPaths.AUTO_ONE_ADVANCED_PATH_1));
            coroutine.await(auto.armDown());
            auto.followPath(coroutine, AutoPaths.AUTO_ONE_ADVANCED_PATH_1);
            auto.shootWhileAiming(coroutine, 3);
            auto.followPath(coroutine, AutoPaths.AUTO_ONE_ADVANCED_PATH_FOLLOW);
            auto.followPath(coroutine, AutoPaths.AUTO_ONE_ADVANCED_PATH_2);
            auto.shootWhileAiming(coroutine, 3);
        }).named(mirror ? "Auto One Advanced (Mirrored)" : "Auto One Advanced");

        // Created in the opmode, so this binding only exists while the opmode is selected. The routine starts when
        // autonomous is enabled and is canceled when it is disabled.
        RobotModeTriggers.autonomous().whileTrue(routine);
    }
}
