// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later
// Ported from BroncBotz 3481 FRC2026.

package first.robot.opmodes.auto;

import first.robot.Robot;
import org.wpilib.command3.Command;
import org.wpilib.command3.button.RobotModeTriggers;
import org.wpilib.opmode.Autonomous;
import org.wpilib.opmode.OpMode;

/** The "Auto One" PathPlanner auto, driven with drive to pose. See {@link AutoPaths} for how the paths were converted. */
@Autonomous(name = "Auto One")
public class AutoOneAuto implements OpMode {
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
        final AutoRoutines auto = new AutoRoutines(robot, mirror);
        final Command routine = Command.noRequirements(coroutine -> {
            coroutine.await(auto.resetPose(AutoPaths.AUTO_ONE_PATH_ONE));
            auto.followPath(coroutine, AutoPaths.AUTO_ONE_PATH_ONE);
            auto.shoot(coroutine);
        }).named(mirror ? "Auto One (Mirrored)" : "Auto One");

        // Created in the opmode, so this binding only exists while the opmode is selected. The routine starts when
        // autonomous is enabled and is canceled when it is disabled.
        RobotModeTriggers.autonomous().whileTrue(routine);
    }
}
