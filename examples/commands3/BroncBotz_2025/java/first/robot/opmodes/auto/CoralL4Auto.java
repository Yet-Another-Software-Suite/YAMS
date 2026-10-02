// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later
// Ported from BroncBotz 3481 FRC2025 (comp branch).

package first.robot.opmodes.auto;

import first.robot.Constants.CoralArmConstants;
import first.robot.Constants.ElevatorConstants;
import first.robot.Robot;
import first.robot.util.ReefTargeting;
import org.wpilib.command3.Command;
import org.wpilib.command3.button.RobotModeTriggers;
import org.wpilib.opmode.Autonomous;
import org.wpilib.opmode.OpMode;

/**
 * Score one coral on L4 of branch H. The comp branch built a PathPlanner auto chooser but always ran
 * {@code justCoralL4Auto(ReefBranch.H)}, which only uses drive to pose, so that is the one routine
 * kept and PathPlanner is not used.
 */
@Autonomous(name = "Coral L4 H")
public class CoralL4Auto implements OpMode {
    private static final ReefTargeting.Branch kBranch = ReefTargeting.Branch.H;

    /**
     * Creates the autonomous opmode. The OpModeRobot framework calls this when the opmode is selected
     * on the driver station.
     *
     * @param robot The robot instance to control.
     */
    public CoralL4Auto(Robot robot) {
        final Command liftClear = robot.elevator.moveTo(ElevatorConstants.kAutoClearHeight);
        final Command holdClear = robot.elevator.holdAt(ElevatorConstants.kAutoClearHeight);
        final Command swingOut = robot.coralArm.moveTo(CoralArmConstants.kStowed);
        final Command score = robot.coralCommands.scoreCoral();

        // Swing the coral arm out of its starting position while holding the elevator clear. The hold
        // is forked inside this step, so it ends with it. Forked by the routine instead, it would
        // still be running when the scoring command takes the elevator, and a command interrupting
        // a sibling of its ancestor cancels their shared ancestor: the whole routine.
        final Command swingOutWhileClear = Command.noRequirements(coroutine -> {
            coroutine.fork(holdClear);
            coroutine.await(swingOut);
        }).named("Swing Out While Clear");

        final Command routine = Command.noRequirements(coroutine -> {
            robot.targeting.setTarget(kBranch, ReefTargeting.Level.L4);
            coroutine.await(liftClear);
            coroutine.await(swingOutWhileClear);
            coroutine.await(score);
        }).named("Coral L4 " + kBranch);

        // Created in the opmode, so this binding only exists while the opmode is selected. The
        // routine starts when autonomous is enabled and is canceled when it is disabled.
        RobotModeTriggers.autonomous().whileTrue(routine);
    }
}
