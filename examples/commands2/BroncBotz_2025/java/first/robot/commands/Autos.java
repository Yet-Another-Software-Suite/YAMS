// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later
// Ported from BroncBotz 3481 FRC2025 (comp branch).

package first.robot.commands;

import first.robot.Constants.CoralArmConstants;
import first.robot.Constants.ElevatorConstants;
import first.robot.subsystems.CoralArm;
import first.robot.subsystems.Elevator;
import first.robot.util.ReefTargeting;
import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;

/**
 * The autonomous routine. The comp branch built a PathPlanner auto chooser but always ran
 * {@code justCoralL4Auto(ReefBranch.H)}, which only uses drive to pose, so that is the one routine
 * kept and PathPlanner is not used.
 */
public final class Autos {
    private Autos() {
        throw new UnsupportedOperationException("This is a utility class!");
    }

    /**
     * Score one coral on L4 of a branch: lift the elevator clear, swing the coral arm out of its
     * starting position, then score and stow the arms.
     */
    public static Command coralL4(ReefTargeting.Branch branch, ReefTargeting targeting, Elevator elevator,
                                  CoralArm coralArm, SuperstructureCommands superstructure) {
        return Commands.sequence(
            Commands.runOnce(() -> targeting.setTarget(branch, ReefTargeting.Level.L4)),
            elevator.moveTo(ElevatorConstants.kAutoClearHeight),
            elevator.holdAt(ElevatorConstants.kAutoClearHeight).withDeadline(coralArm.moveTo(CoralArmConstants.kStowed)),
            superstructure.scoreCoral(),
            superstructure.restArmsSafe()
        ).withName("Coral L4 " + branch);
    }
}
