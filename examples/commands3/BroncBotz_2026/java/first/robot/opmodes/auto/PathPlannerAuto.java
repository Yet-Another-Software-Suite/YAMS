// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later
// Ported from BroncBotz 3481 FRC2026.

package first.robot.opmodes.auto;

import first.robot.Robot;
import first.robot.pathplanner.AutoBuilder;
import org.wpilib.command3.button.RobotModeTriggers;
import org.wpilib.opmode.OpMode;

/**
 * Runs a PathPlanner auto from {@code deploy/pathplanner/autos}. It has no {@code @Autonomous}
 * annotation: {@link Robot} adds one of these for every auto file, so a new auto drawn in the GUI
 * shows up on the driver station without new code.
 */
public class PathPlannerAuto implements OpMode {
    /**
     * Creates the autonomous opmode and loads the auto. The OpModeRobot framework calls this when the
     * opmode is selected on the driver station, so loading happens while disabled.
     *
     * @param autoName Name of the auto in the PathPlanner GUI.
     * @param mirror   Mirror the auto across the field's long centerline, to run it on the other side.
     */
    public PathPlannerAuto(String autoName, boolean mirror) {
        // Created in the opmode, so this binding only exists while the opmode is selected. The auto
        // starts when autonomous is enabled and is canceled when it is disabled.
        RobotModeTriggers.autonomous().whileTrue(AutoBuilder.buildAuto(autoName, mirror));
    }
}
