// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package first.robot.opmodes.auto;

import first.robot.Robot;
import org.wpilib.command3.button.RobotModeTriggers;
import org.wpilib.opmode.Autonomous;
import org.wpilib.opmode.OpMode;

/** Raise the arm and elevator to the scoring position, then shoot the preloaded game piece. */
@Autonomous(name = "Score Preload")
public class ScorePreloadAuto implements OpMode {
  /**
   * Creates the autonomous opmode. LoggedOpModeRobot calls this when the opmode is selected on the driver station.
   *
   * @param robot The robot instance to control.
   */
  public ScorePreloadAuto(Robot robot) {
    // The routine starts when autonomous is enabled and is canceled when it is disabled.
    RobotModeTriggers.autonomous().whileTrue(robot.scorePreloadAuto());
  }
}
