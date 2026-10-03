// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later
// Ported from Team 9658's 2026-KitBot (https://github.com/9658-Camber-Robotics/2026-KitBot).

package first.robot.opmodes.auto;

import first.robot.Robot;
import org.wpilib.command3.button.RobotModeTriggers;
import org.wpilib.opmode.Autonomous;
import org.wpilib.opmode.OpMode;

/** Runs the "Right Auto" PathPlanner auto from {@code deploy/pathplanner/autos}. */
@Autonomous(name = "Right Auto")
public class RightAuto implements OpMode
{

  /**
   * Creates the autonomous opmode and loads the auto. The OpModeRobot framework calls this when the
   * opmode is selected on the driver station, so loading happens while disabled.
   *
   * @param robot The robot instance to control.
   */
  public RightAuto(Robot robot)
  {
    // The auto starts when autonomous is enabled and is canceled when it is disabled.
    RobotModeTriggers.autonomous().whileTrue(robot.drivebase.getAutonomousCommand("Right Auto"));
  }
}
