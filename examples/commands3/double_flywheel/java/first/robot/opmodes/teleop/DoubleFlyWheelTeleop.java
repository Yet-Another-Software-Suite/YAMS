// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package first.robot.opmodes.teleop;

import static first.robot.Constants.DoubleFlyWheelConstants.*;

import first.robot.Robot;
import org.wpilib.opmode.OpMode;
import org.wpilib.opmode.Teleop;

/** Teleop for the double flywheel. The bindings are created when this opmode is selected and removed when it exits. */
@Teleop(name = "Double FlyWheel Teleop")
public class DoubleFlyWheelTeleop implements OpMode {
  /**
   * Creates the teleop opmode. This constructor is called automatically by the OpModeRobot framework when the opmode
   * is selected on the driver station.
   *
   * @param robot The robot instance to control.
   */
  public DoubleFlyWheelTeleop(Robot robot) {
    // Hold the right bumper to spin both wheels up for a shot.
    robot.operatorController.rightBumper().whileTrue(robot.doubleFlyWheel.spinUp(SHOT_LOWER_SPEED, SHOT_UPPER_SPEED));
    // Press Start to live tune both wheels from code, the same command the YAMS Live Tuner starts. Press it again to
    // stop.
    robot.operatorController.start().toggleOnTrue(robot.doubleFlyWheel.liveTune());
  }
}
