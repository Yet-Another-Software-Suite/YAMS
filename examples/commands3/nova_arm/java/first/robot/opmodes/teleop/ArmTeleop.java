// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package first.robot.opmodes.teleop;

import static first.robot.Constants.ArmConstants.*;

import first.robot.Robot;
import org.wpilib.opmode.OpMode;
import org.wpilib.opmode.Teleop;

/** Teleop for the arm. The bindings are created when this opmode is selected and removed when it exits. */
@Teleop(name = "Arm Teleop")
public class ArmTeleop implements OpMode {
  /**
   * Creates the teleop opmode. This constructor is called automatically by the OpModeRobot framework when the opmode
   * is selected on the driver station.
   *
   * @param robot The robot instance to control.
   */
  public ArmTeleop(Robot robot) {
    // Hold a button to move the arm there; let go and it stows.
    robot.operatorController.a().whileTrue(robot.arm.setAngle(LEVEL));
    robot.operatorController.b().whileTrue(robot.arm.setAngle(HIGH));
    robot.operatorController.y().whileTrue(robot.arm.setAngle(UP));
  }
}
