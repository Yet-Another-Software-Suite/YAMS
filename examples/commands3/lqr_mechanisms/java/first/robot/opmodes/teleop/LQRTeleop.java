// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package first.robot.opmodes.teleop;

import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.RPM;

import first.robot.Robot;
import org.wpilib.opmode.OpMode;
import org.wpilib.opmode.Teleop;

/** Teleop for the LQR mechanisms. The bindings are created when this opmode is selected and removed when it exits. */
@Teleop(name = "LQR Teleop")
public class LQRTeleop implements OpMode {
  /**
   * Creates the teleop opmode. This constructor is called automatically by the OpModeRobot framework when the opmode
   * is selected on the driver station.
   *
   * @param robot The robot instance to control.
   */
  public LQRTeleop(Robot robot) {
    // A raises the arm level, B stows it.
    robot.operatorController.a().onTrue(robot.arm.setAngle(Degrees.of(0)));
    robot.operatorController.b().onTrue(robot.arm.setAngle(Degrees.of(-20)));
    // Hold the right bumper to spin the flywheel up.
    robot.operatorController.rightBumper().whileTrue(robot.flyWheel.spinUp(RPM.of(3000)));
  }
}
