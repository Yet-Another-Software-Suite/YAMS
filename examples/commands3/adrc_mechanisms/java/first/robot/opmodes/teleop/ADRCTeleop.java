// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package first.robot.opmodes.teleop;

import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.RPM;

import first.robot.Robot;
import org.wpilib.opmode.OpMode;
import org.wpilib.opmode.Teleop;

/** Teleop for the ADRC mechanisms. The bindings are created when this opmode is selected and removed when it exits. */
@Teleop(name = "ADRC Teleop")
public class ADRCTeleop implements OpMode {
  /**
   * Creates the teleop opmode. This constructor is called automatically by the OpModeRobot framework when the opmode
   * is selected on the driver station.
   *
   * @param robot The robot instance to control.
   */
  public ADRCTeleop(Robot robot) {
    // A raises the arm level, B stows it.
    robot.operatorController.a().onTrue(robot.arm.setAngle(Degrees.of(0)));
    robot.operatorController.b().onTrue(robot.arm.setAngle(Degrees.of(-20)));
    // Y raises the elevator to 1 m, X lowers it.
    robot.operatorController.y().onTrue(robot.elevator.setHeight(Meters.of(1)));
    robot.operatorController.x().onTrue(robot.elevator.setHeight(Meters.of(0)));
    // Hold the right bumper to spin the shooter up.
    robot.operatorController.rightBumper().whileTrue(robot.shooter.spinUp(RPM.of(3000)));
  }
}
