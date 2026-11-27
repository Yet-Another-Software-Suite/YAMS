// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package first.robot.opmodes.teleop;

import first.robot.Robot;
import org.wpilib.opmode.OpMode;
import org.wpilib.opmode.Teleop;

@Teleop(name = "Power Health Monitoring")
public class PowerHealthTeleop implements OpMode {
  public PowerHealthTeleop(Robot robot) {
    robot.drive.setDefaultCommand(
        robot.drive.driveFieldRelative(
            () -> -robot.driverController.getLeftY(),
            () -> -robot.driverController.getLeftX(),
            () -> -robot.driverController.getRightX()));
  }
}
