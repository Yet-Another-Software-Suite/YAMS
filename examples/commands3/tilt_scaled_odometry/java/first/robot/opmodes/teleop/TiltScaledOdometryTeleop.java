// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package first.robot.opmodes.teleop;

import first.robot.Robot;
import org.wpilib.opmode.OpMode;
import org.wpilib.opmode.Teleop;
import yams.commands3.swerve.SwerveInputStream;

@Teleop(name = "Tilt Scaled Odometry Teleop")
public class TiltScaledOdometryTeleop implements OpMode {
  public TiltScaledOdometryTeleop(Robot robot) {
    SwerveInputStream input =
        robot.drive.getChassisSpeedsSupplier(
            () -> -robot.driverController.getLeftY(),
            () -> -robot.driverController.getLeftX(),
            () -> -robot.driverController.getRightX());
    robot.drive.setDefaultCommand(robot.drive.driveFieldRelative(input));
    robot.driverController.y().onTrue(robot.drive.zeroGyro());
  }
}
