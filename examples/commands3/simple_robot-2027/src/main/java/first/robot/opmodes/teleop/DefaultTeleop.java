// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package first.robot.opmodes.teleop;

import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.RPM;

import first.robot.Robot;
import org.wpilib.opmode.OpMode;
import org.wpilib.opmode.Teleop;

@Teleop
public class DefaultTeleop implements OpMode {
  public DefaultTeleop(Robot robot) {
    var hid = robot.xboxController.getHID();

    hid.button(1).whileTrue(robot.arm.setAngle(Degrees.of(30)));
    hid.button(2).whileTrue(robot.arm.setAngle(Degrees.of(80)));

    hid.button(3).whileTrue(robot.elevator.setHeight(Meters.of(0.5)));
    hid.button(4).whileTrue(robot.elevator.setHeight(Meters.of(1.5)));

    hid.button(5).whileTrue(robot.shooter.setVelocity(RPM.of(3500)));
    hid.button(6).whileTrue(robot.shooter.setVelocity(RPM.of(2000)));
  }
}
