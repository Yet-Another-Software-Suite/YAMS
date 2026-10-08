// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package first.robot.opmodes.teleop;

import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.RPM;

import first.robot.Robot;

/** Mechanism bindings shared by every teleop opmode. Call from an opmode constructor so they are opmode-scoped. */
final class TeleopBindings {
  private TeleopBindings() {
    throw new UnsupportedOperationException("This is a utility class!");
  }

  /**
   * Bind the arm, elevator, and shooter controls.
   *
   * @param robot The robot.
   */
  static void bindMechanisms(Robot robot) {
    var controller = robot.xboxController;
    var hid = controller.getHID();

    hid.button(1).whileTrue(robot.arm.setAngle(Degrees.of(30)));
    hid.button(2).whileTrue(robot.arm.setAngle(Degrees.of(80)));

    hid.button(3).whileTrue(robot.elevator.setHeight(Meters.of(0.5)));
    hid.button(4).whileTrue(robot.elevator.setHeight(Meters.of(1.5)));

    hid.button(5).whileTrue(robot.shooter.setVelocity(RPM.of(3500)));
    hid.button(6).whileTrue(robot.shooter.setVelocity(RPM.of(2000)));

    controller.leftTrigger().whileTrue(robot.arm.pickUp());
    controller.rightTrigger().onTrue(robot.scoreHigh());
  }
}
