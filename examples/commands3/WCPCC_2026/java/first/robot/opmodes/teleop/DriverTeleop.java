// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later
// Ported from the WCP 2026 Competitive Concept (MIT, see LICENSE-WCP).

package first.robot.opmodes.teleop;

import first.robot.Robot;
import first.robot.mechanisms.Hanger;
import first.robot.mechanisms.IntakePivot;
import org.wpilib.command3.button.CommandNiDsXboxController;
import org.wpilib.opmode.OpMode;
import org.wpilib.opmode.Teleop;

/**
 * Driver controlled teleop with the original WCP bindings. The bindings are created in the
 * constructor, so they only exist while this opmode is selected.
 */
@Teleop(name = "Driver Teleop")
public class DriverTeleop implements OpMode {
    private final Robot robot;

    /**
     * Creates the teleop opmode. The OpModeRobot framework calls this when the opmode is selected on
     * the driver station.
     *
     * @param robot The robot instance to control.
     */
    public DriverTeleop(Robot robot) {
        this.robot = robot;
        final CommandNiDsXboxController driver = robot.driver;

        // The drive command aims at the hub while the right trigger is held.
        driver.rightTrigger().whileTrue(robot.mechanismCommands.aimAndShoot());
        driver.rightBumper().whileTrue(robot.mechanismCommands.shootManually());
        driver.leftTrigger().whileTrue(robot.mechanismCommands.intake());
        driver.leftBumper().onTrue(robot.intakePivot.positionCommand(IntakePivot.Position.STOWED));
        // The D-pad triggers live on the generic HID in 2027.
        driver.getHID().povUp().onTrue(robot.hanger.positionCommand(Hanger.Position.HANGING));
        driver.getHID().povDown().onTrue(robot.hanger.positionCommand(Hanger.Position.HUNG));
    }

    /** Homing runs when teleop is enabled, as it did on the v2 port's teleop trigger. */
    @Override
    public void start() {
        robot.scheduleHoming();
    }

    /** Stop homing when teleop is disabled, as the v2 port did. */
    @Override
    public void end() {
        robot.cancelHoming();
    }
}
