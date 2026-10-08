// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later
// Ported from the WCP 2026 Competitive Concept (MIT, see LICENSE-WCP).

package first.robot.opmodes.teleop;

import first.robot.Robot;
import first.robot.mechanisms.Hanger;
import first.robot.mechanisms.IntakePivot;
import org.wpilib.command3.Command;
import org.wpilib.command3.button.CommandNiDsXboxController;

/**
 * Bindings shared by every teleop opmode, with the original WCP buttons. Call from an opmode constructor so they are
 * opmode-scoped. Homing is bound in {@link Robot}.
 */
final class TeleopBindings {
    private TeleopBindings() {
        throw new UnsupportedOperationException("This is a utility class!");
    }

    /**
     * Bind the shooting, intake, hanger, and field centric reset controls.
     *
     * @param robot The robot.
     */
    static void bindMechanisms(Robot robot) {
        final CommandNiDsXboxController driver = robot.driver;

        // The drive input stream aims at the hub while the right trigger is held.
        driver.rightTrigger().whileTrue(robot.mechanismCommands.shootWhenAimed());
        driver.rightBumper().whileTrue(robot.mechanismCommands.shootManually());
        driver.leftTrigger().whileTrue(robot.mechanismCommands.intake());
        driver.leftBumper().onTrue(robot.intakePivot.moveTo(IntakePivot.Position.STOWED));
        // The D-pad triggers live on the generic HID in 2027.
        driver.getHID().povUp().onTrue(robot.hanger.moveTo(Hanger.Position.HANGING));
        driver.getHID().povDown().onTrue(robot.hanger.moveTo(Hanger.Position.HUNG));
        // Make the direction the robot is facing "forward".
        driver.back().onTrue(Command.noRequirements(coroutine -> robot.swerve.seedFieldCentric())
            .named("Seed Field Centric"));
    }
}
