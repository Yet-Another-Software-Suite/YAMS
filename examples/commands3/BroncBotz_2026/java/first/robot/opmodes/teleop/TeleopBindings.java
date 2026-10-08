// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later
// Ported from BroncBotz 3481 FRC2026.

package first.robot.opmodes.teleop;

import static first.robot.Constants.FeedConstants.kAgitatorBackOff;

import first.robot.Constants.ShooterConstants;
import first.robot.Field;
import first.robot.Robot;
import org.wpilib.command3.button.CommandNiDsXboxController;

/**
 * The original's driver and operator bindings, shared by every teleop opmode. Call from an opmode constructor so they
 * only exist while that opmode is selected. The operator's manual intake arm control is a default command, set in
 * {@link Robot}.
 */
final class TeleopBindings {
    private TeleopBindings() {
        throw new UnsupportedOperationException("This is a utility class!");
    }

    /**
     * Bind the driver and operator controls.
     *
     * @param robot The robot.
     */
    static void bindAll(Robot robot) {
        bindDriver(robot, robot.driver);
        bindOperator(robot, robot.operator);
    }

    private static void bindDriver(Robot robot, CommandNiDsXboxController driver) {
        driver.a().whileTrue(robot.swerve.aimAtHub());
        driver.rightBumper().whileTrue(robot.swerve.slowMode());
        driver.leftBumper().whileTrue(robot.swerve.lockPose());
        driver.start().and(driver.back()).onTrue(robot.swerve.zeroGyroWithAlliance());
        driver.x().whileTrue(robot.agitator.runAt(kAgitatorBackOff));
        // The D-pad triggers live on the generic HID in 2027.
        driver.getHID().povUp().onTrue(robot.swerve.resetOdometryCommand(Field.kBlueRobotPoseAtHub));
    }

    private static void bindOperator(Robot robot, CommandNiDsXboxController operator) {
        operator.getHID().povRight().whileTrue(robot.superstructure.shootAt(ShooterConstants.kCloseShot));
        operator.x().whileTrue(robot.superstructure.shootAt(ShooterConstants.kMidShot));
        operator.y().whileTrue(robot.superstructure.shootAt(ShooterConstants.kFarShot));
        operator.rightTrigger(0.2).whileTrue(robot.superstructure.shootFromDistance());
        operator.getHID().povLeft().whileTrue(robot.superstructure.pass());

        operator.leftTrigger(0.3).whileTrue(robot.superstructure.intake());
        operator.b().whileTrue(robot.superstructure.outtake());
        operator.a().whileTrue(robot.superstructure.unjam());

        operator.leftBumper().onTrue(robot.intakeArm.toIntake());
        operator.rightBumper().whileTrue(robot.intakeArm.agitate());
        operator.getHID().povDown().onTrue(robot.intakeArm.toDown());
        operator.getHID().povUp().onTrue(robot.intakeArm.toUp());
        operator.start().and(operator.back()).onTrue(robot.intakeArm.resetEncoder());
    }
}
