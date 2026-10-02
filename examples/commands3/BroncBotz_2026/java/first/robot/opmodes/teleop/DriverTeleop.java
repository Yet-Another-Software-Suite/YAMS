// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later
// Ported from BroncBotz 3481 FRC2026.

package first.robot.opmodes.teleop;

import static first.robot.Constants.FeedConstants.kAgitatorBackOff;

import first.robot.Constants.DriveConstants;
import first.robot.Constants.ShooterConstants;
import first.robot.Field;
import first.robot.Robot;
import org.wpilib.command3.button.CommandNiDsXboxController;
import org.wpilib.opmode.OpMode;
import org.wpilib.opmode.Teleop;

/**
 * Teleop with the original's driver and operator bindings. They are created in the constructor, so
 * they only exist while this opmode is selected. Driving and the operator's manual intake arm control
 * are default commands, set in {@link Robot}.
 */
@Teleop(name = "Driver Teleop")
public class DriverTeleop implements OpMode {
    /**
     * Creates the teleop opmode. The OpModeRobot framework calls this when the opmode is selected on
     * the driver station.
     *
     * @param robot The robot instance to control.
     */
    public DriverTeleop(Robot robot) {
        configureDriverBindings(robot, robot.driver);
        configureOperatorBindings(robot, robot.operator);
    }

    private static void configureDriverBindings(Robot robot, CommandNiDsXboxController driver) {
        driver.a().whileTrue(robot.aimAtHub());
        driver.rightBumper().whileTrue(robot.swerve.drive(() -> -driver.getLeftY(), () -> -driver.getLeftX(),
            () -> -driver.getRightX(), DriveConstants.kSlowModeTranslationScale, false, "Slow Mode"));
        driver.leftBumper().whileTrue(robot.swerve.lockPose());
        driver.start().and(driver.back()).onTrue(robot.swerve.zeroGyroWithAlliance());
        driver.x().whileTrue(robot.agitator.runAt(kAgitatorBackOff));
        // The D-pad triggers live on the generic HID in 2027.
        driver.getHID().povUp().onTrue(robot.swerve.resetOdometryCommand(Field.kBlueRobotPoseAtHub));
    }

    private static void configureOperatorBindings(Robot robot, CommandNiDsXboxController operator) {
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
