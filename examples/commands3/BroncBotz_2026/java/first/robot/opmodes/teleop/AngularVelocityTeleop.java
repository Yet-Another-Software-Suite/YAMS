// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later
// Ported from BroncBotz 3481 FRC2026.

package first.robot.opmodes.teleop;

import first.robot.Constants.DriveConstants;
import first.robot.Constants.OperatorConstants;
import first.robot.Robot;
import org.wpilib.opmode.OpMode;
import org.wpilib.opmode.Teleop;
import yams.commands3.swerve.SwerveInputStream;

/** Teleop where the left stick translates and the X axis of the right stick sets the rotation rate. */
@Teleop(name = "Angular Velocity Teleop")
public class AngularVelocityTeleop implements OpMode {
    /**
     * Creates the teleop opmode. The OpModeRobot framework calls this when the opmode is selected on
     * the driver station.
     *
     * @param robot The robot instance to control.
     */
    public AngularVelocityTeleop(Robot robot) {
        var driver = robot.driver;
        robot.swerve.setInputStream(SwerveInputStream.of(
                robot.swerve.getSwerveDrive(),
                () -> -driver.getLeftY(),
                () -> -driver.getLeftX(),
                () -> -driver.getRightX())
            .withDeadband(OperatorConstants.kDeadband)
            .withScaleTranslation(DriveConstants.kTranslationScale)
            .withScaleRotation(DriveConstants.kRotationScale)
            .withAllianceRelativeControl());

        // Opmode-scoped default command and bindings: they only exist while this teleop runs.
        robot.swerve.setDefaultCommand(robot.swerve.driveInputStream());
        TeleopBindings.bindAll(robot);
    }
}
