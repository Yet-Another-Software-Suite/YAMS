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

/** Teleop where the left stick translates and the robot faces the direction the right stick is pushed. */
@Teleop(name = "Heading Teleop")
public class HeadingTeleop implements OpMode {
    /**
     * Creates the teleop opmode. The OpModeRobot framework calls this when the opmode is selected on
     * the driver station.
     *
     * @param robot The robot instance to control.
     */
    public HeadingTeleop(Robot robot) {
        var driver = robot.driver;
        robot.swerve.setInputStream(SwerveInputStream.of(
                robot.swerve.getSwerveDrive(),
                () -> -driver.getLeftY(),
                () -> -driver.getLeftX())
            // Heading axes are (left, forward), so pushing the right stick away from the driver faces the robot
            // forward.
            .withControllerHeadingAxis(() -> -driver.getRightX(), () -> -driver.getRightY())
            .withHeadingControl(() -> true)
            .withDeadband(OperatorConstants.kDeadband)
            .withScaleTranslation(DriveConstants.kTranslationScale)
            .withAllianceRelativeControl());

        // Opmode-scoped default command and bindings: they only exist while this teleop runs.
        robot.swerve.setDefaultCommand(robot.swerve.driveInputStream());
        TeleopBindings.bindAll(robot);
    }
}
