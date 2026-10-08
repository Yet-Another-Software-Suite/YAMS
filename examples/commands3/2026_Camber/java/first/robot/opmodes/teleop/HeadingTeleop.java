// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later
// Ported from Team 9658's 2026-KitBot (https://github.com/9658-Camber-Robotics/2026-KitBot).

package first.robot.opmodes.teleop;

import first.robot.Robot;
import org.wpilib.command3.button.CommandNiDsXboxController;
import org.wpilib.opmode.OpMode;
import org.wpilib.opmode.Teleop;
import yams.commands3.swerve.SwerveInputStream;

/**
 * Teleop where the left stick translates and the robot faces the direction the right stick is pushed, replacing the
 * original {@code driveDirectAngle}.
 */
@Teleop
public class HeadingTeleop implements OpMode
{

  /**
   * Creates the teleop opmode. The OpModeRobot framework calls this when the opmode is selected on
   * the driver station.
   *
   * @param robot The robot instance to control.
   */
  public HeadingTeleop(Robot robot)
  {
    CommandNiDsXboxController driverController = robot.driverController;
    robot.drivebase.setInputStream(SwerveInputStream.of(
            robot.drivebase.getSwerveDrive(),
            () -> -driverController.getLeftY(),
            () -> -driverController.getLeftX())
        // Heading axes are (left, forward), so pushing the right stick away from the driver faces the robot forward.
        .withControllerHeadingAxis(() -> -driverController.getRightX(), () -> -driverController.getRightY())
        .withHeadingControl(() -> true)
        .withDeadband(0.1)
        .withScaleTranslation(.8)
        .withAllianceRelativeControl());

    // Opmode-scoped default command and bindings: they only exist while this teleop runs.
    robot.drivebase.setDefaultCommand(robot.drivebase.driveInputStream());
    TeleopBindings.bind(robot);
  }
}
