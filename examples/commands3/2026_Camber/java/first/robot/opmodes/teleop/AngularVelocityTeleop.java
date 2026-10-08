// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later
// Ported from Team 9658's 2026-KitBot (https://github.com/9658-Camber-Robotics/2026-KitBot).

package first.robot.opmodes.teleop;

import first.robot.Robot;
import org.wpilib.command3.button.CommandNiDsXboxController;
import org.wpilib.opmode.OpMode;
import org.wpilib.opmode.Teleop;
import yams.commands3.swerve.SwerveInputStream;
import yams.core.telemetry.enums.TelemetryVerbosity;

/** Teleop where the left stick translates and the X axis of the right stick sets the rotation rate. */
@Teleop
public class AngularVelocityTeleop implements OpMode
{

  /**
   * Creates the teleop opmode. The OpModeRobot framework calls this when the opmode is selected on
   * the driver station.
   *
   * @param robot The robot instance to control.
   */
  public AngularVelocityTeleop(Robot robot)
  {
    CommandNiDsXboxController driverController = robot.driverController;
    robot.drivebase.setInputStream(SwerveInputStream.of(
            robot.drivebase.getSwerveDrive(),
            () -> -driverController.getLeftY(),
            () -> -driverController.getLeftX(),
            () -> -driverController.getRightX())
        .withDeadband(0.1)
        .withScaleTranslation(.8)
        .withAllianceRelativeControl()
        .withTelemetry("Driver", TelemetryVerbosity.HIGH));

    // Opmode-scoped default command and bindings: they only exist while this teleop runs.
    robot.drivebase.setDefaultCommand(robot.drivebase.driveInputStream());
    TeleopBindings.bind(robot);
  }
}
