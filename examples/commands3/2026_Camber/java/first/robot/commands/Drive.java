// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later
// Ported from Team 9658's 2026-KitBot (https://github.com/9658-Camber-Robotics/2026-KitBot).

package first.robot.commands;

import static org.wpilib.units.Units.Radians;

import first.robot.Constants.SwerveDrive;
import first.robot.mechanisms.SwerveMechanism;
import first.robot.utils.AllianceFlipUtil;
import first.robot.utils.FieldConstants.Hub;
import java.util.List;
import org.wpilib.command3.Command;
import org.wpilib.command3.button.CommandNiDsXboxController;
import org.wpilib.driverstation.NiDsXboxController;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import yams.commands3.swerve.SwerveInputStream;

/**
 * Driver commands for the swerve drivetrain: the original {@code driveDirectAngle} default drive and
 * {@code AutoAimCommand}. Each command owns a YAMS {@link SwerveInputStream}, sets its sticks every
 * loop, and drives from it.
 */
public final class Drive
{

  private Drive()
  {
    throw new UnsupportedOperationException("This is a utility class!");
  }

  /**
   * Field oriented drive: the left stick translates and the robot faces the heading the right stick
   * points at. Like YAGSL, the last heading is kept while the right stick is near the center, starting
   * at 0.
   *
   * @param swerve     The drivetrain.
   * @param controller The driver's controller.
   * @return A command that drives until canceled.
   */
  public static Command driveDirectAngle(SwerveMechanism swerve, CommandNiDsXboxController controller)
  {
    NiDsXboxController hid = controller.getNiDsXboxController();
    return swerve.run(coroutine -> {
      double[] heading = new double[]{0};
      SwerveInputStream input = swerve.createInputStream(
          () -> hid.getLeftY() * -1,
          () -> hid.getLeftX() * -1
      ).withHeading(() -> {
        double x = hid.getRightX();
        double y = hid.getRightY();
        if (Math.hypot(x, y) > SwerveDrive.Modules.angleJoystickRadiusDeadband)
        {
          heading[0] = Math.atan2(x, y);
        }
        return Radians.of(heading[0]);
      }).setHeadingControl(true);
      while (true)
      {
        swerve.driveFieldOrientedSetpoint(input.get());
        coroutine.yield();
      }
    }).named("Drive Direct Angle");
  }

  /**
   * Translate from the left stick while the robot turns to face the hub. Runs until canceled.
   *
   * @param swerve     The drivetrain.
   * @param controller The driver's controller.
   * @return A command that aims while driving until canceled.
   */
  public static Command autoAim(SwerveMechanism swerve, CommandNiDsXboxController controller)
  {
    NiDsXboxController hid = controller.getNiDsXboxController();
    return swerve.run(coroutine -> {
      Pose2d targetPose = AllianceFlipUtil.apply(new Pose2d(Hub.topCenterPoint.toTranslation2d(), Rotation2d.ZERO));
      swerve.getField().getObject("AimTarget").setPose(targetPose);
      SwerveInputStream input = swerve.createInputStream(
          () -> hid.getLeftY() * -1,
          () -> hid.getLeftX() * -1
      ).withControllerRotationAxis(() -> hid.getRightX())
       .withAimTarget(() -> targetPose)
       .setAim(true);
      while (true)
      {
        swerve.driveFieldOrientedSetpoint(input.get());
        coroutine.yield();
      }
    }).whenCanceled(() -> swerve.getField().getObject("AimTarget").setPoses(List.of())).named("Auto Aim");
  }
}
