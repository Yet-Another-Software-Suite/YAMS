// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later
// Ported from Team 9658's 2026-KitBot (https://github.com/9658-Camber-Robotics/2026-KitBot).

package first.robot.commands;

import first.robot.subsystems.SwerveSubsystem;
import first.robot.utils.AllianceFlipUtil;
import first.robot.utils.FieldConstants.Hub;
import java.util.List;
import org.wpilib.command2.Command;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import yams.commands2.swerve.SwerveInputStream;


public class AutoAimCommand extends Command
{

  private final SwerveSubsystem   swerveSubsystem;
  private final SwerveInputStream swerveInputStream;
  private Pose2d targetPose;
  private boolean aiming = false;

  public AutoAimCommand(SwerveSubsystem swerveSubsystem, SwerveInputStream swerveInputStream, double slowScale)
  {
    this.swerveSubsystem = swerveSubsystem;
    this.swerveInputStream = swerveInputStream.clone()
                                              // The target is picked each time the command starts.
                                              .withAim(() -> targetPose, () -> aiming);
//    this.swerveInputStream.withScaleTranslation(slowScale);
    // each subsystem used by the command must be passed into the
    // addRequirements() method (which takes a vararg of Subsystem)
    addRequirements(this.swerveSubsystem);
  }

  @Override
  public void initialize()
  {
    targetPose = AllianceFlipUtil.apply(new Pose2d(Hub.topCenterPoint.toTranslation2d(), Rotation2d.ZERO));
    swerveSubsystem.getField().getObject("AimTarget").setPose(targetPose);

    aiming = true;

  }

  @Override
  public void execute()
  {
    swerveSubsystem.driveFieldOrientedSetpoint(swerveInputStream.get());
  }

  @Override
  public boolean isFinished()
  {
    // TODO: Make this return true when this Command no longer needs to run execute()
    return false;
  }

  @Override
  public void end(boolean interrupted)
  {
    aiming = false;
    swerveSubsystem.getField().getObject("AimTarget").setPoses(List.of());
  }
}
