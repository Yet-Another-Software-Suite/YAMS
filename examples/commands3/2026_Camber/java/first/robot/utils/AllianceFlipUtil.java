// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.
//
// Ported to WPILib 2027 for YAMS from Team 9658's 2026-KitBot
// (https://github.com/9658-Camber-Robotics/2026-KitBot).

package first.robot.utils;

import org.wpilib.driverstation.Alliance;
import org.wpilib.driverstation.MatchState;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.geometry.Translation3d;

public class AllianceFlipUtil
{

  public static double applyX(double x)
  {
    return shouldFlip() ? FieldConstants.fieldLength - x : x;
  }

  public static double applyY(double y)
  {
    return shouldFlip() ? FieldConstants.fieldWidth - y : y;
  }

  public static Translation2d apply(Translation2d translation)
  {
    return new Translation2d(applyX(translation.getX()), applyY(translation.getY()));
  }

  public static Rotation2d apply(Rotation2d rotation)
  {
    return shouldFlip() ? rotation.rotateBy(Rotation2d.k180deg) : rotation;
  }

  public static Pose2d apply(Pose2d pose)
  {
    return shouldFlip()
           ? new Pose2d(apply(pose.getTranslation()), apply(pose.getRotation()))
           : pose;
  }

  public static Translation3d apply(Translation3d translation)
  {
    return new Translation3d(
        applyX(translation.getX()), applyY(translation.getY()), translation.getZ());
  }

  public static Rotation3d apply(Rotation3d rotation)
  {
    return shouldFlip() ? rotation.rotateBy(new Rotation3d(0.0, 0.0, Math.PI)) : rotation;
  }

  public static Pose3d apply(Pose3d pose)
  {
    return new Pose3d(apply(pose.getTranslation()), apply(pose.getRotation()));
  }

  /**
   * Mirror a pose to the other alliance's side, whatever the current alliance is. Used for
   * PathPlanner paths, which are always drawn from the blue alliance.
   *
   * @param pose Pose, blue alliance origin.
   * @return The same spot on the red alliance's side.
   */
  public static Pose2d flip(Pose2d pose)
  {
    return new Pose2d(FieldConstants.fieldLength - pose.getX(),
                      FieldConstants.fieldWidth - pose.getY(),
                      pose.getRotation().rotateBy(Rotation2d.k180deg));
  }

  public static boolean shouldFlip()
  {
    return MatchState.getAlliance().isPresent()
           && MatchState.getAlliance().get() == Alliance.RED;
  }
}
