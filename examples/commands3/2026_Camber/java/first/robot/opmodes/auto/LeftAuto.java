// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later
// Ported from Team 9658's 2026-KitBot (https://github.com/9658-Camber-Robotics/2026-KitBot).

package first.robot.opmodes.auto;

import static first.robot.opmodes.auto.AutoSteps.pose;

import first.robot.Robot;
import org.wpilib.command3.Command;
import org.wpilib.command3.button.RobotModeTriggers;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.opmode.Autonomous;
import org.wpilib.opmode.OpMode;

/**
 * Shoot the preload, drive along the left trench to the middle of the field, intake down the center line, then come
 * back through the trench and shoot again. The original PathPlanner "Left Auto".
 */
@Autonomous(name = "Left Auto")
public class LeftAuto implements OpMode
{

  private static final Pose2d kStart         = pose(3.522, 7.38, 0);
  private static final Pose2d kShot          = pose(3.03, 4.779, -30.53);
  private static final Pose2d kTrench        = pose(3.56, 7.431, 0);
  private static final Pose2d kMiddle        = pose(7.727, 6.979, 90);
  private static final Pose2d kMiddleIntake  = pose(7.752, 4.443, 89.14);

  /**
   * Creates the autonomous opmode. The OpModeRobot framework calls this when the opmode is selected on the driver
   * station.
   *
   * @param robot The robot instance to control.
   */
  public LeftAuto(Robot robot)
  {
    // The auto starts when autonomous is enabled and is canceled when it is disabled.
    RobotModeTriggers.autonomous().whileTrue(leftAuto(robot));
  }

  private static Command leftAuto(Robot robot)
  {
    return Command.noRequirements(coroutine -> {
      coroutine.await(AutoSteps.resetPose(robot, kStart));
      // Left Shot
      coroutine.await(AutoSteps.drive(robot, kShot, true));
      coroutine.await(AutoSteps.shootBalls(robot));
      // Left Trench, Left Middle
      coroutine.await(AutoSteps.drive(robot, kTrench, false));
      coroutine.await(AutoSteps.drive(robot, kMiddle, false));
      // Left Middle intake, then back to Left Middle intaking over the first third
      coroutine.await(AutoSteps.driveIntaking(robot, kMiddle, kMiddleIntake, false, 0, 1));
      coroutine.await(AutoSteps.driveIntaking(robot, kMiddleIntake, kMiddle, false, 0, 0.325));
      // Left Middle to Left Trench, Left Trench to Left Shot
      coroutine.await(AutoSteps.drive(robot, kTrench, false));
      coroutine.await(AutoSteps.drive(robot, kShot, true));
      coroutine.await(AutoSteps.shootBalls(robot));
    }).named("Left Auto");
  }
}
