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
 * Shoot the preload, drive along the right trench to the middle of the field, intake up the center line, then come
 * back through the trench and shoot again. The original PathPlanner "Right Auto".
 */
@Autonomous(name = "Right Auto")
public class RightAuto implements OpMode
{

  private static final Pose2d kStart        = pose(3.56, 0.677, 0);
  private static final Pose2d kShot         = pose(2.965, 3.369, 18.08);
  private static final Pose2d kTrench       = pose(3.716, 0.548, 0);
  private static final Pose2d kMiddle       = pose(6.808, 0.677, -91.43);
  private static final Pose2d kMiddleIntake = pose(7.791, 3.602, -88.28);

  /**
   * Creates the autonomous opmode. The OpModeRobot framework calls this when the opmode is selected on the driver
   * station.
   *
   * @param robot The robot instance to control.
   */
  public RightAuto(Robot robot)
  {
    // The auto starts when autonomous is enabled and is canceled when it is disabled.
    RobotModeTriggers.autonomous().whileTrue(rightAuto(robot));
  }

  private static Command rightAuto(Robot robot)
  {
    return Command.noRequirements(coroutine -> {
      coroutine.await(AutoSteps.resetPose(robot, kStart));
      // Right Shot
      coroutine.await(AutoSteps.drive(robot, kShot, true));
      coroutine.await(AutoSteps.shootBalls(robot));
      // Right Trench, Right Middle
      coroutine.await(AutoSteps.drive(robot, kTrench, false));
      coroutine.await(AutoSteps.drive(robot, kMiddle, false));
      // Right Middle Intake from about a fifth of the way, then back to Right Middle intaking over the first third
      coroutine.await(AutoSteps.driveIntaking(robot, kMiddle, kMiddleIntake, false, 0.224, 1));
      coroutine.await(AutoSteps.driveIntaking(robot, kMiddleIntake, kMiddle, false, 0, 0.305));
      // Right Middle to Right Trench, Right Trench to Right Shot
      coroutine.await(AutoSteps.drive(robot, kTrench, false));
      coroutine.await(AutoSteps.drive(robot, kShot, true));
      coroutine.await(AutoSteps.shootBalls(robot));
    }).named("Right Auto");
  }
}
