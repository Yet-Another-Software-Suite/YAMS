// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later
// Ported from Team 9658's 2026-KitBot (https://github.com/9658-Camber-Robotics/2026-KitBot).

package first.robot.opmodes.auto;

import static first.robot.opmodes.auto.AutoSteps.pose;
import static org.wpilib.units.Units.Seconds;

import first.robot.Robot;
import org.wpilib.command3.Command;
import org.wpilib.command3.button.RobotModeTriggers;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.opmode.Autonomous;
import org.wpilib.opmode.OpMode;

/**
 * Back up and shoot the preload, intake from the depot, then return and shoot again. The original PathPlanner "Middle
 * Auto".
 */
@Autonomous(name = "Middle Auto")
public class MiddleAuto implements OpMode
{

  private static final Pose2d kStart = pose(3.516, 3.946, 1.19);
  private static final Pose2d kShot  = pose(2.888, 3.946, 1.27);
  private static final Pose2d kDepot = pose(0.496, 5.979, 0);

  /**
   * Creates the autonomous opmode. The OpModeRobot framework calls this when the opmode is selected on the driver
   * station.
   *
   * @param robot The robot instance to control.
   */
  public MiddleAuto(Robot robot)
  {
    // The auto starts when autonomous is enabled and is canceled when it is disabled.
    RobotModeTriggers.autonomous().whileTrue(middleAuto(robot));
  }

  private static Command middleAuto(Robot robot)
  {
    return Command.noRequirements(coroutine -> {
      coroutine.await(AutoSteps.resetPose(robot, kStart));
      // Back
      coroutine.await(AutoSteps.drive(robot, kShot, true));
      coroutine.await(AutoSteps.shootBalls(robot));
      // Depot, intaking over the last third, then shoot A, intaking over the first third
      coroutine.await(AutoSteps.driveIntaking(robot, kShot, kDepot, false, 0.646, 1));
      coroutine.await(AutoSteps.driveIntaking(robot, kDepot, kShot, true, 0, 0.378));
      coroutine.await(AutoSteps.shootBalls(robot));
      coroutine.await(robot.shooterCommands.stopCommand());
      coroutine.wait(Seconds.of(3));
    }).named("Middle Auto");
  }
}
