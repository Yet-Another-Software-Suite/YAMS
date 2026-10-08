// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later
// Ported from Team 9658's 2026-KitBot (https://github.com/9658-Camber-Robotics/2026-KitBot).

package first.robot.opmodes.auto;

import static org.wpilib.units.Units.Seconds;

import first.robot.Constants.Shooter.Setpoints;
import first.robot.Robot;
import first.robot.utils.AllianceFlipUtil;
import java.util.function.DoubleSupplier;
import org.wpilib.command3.Command;
import org.wpilib.command3.button.RobotModeTriggers;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
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
      robot.drivebase.resetPose(AllianceFlipUtil.apply(kStart));
      // Back
      coroutine.await(robot.drivebase.driveToPose(kShot));
      coroutine.await(shootBalls(robot));
      // Depot, intaking over the last third, then shoot A, intaking over the first third
      coroutine.await(driveIntaking(robot, kShot, kDepot, false, 0.646, 1));
      coroutine.await(driveIntaking(robot, kDepot, kShot, true, 0, 0.378));
      coroutine.await(shootBalls(robot));
      coroutine.await(robot.shooterCommands.stopCommand());
      coroutine.wait(Seconds.of(3));
    }).named("Middle Auto");
  }

  /**
   * Drive to the end of a path, intaking over part of it. Replaces the path's {@code StartIntake} event marker, which
   * ran the intake from one position along the path to another.
   *
   * @param robot       The robot.
   * @param start       Blue-origin start of the path.
   * @param end         Blue-origin end of the path.
   * @param stop        True to stop at the end, false to drive on into the next path once near it.
   * @param intakeFrom  Fraction of the path, by distance to the end, at which the intake starts.
   * @param intakeUntil Fraction of the path at which the intake stops. The intake always stops at the end of the path.
   * @return {@link Command} that ends at the end of the path.
   */
  private static Command driveIntaking(Robot robot, Pose2d start, Pose2d end, boolean stop, double intakeFrom,
                                       double intakeUntil)
  {
    return Command.noRequirements(coroutine -> {
      Translation2d flippedEnd = AllianceFlipUtil.apply(end).getTranslation();
      double length = AllianceFlipUtil.apply(start).getTranslation().getDistance(flippedEnd);
      DoubleSupplier progress = () -> 1 - robot.drivebase.getPose().getTranslation().getDistance(flippedEnd) / length;
      // Forked, so it is canceled when the drive reaches the end of the path at the latest.
      coroutine.fork(Command.noRequirements(intake -> {
        intake.waitUntil(() -> progress.getAsDouble() >= intakeFrom);
        intake.awaitAny(robot.shooterCommands.intake(),
                        Command.waitUntil(() -> progress.getAsDouble() >= intakeUntil).named("Intake Zone End"));
      }).named("Intake Zone"));
      coroutine.await(stop ? robot.drivebase.driveToPose(end) : robot.drivebase.driveThroughPose(end));
    }).named("Drive To " + end + " Intaking");
  }

  /**
   * Shoot for four seconds at the autonomous RPM, the original {@code ShootBalls} named command.
   *
   * @param robot The robot.
   * @return {@link Command} that shoots and ends.
   */
  private static Command shootBalls(Robot robot)
  {
    return robot.shooterCommands.shootAndIndex(Setpoints.autonomousPeriodRPM).withTimeout(Seconds.of(4));
  }

  /**
   * A blue-origin pose.
   *
   * @param x       X in meters.
   * @param y       Y in meters.
   * @param degrees Heading in degrees.
   * @return The {@link Pose2d}.
   */
  private static Pose2d pose(double x, double y, double degrees)
  {
    return new Pose2d(x, y, Rotation2d.fromDegrees(degrees));
  }
}
