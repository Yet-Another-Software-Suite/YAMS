// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later
// Ported from Team 9658's 2026-KitBot (https://github.com/9658-Camber-Robotics/2026-KitBot).

package first.robot.opmodes.auto;

import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.Seconds;

import first.robot.Constants.Shooter.Setpoints;
import first.robot.Robot;
import first.robot.utils.AllianceFlipUtil;
import java.util.function.DoubleSupplier;
import org.wpilib.command3.Command;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.Distance;

/**
 * Steps shared by the autos, which replace the original PathPlanner autos. Each PathPlanner path was a single segment
 * from one waypoint to the next, so each becomes one drive to pose to the path's end. Poses are blue-origin and are
 * flipped for the red alliance when the step runs.
 */
final class AutoSteps
{

  /** Tolerances for a path end the robot drives straight on from, into the next path. */
  private static final Distance kPassThroughTranslationTolerance = Meters.of(0.3);
  private static final Angle    kPassThroughRotationTolerance    = Degrees.of(15);
  /** Tolerances for a path end the robot stops at, to shoot or finish. */
  private static final Distance kStopTranslationTolerance        = Meters.of(0.05);
  private static final Angle    kStopRotationTolerance           = Degrees.of(3);

  private AutoSteps()
  {
    throw new UnsupportedOperationException("This is a utility class!");
  }

  /**
   * A blue-origin pose.
   *
   * @param x       X in meters.
   * @param y       Y in meters.
   * @param degrees Heading in degrees.
   * @return The {@link Pose2d}.
   */
  static Pose2d pose(double x, double y, double degrees)
  {
    return new Pose2d(x, y, Rotation2d.fromDegrees(degrees));
  }

  /**
   * Reset odometry to the auto's starting pose, as the PathPlanner autos did.
   *
   * @param robot The robot.
   * @param start Blue-origin starting pose.
   * @return {@link Command} that resets odometry and ends.
   */
  static Command resetPose(Robot robot, Pose2d start)
  {
    return Command.noRequirements(coroutine -> robot.drivebase.resetPose(AllianceFlipUtil.apply(start)))
                  .named("Reset Pose");
  }

  /**
   * Drive to the end of a path.
   *
   * @param robot The robot.
   * @param end   Blue-origin end of the path.
   * @param stop  True to stop at the end, false to drive on into the next path once near it.
   * @return {@link Command} that ends at the end of the path.
   */
  static Command drive(Robot robot, Pose2d end, boolean stop)
  {
    return Command.noRequirements(coroutine -> coroutine.await(driveToPose(robot, AllianceFlipUtil.apply(end), stop)))
                  .named("Drive To " + end);
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
  static Command driveIntaking(Robot robot, Pose2d start, Pose2d end, boolean stop, double intakeFrom,
                               double intakeUntil)
  {
    return Command.noRequirements(coroutine -> {
      Pose2d flippedEnd = AllianceFlipUtil.apply(end);
      double length = AllianceFlipUtil.apply(start).getTranslation().getDistance(flippedEnd.getTranslation());
      DoubleSupplier progress = () -> 1 - robot.drivebase.getPose().getTranslation()
                                                        .getDistance(flippedEnd.getTranslation()) / length;
      // Forked, so it is canceled when the drive reaches the end of the path at the latest.
      coroutine.fork(Command.noRequirements(intake -> {
        intake.waitUntil(() -> progress.getAsDouble() >= intakeFrom);
        intake.awaitAny(robot.shooterCommands.intake(),
                        Command.waitUntil(() -> progress.getAsDouble() >= intakeUntil).named("Intake Zone End"));
      }).named("Intake Zone"));
      coroutine.await(driveToPose(robot, flippedEnd, stop));
    }).named("Drive To " + end + " Intaking");
  }

  /**
   * Shoot for four seconds at the autonomous RPM, the original {@code ShootBalls} named command.
   *
   * @param robot The robot.
   * @return {@link Command} that shoots and ends.
   */
  static Command shootBalls(Robot robot)
  {
    return robot.shooterCommands.shootAndIndex(Setpoints.autonomousPeriodRPM).withTimeout(Seconds.of(4));
  }

  private static Command driveToPose(Robot robot, Pose2d pose, boolean stop)
  {
    return stop
           ? robot.drivebase.driveToPose(pose, kStopTranslationTolerance, kStopRotationTolerance)
           : robot.drivebase.driveToPose(pose, kPassThroughTranslationTolerance, kPassThroughRotationTolerance);
  }
}
