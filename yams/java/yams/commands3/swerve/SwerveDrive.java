// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package yams.commands3.swerve;

import static org.wpilib.units.Units.Radians;

import java.util.function.Supplier;
import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.tunable.Tunables;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.Distance;
import yams.commands3.config.SwerveDriveConfig;
import yams.commands3.telemetry.CommandTunable;

/**
 * Command-based extension of {@link yams.core.mechanisms.swerve.SwerveDrive} that adds the
 * {@link Mechanism} binding and {@link Command} factories.
 *
 * <p>The typical pattern is to wrap {@code SwerveDrive} inside a class implementing
 * {@link Mechanism}.
 *
 * <pre>{@code
 * public class SwerveSubsystem implements Mechanism {
 *     private final SwerveDrive drive;
 *
 *     public SwerveSubsystem() {
 *         drive = new SwerveDrive(new SwerveDriveConfig(this, modules)....);
 *     }
 *
 *     public Command driveWithJoystick(CommandXboxController controller) {
 *         return drive.drive(() -> new ChassisVelocities(...));
 *     }
 *
 *     public Command driveFieldRelative(Supplier<ChassisVelocities> speedsSupplier) {
 *         return runRepeatedly(() -> drive.setFieldRelativeChassisSpeeds(speedsSupplier.get()))
 *             .named("Drive Field Relative");
 *     }
 * }
 * }</pre>
 */
public class SwerveDrive extends yams.core.mechanisms.swerve.SwerveDrive {
  /** Mechanism the drive's commands should require. */
  private final Mechanism mechanism;

  /**
   * Construct a {@link SwerveDrive}.
   *
   * @param config {@link SwerveDriveConfig} to use, with a {@link Mechanism} set via
   *               {@code withMechanism(Mechanism)}.
   */
  public SwerveDrive(SwerveDriveConfig config) {
    super(config);
    this.mechanism = config.getMechanism();
    // Drive to pose tuning needs both controllers, so only offer it when they are configured.
    if (config.getTranslationPID().isPresent() && config.getRotationPID().isPresent()) {
      Tunables.publish("Mechanisms/" + getName() + "/tuning/driveToPose", new CommandTunable(Command.noRequirements(coroutine -> {
        startDriveToPoseTuning();
        while (true) {
          applyDriveToPoseTuningValues();
          coroutine.yield();
        }
      }).named(getName() + " DriveToPoseTuning")));
    }
  }

  /**
   * Mechanism the drive's commands require.
   *
   * @return {@link Mechanism} for the drive.
   */
  public Mechanism getMechanism() {
    return mechanism;
  }

  /**
   * Create a {@link Command} to drive the swerve drive with robot relative chassis speeds.
   *
   * @param robotRelativeChassisSpeeds {@link Supplier} of {@link ChassisVelocities} for the robot
   *                                   relative chassis speeds. Could also use {@link
   *                                   yams.core.mechanisms.swerve.utility.SwerveInputStream}
   * @return {@link Command} to drive the swerve drive.
   * @implNote Not compatible with AdvantageKit
   */
  public Command drive(Supplier<ChassisVelocities> robotRelativeChassisSpeeds) {
    return mechanism.runRepeatedly(() -> setRobotRelativeChassisSpeeds(robotRelativeChassisSpeeds.get())).named("Drive");
  }

  /**
   * Drive the robot to the given pose. The command runs until it is canceled and stops the drive
   * when it is.
   *
   * @param pose {@link Pose2d} to drive the robot to. Field relative, blue-origin where 0deg is
   *             facing towards RED
   * @return {@link Command} to drive the robot to the given pose.
   * @implNote Not compatible with AdvantageKit
   */
  public Command driveToPose(Pose2d pose) {
    return mechanism.run(coroutine -> {
      startDriveToPoseTuning();
      while (true) {
        // driveToPoseSetpoint already returns robot relative speeds.
        setRobotRelativeChassisSpeeds(driveToPoseSetpoint(pose));
        coroutine.yield();
      }
    }).whenCanceled(this::stop).named("Drive to Pose");
  }

  /**
   * Drive the robot to the given pose, ending once the robot is within the given tolerances. The
   * drive is stopped when the command ends or is canceled.
   *
   * @param pose                 {@link Pose2d} to drive the robot to. Field relative, blue-origin
   *                             where 0deg is facing towards RED
   * @param translationTolerance Maximum distance from the pose to be considered at the pose.
   * @param rotationTolerance    Maximum heading error from the pose to be considered at the pose.
   * @return {@link Command} to drive the robot to the given pose.
   * @implNote Not compatible with AdvantageKit
   */
  public Command driveToPose(Pose2d pose, Distance translationTolerance, Angle rotationTolerance) {
    return mechanism.run(coroutine -> {
      startDriveToPoseTuning();
      while (!isNear(pose, translationTolerance, rotationTolerance)) {
        // driveToPoseSetpoint already returns robot relative speeds.
        setRobotRelativeChassisSpeeds(driveToPoseSetpoint(pose));
        coroutine.yield();
      }
      stop();
    }).whenCanceled(this::stop).named("Drive to Pose");
  }

  /**
   * Whether the robot is within the given tolerances of a pose.
   *
   * @param pose                 {@link Pose2d} to compare against.
   * @param translationTolerance Maximum distance from the pose.
   * @param rotationTolerance    Maximum heading error from the pose.
   * @return True if the robot is within both tolerances of the pose.
   */
  public boolean isNear(Pose2d pose, Distance translationTolerance, Angle rotationTolerance) {
    return getDistanceFromPose(pose).lte(translationTolerance) &&
           Math.abs(getAngleDifferenceFromPose(pose).in(Radians)) <= rotationTolerance.in(Radians);
  }

  /** Stop the drive by commanding zero chassis speeds. */
  public void stop() {
    setRobotRelativeChassisSpeeds(new ChassisVelocities());
  }
}
