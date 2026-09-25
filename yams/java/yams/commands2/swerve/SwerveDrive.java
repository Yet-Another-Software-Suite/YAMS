// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package yams.commands2.swerve;

import java.util.function.Supplier;
import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import org.wpilib.command2.Subsystem;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.tunable.Tunables;
import yams.commands2.config.SwerveDriveConfig;

/**
 * Command-based extension of {@link yams.core.mechanisms.swerve.SwerveDrive} that adds the
 * {@link Subsystem} binding and {@link Command} factories.
 *
 * <p>The typical pattern is to wrap {@code SwerveDrive} inside a WPILib {@code SubsystemBase}.
 *
 * <pre>{@code
 * public class SwerveSubsystem extends SubsystemBase {
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
 *         return run(() -> drive.setFieldRelativeChassisSpeeds(speedsSupplier.get()))
 *             .withName("Drive Field Relative");
 *     }
 * }
 * }</pre>
 */
public class SwerveDrive extends yams.core.mechanisms.swerve.SwerveDrive {
  /** Subsystem the drive's commands should require. */
  private final Subsystem subsystem;

  /**
   * Construct a {@link SwerveDrive}.
   *
   * @param config {@link SwerveDriveConfig} to use, with a {@link Subsystem} set via
   *               {@code withSubsystem(Subsystem)}.
   */
  public SwerveDrive(SwerveDriveConfig config) {
    super(config);
    this.subsystem = config.getSubsystem();
    // Drive to pose tuning needs both controllers, so only offer it when they are configured.
    if (config.getTranslationPID().isPresent() && config.getRotationPID().isPresent()) {
      Tunables.publish("Mechanisms/" + getName() + "/tuning/driveToPose", Commands.startRun(this::startDriveToPoseTuning, this::applyDriveToPoseTuningValues));
    }
  }

  /**
   * Subsystem the drive's commands require.
   *
   * @return {@link Subsystem} for the drive.
   */
  public Subsystem getSubsystem() {
    return subsystem;
  }

  /**
   * Create a {@link Command} to drive the swerve drive with robot relative chassis speeds.
   *
   * @param robotRelativeChassisSpeeds {@link Supplier} of {@link ChassisVelocities} for the robot
   *                                   relative chassis speeds. Could also use {@link
   *                                   yams.commands2.swerve.SwerveInputStream}
   * @return {@link Command} to drive the swerve drive.
   * @implNote Not compatible with AdvantageKit
   */
  public Command drive(Supplier<ChassisVelocities> robotRelativeChassisSpeeds) {
    return Commands.run(() -> setRobotRelativeChassisSpeeds(robotRelativeChassisSpeeds.get()), subsystem).withName("Drive");
  }

  /**
   * Drive the robot to the given pose.
   *
   * @param pose {@link Pose2d} to drive the robot to. Field relative, blue-origin where 0deg is
   *             facing towards RED
   * @return {@link Command} that drives the robot toward the given pose until interrupted.
   * @implNote Not compatible with AdvantageKit
   */
  public Command driveToPose(Pose2d pose) {
    // driveToPoseSetpoint already returns robot relative speeds.
    return Commands.startRun(this::startDriveToPoseTuning, () -> setRobotRelativeChassisSpeeds(driveToPoseSetpoint(pose)), subsystem).withName("Drive to Pose");
  }
}
