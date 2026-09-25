// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

/**
 * Commands v3 swerve drive.
 *
 * <p>{@link yams.commands3.swerve.SwerveDrive} extends the core
 * {@link yams.core.mechanisms.swerve.SwerveDrive} (kinematics, odometry, pose estimation and
 * telemetry) with {@link org.wpilib.command3.Command} factories that require the
 * {@link org.wpilib.command3.Mechanism} set in its {@link yams.commands3.config.SwerveDriveConfig}.
 * Each wheel is a {@link yams.core.mechanisms.swerve.SwerveModule} backed by two
 * {@link yams.core.motorcontrollers.SmartMotorController}s (drive and steer), configured with
 * {@link yams.core.mechanisms.config.SwerveModuleConfig}.
 *
 * <pre>{@code
 * public class DriveMechanism implements Mechanism {
 *   private final SwerveDrive drive = new SwerveDrive((SwerveDriveConfig)
 *       new SwerveDriveConfig(this, frontLeft, frontRight, backLeft, backRight)
 *           .withTranslationController(new PIDController(1, 0, 0))
 *           .withRotationController(new PIDController(1, 0, 0)));
 *
 *   public Command driveTo(Pose2d pose) {
 *     return drive.driveToPose(pose, Centimeters.of(5), Degrees.of(3));
 *   }
 * }
 * }</pre>
 *
 * @see yams.core.mechanisms.swerve
 * @see yams.core.mechanisms.swerve.utility.SwerveInputStream
 */
package yams.commands3.swerve;
