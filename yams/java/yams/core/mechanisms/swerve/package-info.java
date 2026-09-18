// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

/**
 * Swerve drive mechanism classes.
 *
 * <p>{@link yams.core.mechanisms.swerve.SwerveDrive} is a full swerve-drive implementation with
 * integrated kinematics, odometry, and pose estimation. Each wheel is represented by a {@link
 * yams.core.mechanisms.swerve.SwerveModule} backed by two {@link
 * yams.core.motorcontrollers.SmartMotorController} instances (drive and steer).
 *
 * <p>Configure the full drivetrain via {@link yams.core.mechanisms.config.SwerveDriveConfig} and
 * each module via {@link yams.core.mechanisms.config.SwerveModuleConfig}.
 *
 * @see yams.core.mechanisms.swerve.utility
 */
package yams.core.mechanisms.swerve;
