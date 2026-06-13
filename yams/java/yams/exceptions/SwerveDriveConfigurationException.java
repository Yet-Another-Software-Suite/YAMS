// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package yams.exceptions;

/**
 * Thrown by {@link yams.mechanisms.swerve.SwerveDrive} or {@link yams.mechanisms.config.SwerveDriveConfig}
 * when a required field is missing or a config invariant is violated.
 *
 * <p>Common triggers include:
 * <ul>
 *   <li>Module configurations (drive and steer motors) not provided for one or more swerve modules</li>
 *   <li>Gyro sensor not configured — required for field-relative driving and odometry</li>
 *   <li>Kinematic parameters missing (e.g., track width or wheelbase not set)</li>
 *   <li>A {@code SwerveDriveConfig} method called in an incompatible sequence or called twice</li>
 * </ul>
 *
 * <p><b>Resolution:</b> Ensure {@code SwerveDriveConfig} is fully populated with all swerve module
 * configs, a gyro source, and the required kinematic geometry before constructing
 * {@link yams.mechanisms.swerve.SwerveDrive}. Consult the {@code SwerveDriveConfig} builder
 * methods for the specific setter that the error message recommends.
 *
 * <p>Example minimal configuration:
 * <pre>{@code
 * SwerveDriveConfig config = new SwerveDriveConfig()
 *     .withModules(frontLeft, frontRight, backLeft, backRight)
 *     .withGyro(gyro)
 *     .withTrackWidth(Inches.of(22))
 *     .withWheelBase(Inches.of(22));
 * SwerveDrive drive = new SwerveDrive(config);
 * }</pre>
 *
 * @see yams.mechanisms.swerve.SwerveDrive
 * @see yams.mechanisms.config.SwerveDriveConfig
 */
public class SwerveDriveConfigurationException extends RuntimeException {
    /**
     * SwerveDrive configuration exception.
     *
     * @param message        Message to display.
     * @param result         Result of the configuration.
     * @param remedyFunction Remedy function to use.
     */
    public SwerveDriveConfigurationException(String message, String result, String remedyFunction)
    {
        super(message + "!\n" + result + "\nPlease use SwerveDriveConfig." + remedyFunction + " to fix this error.");
    }
}
