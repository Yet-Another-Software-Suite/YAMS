// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

/**
 * Swerve drive utility classes.
 *
 * <p>{@link yams.mechanisms.swerve.utility.SwerveInputStream} converts raw joystick axis values
 * into chassis speed vectors suitable for passing to
 * {@link yams.mechanisms.swerve.SwerveDrive}. It handles deadbanding, field-relative / robot-relative
 * switching, and optional heading-lock control.
 */
package yams.mechanisms.swerve.utility;
