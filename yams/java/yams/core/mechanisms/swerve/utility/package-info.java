// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

/**
 * Swerve drive utility classes.
 *
 * <p>{@link yams.core.mechanisms.swerve.utility.SwerveInputStream} converts raw joystick axis
 * values into chassis speed vectors suitable for passing to {@link
 * yams.core.mechanisms.swerve.SwerveDrive}. It handles deadbanding, field-relative / robot-relative
 * switching, and optional heading-lock control.
 */
package yams.core.mechanisms.swerve.utility;
