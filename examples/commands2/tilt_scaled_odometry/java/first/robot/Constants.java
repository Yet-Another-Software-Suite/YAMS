// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package first.robot;

import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Inches;

import org.wpilib.math.geometry.Translation2d;
import org.wpilib.units.measure.Angle;

/** Constants for the tilt-scaled odometry example. */
public final class Constants {
  private Constants() {}

  public static final class OperatorConstants {
    public static final int DRIVER_CONTROLLER_PORT = 0;
  }

  public static final class TiltConstants {
    public static final Translation2d[] MODULE_LOCATIONS = new Translation2d[] {
      new Translation2d(Inches.of(24), Inches.of(24)),
      new Translation2d(Inches.of(24), Inches.of(-24)),
      new Translation2d(Inches.of(-24), Inches.of(24)),
      new Translation2d(Inches.of(-24), Inches.of(-24))
    };

    /** Geometric cos(tilt) preserves true flat travel on ramps; switch to LINEAR_TRUST_RAMP for abrupt bumps. */
    public static final TiltOdometryScaler.ScalingMode SCALING_MODE =
        TiltOdometryScaler.ScalingMode.COSINE_PROJECTION;

    public static final Angle FULL_TRUST_TILT = Degrees.of(2.0);
    public static final Angle ZERO_TRUST_TILT = Degrees.of(25.0);
    public static final Angle MAX_VISION_TILT = Degrees.of(10.0);
  }
}
