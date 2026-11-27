// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package first.robot;

import static org.wpilib.units.Units.Inches;
import static org.wpilib.units.Units.MetersPerSecond;

import org.wpilib.math.geometry.Translation2d;
import org.wpilib.units.measure.LinearVelocity;

/** Constants for the swerve skid detection example. */
public final class Constants {
  private Constants() {}

  public static final class OperatorConstants {
    public static final int DRIVER_CONTROLLER_PORT = 0;
  }

  public static final class SkidConstants {
    /** Module mounting positions (+X forward, +Y left) matching SwerveModuleConfig.withLocation. */
    public static final Translation2d[] MODULE_LOCATIONS = new Translation2d[] {
      new Translation2d(Inches.of(24), Inches.of(24)),
      new Translation2d(Inches.of(24), Inches.of(-24)),
      new Translation2d(Inches.of(-24), Inches.of(24)),
      new Translation2d(Inches.of(-24), Inches.of(-24))
    };

    /** Maximum allowed max/min translational speed ratio across modules before flagging a skid. */
    public static final double SKID_RATIO_THRESHOLD = 1.35;

    /** Slowest-module translational speed deadband below which ratios are ignored. */
    public static final LinearVelocity MIN_MODULE_TRANSLATIONAL_SPEED = MetersPerSecond.of(0.20);
  }
}
