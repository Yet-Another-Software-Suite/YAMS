// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package first.robot;

import static org.wpilib.units.Units.Seconds;

import org.wpilib.units.measure.Time;

/** Shared constants for the collision and jerk detection example robot. */
public final class Constants {
  private Constants() {}

  public static final class OperatorConstants {
    public static final int DRIVER_CONTROLLER_PORT = 0;

    private OperatorConstants() {}
  }

  public static final class CollisionConstants {
    public static final double JERK_THRESHOLD_GS_PER_SECOND = 20.0;
    public static final double MIN_IMPACT_LINEAR_ACCEL_GS = 0.50;
    public static final Time BUMP_LATCH_DURATION = Seconds.of(0.60);
    public static final Time COLLISION_COOLDOWN = Seconds.of(0.25);

    public static final CollisionJerkMonitor.Config MONITOR_CONFIG =
        new CollisionJerkMonitor.Config(
            JERK_THRESHOLD_GS_PER_SECOND,
            MIN_IMPACT_LINEAR_ACCEL_GS,
            BUMP_LATCH_DURATION,
            COLLISION_COOLDOWN);

    private CollisionConstants() {}
  }
}
