// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package first.robot;

import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.RPM;
import static org.wpilib.units.Units.Seconds;

import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Time;

/** Constants for the shoot on the move robot. */
public final class Constants {
  private Constants() {}

  /** When a shot is ready to fire. Tighter is more accurate, and slower to fire. */
  public static final class ShotConstants {
    private ShotConstants() {}

    /** How far the robot's heading may be from the aim heading. */
    public static final Angle AIM_TOLERANCE = Degrees.of(2);
    /** How far the flywheel may be from the shot's speed. */
    public static final AngularVelocity SPEED_TOLERANCE = RPM.of(100);
    /** How long every check must hold before the feeder runs, so one lucky loop does not fire a shot. */
    public static final Time READY_DEBOUNCE = Seconds.of(0.1);
    /**
     * How far ahead to aim from: the time between deciding to shoot and the game piece leaving. Tune it by driving
     * straight across the front of the target.
     */
    public static final Time AIM_LOOKAHEAD = Seconds.of(0.1);
  }

  /** The driver's controller. */
  public static final class OperatorConstants {
    private OperatorConstants() {}

    public static final int DRIVER_CONTROLLER_PORT = 0;
  }
}
