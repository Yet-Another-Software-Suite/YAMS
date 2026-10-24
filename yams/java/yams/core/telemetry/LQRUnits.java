// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package yams.core.telemetry;

import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.RPM;
import static org.wpilib.units.Units.Radians;
import static org.wpilib.units.Units.RadiansPerSecond;

import yams.core.math.LQRConfig.LQRType;

/**
 * Converts the LQR's tolerances and standard deviations between the SI units {@link yams.core.math.LQRConfig} keeps
 * them in and the units the LQR telemetry fields use, which match the tunable setpoints: degrees and RPM for an arm or
 * flywheel, meters and meters per second (unchanged) for an elevator.
 */
final class LQRUnits {
  private LQRUnits() {}

  /**
   * From the telemetry field's units to SI.
   *
   * @param type     {@link LQRType} of the LQR.
   * @param position True for a position (degrees), false for a velocity (RPM).
   * @param value    Value in the telemetry field's units.
   * @return Value in SI units.
   */
  static double toSI(LQRType type, boolean position, double value) {
    if (type == LQRType.ELEVATOR) {
      return value;
    }
    return position ? Degrees.of(value).in(Radians) : RPM.of(value).in(RadiansPerSecond);
  }

  /**
   * From SI to the telemetry field's units.
   *
   * @param type     {@link LQRType} of the LQR.
   * @param position True for a position (radians), false for a velocity (rad/s).
   * @param value    Value in SI units.
   * @return Value in the telemetry field's units.
   */
  static double fromSI(LQRType type, boolean position, double value) {
    if (type == LQRType.ELEVATOR) {
      return value;
    }
    return position ? Radians.of(value).in(Degrees) : RadiansPerSecond.of(value).in(RPM);
  }
}
