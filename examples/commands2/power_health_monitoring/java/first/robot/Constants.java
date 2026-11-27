// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package first.robot;

import static org.wpilib.units.Units.MilliOhms;
import static org.wpilib.units.Units.Seconds;
import static org.wpilib.units.Units.Volts;

import org.wpilib.units.measure.Resistance;
import org.wpilib.units.measure.Time;
import org.wpilib.units.measure.Voltage;

/** Shared constants for the power health monitoring example robot. */
public final class Constants {
  private Constants() {}

  public static final class OperatorConstants {
    public static final int DRIVER_CONTROLLER_PORT = 0;

    private OperatorConstants() {}
  }

  public static final class PowerHealthConstants {
    public static final Voltage WARNING_VOLTAGE = Volts.of(10.5);
    public static final Voltage EMERGENCY_VOLTAGE = Volts.of(9.5);
    /** Match whatever brownout rail you configure via RobotController.setBrownoutVoltages(...). */
    public static final Voltage BROWNOUT_RAIL_VOLTAGE = Volts.of(6.75);
    public static final Voltage HYSTERESIS_VOLTAGE = Volts.of(0.30);

    public static final Time WARNING_DEBOUNCE = Seconds.of(0.25);
    public static final Time EMERGENCY_DEBOUNCE = Seconds.of(0.15);

    public static final Voltage THROTTLE_START_VOLTAGE = Volts.of(11.0);
    public static final Voltage THROTTLE_FLOOR_VOLTAGE = Volts.of(9.0);
    public static final double MIN_THROTTLE_SCALE = 0.35;

    public static final double BATTERY_CAPACITY_AH = 18.0;
    public static final double INITIAL_STATE_OF_CHARGE = 1.0;
    public static final Voltage SIM_BATTERY_FULL_VOLTAGE = Volts.of(12.9);
    public static final Resistance SIM_BATTERY_INTERNAL_RESISTANCE = MilliOhms.of(20);

    public static final PowerHealthMonitor.Config MONITOR_CONFIG =
        new PowerHealthMonitor.Config(
            WARNING_VOLTAGE,
            EMERGENCY_VOLTAGE,
            BROWNOUT_RAIL_VOLTAGE,
            HYSTERESIS_VOLTAGE,
            WARNING_DEBOUNCE,
            EMERGENCY_DEBOUNCE,
            THROTTLE_START_VOLTAGE,
            THROTTLE_FLOOR_VOLTAGE,
            MIN_THROTTLE_SCALE,
            BATTERY_CAPACITY_AH,
            INITIAL_STATE_OF_CHARGE);

    private PowerHealthConstants() {}
  }
}
