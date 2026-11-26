// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package yams.core.diagnostics;

import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.Seconds;
import static org.wpilib.units.Units.Volts;

import java.util.Objects;
import org.wpilib.units.measure.Current;
import org.wpilib.units.measure.Time;
import org.wpilib.units.measure.Voltage;

/**
 * Real-time chassis power-health, amp-hour consumption, low-voltage alert, and smooth brownout
 * throttling monitor.
 *
 * <p>Tracks cumulative amp-hours against a configurable battery capacity (default {@code 18.0 Ah}),
 * flags total current ({@code >= 120 A}) and per-channel ({@code >= 40 A}) overcurrent alerts,
 * debounces low-voltage warnings ({@code < 12.3 V} for {@code 3.0 s}) and emergency alerts ({@code
 * < 11.8 V} for {@code 2.0 s}), counts distinct brownout transitions ({@code < 6.8 V}), and outputs
 * a smooth {@code powerThrottleScale} in {@code [minThrottleScale, 1.0]} for automatic low-voltage
 * load shedding.
 *
 * <h2>Example</h2>
 *
 * <pre>{@code
 * PowerHealthMonitor powerMonitor = new PowerHealthMonitor()
 *     .withBatteryCapacityAmpHours(18.0)
 *     .withTotalCurrentAlert(Amps.of(120))
 *     .withChannelCurrentAlert(Amps.of(40))
 *     .withThrottleVoltageRange(Volts.of(10.5), Volts.of(8.0), 0.5);
 *
 * // Inside robotPeriodic():
 * PowerHealthMonitor.PowerStatus status =
 *     powerMonitor.update(Timer.getFPGATimestamp(), busVolts, totalAmps, channelAmps);
 * double driveScale = status.powerThrottleScale();
 * }</pre>
 */
public final class PowerHealthMonitor {

  /** Default nominal FRC 12 V SLA battery capacity in amp-hours. */
  public static final double DEFAULT_BATTERY_CAPACITY_AH = 18.0;

  /** Default total PDH/PDP current alert threshold in amps. */
  public static final Current DEFAULT_TOTAL_CURRENT_ALERT = Amps.of(120.0);

  /** Default single PDH/PDP channel current alert threshold in amps. */
  public static final Current DEFAULT_CHANNEL_CURRENT_ALERT = Amps.of(40.0);

  /** Default low-voltage warning threshold. */
  public static final Voltage DEFAULT_LOW_VOLTAGE_WARN = Volts.of(12.3);

  /** Default low-voltage emergency threshold. */
  public static final Voltage DEFAULT_LOW_VOLTAGE_EMERGENCY = Volts.of(11.8);

  /** Default brownout rail threshold. */
  public static final Voltage DEFAULT_BROWNOUT_VOLTAGE = Volts.of(6.8);

  /** Default voltage at or above which full power ({@code 1.0}) is commanded. */
  public static final Voltage DEFAULT_THROTTLE_FULL_VOLTAGE = Volts.of(10.5);

  /** Default voltage at or below which {@code minThrottleScale} is commanded. */
  public static final Voltage DEFAULT_THROTTLE_MIN_VOLTAGE = Volts.of(8.0);

  /** Default minimum throttle scale applied during heavy voltage sag. */
  public static final double DEFAULT_MIN_THROTTLE_SCALE = 0.5;

  /** Default continuous duration below {@link #DEFAULT_LOW_VOLTAGE_WARN} before asserting warning. */
  public static final Time DEFAULT_LOW_VOLTAGE_WARN_DEBOUNCE = Seconds.of(3.0);

  /**
   * Default continuous duration below {@link #DEFAULT_LOW_VOLTAGE_EMERGENCY} before asserting
   * emergency.
   */
  public static final Time DEFAULT_LOW_VOLTAGE_EMERGENCY_DEBOUNCE = Seconds.of(2.0);

  /** Maximum dt clamp in seconds so pauses or breakpoints never corrupt amp-hour integration. */
  private static final double MAX_INTEGRATION_DT_SECONDS = 0.25;

  /** Number of seconds in one hour for amp-second to amp-hour conversion. */
  private static final double SECONDS_PER_HOUR = 3600.0;

  /** Full-scale percentage multiplier for battery remaining estimation. */
  private static final double PERCENT_SCALE = 100.0;

  /** Full unthrottled power scale factor. */
  private static final double FULL_THROTTLE_SCALE = 1.0;

  /**
   * Immutable snapshot of the latest power-health evaluation.
   *
   * @param busVoltageVolts Sanitized battery/bus voltage in volts.
   * @param totalCurrentAmps Sanitized total PDH/PDP current draw in amps.
   * @param maxChannelCurrentAmps Highest single-channel current draw in amps.
   * @param hottestChannelIndex Index of the highest-current channel, or {@code -1} if unavailable.
   * @param consumedAmpHours Cumulative amp-hours consumed since creation or reset.
   * @param estimatedBatteryPercent Estimated remaining battery percentage in {@code [0.0, 100.0]}.
   * @param powerThrottleScale Smooth load-shedding scale factor in {@code [minThrottleScale, 1.0]}.
   * @param totalCurrentAlert Whether total current meets or exceeds the total-current threshold.
   * @param channelCurrentAlert Whether any channel meets or exceeds the per-channel threshold.
   * @param lowVoltageWarning Whether bus voltage has stayed below the warning threshold for the
   *     configured debounce duration.
   * @param lowVoltageEmergency Whether bus voltage has stayed below the emergency threshold for the
   *     configured debounce duration.
   * @param brownoutActive Whether bus voltage is currently below the brownout cutoff.
   * @param brownoutCount Total number of distinct brownout transitions observed since reset.
   */
  public record PowerStatus(
      double busVoltageVolts,
      double totalCurrentAmps,
      double maxChannelCurrentAmps,
      int hottestChannelIndex,
      double consumedAmpHours,
      double estimatedBatteryPercent,
      double powerThrottleScale,
      boolean totalCurrentAlert,
      boolean channelCurrentAlert,
      boolean lowVoltageWarning,
      boolean lowVoltageEmergency,
      boolean brownoutActive,
      int brownoutCount) {}

  /** Nominal battery capacity in amp-hours. */
  private double batteryCapacityAmpHours = DEFAULT_BATTERY_CAPACITY_AH;

  /** Total current alert threshold in amps. */
  private double totalCurrentAlertAmps = DEFAULT_TOTAL_CURRENT_ALERT.in(Amps);

  /** Single-channel current alert threshold in amps. */
  private double channelCurrentAlertAmps = DEFAULT_CHANNEL_CURRENT_ALERT.in(Amps);

  /** Low-voltage warning threshold in volts. */
  private double lowVoltageWarnVolts = DEFAULT_LOW_VOLTAGE_WARN.in(Volts);

  /** Low-voltage emergency threshold in volts. */
  private double lowVoltageEmergencyVolts = DEFAULT_LOW_VOLTAGE_EMERGENCY.in(Volts);

  /** Continuous duration below warning voltage required to assert warning. */
  private double lowVoltageWarnDebounceSec = DEFAULT_LOW_VOLTAGE_WARN_DEBOUNCE.in(Seconds);

  /** Continuous duration below emergency voltage required to assert emergency. */
  private double lowVoltageEmergencyDebounceSec = DEFAULT_LOW_VOLTAGE_EMERGENCY_DEBOUNCE.in(Seconds);

  /** Brownout voltage cutoff in volts. */
  private double brownoutVoltageVolts = DEFAULT_BROWNOUT_VOLTAGE.in(Volts);

  /** Bus voltage at or above which {@code powerThrottleScale == 1.0}. */
  private double throttleFullVolts = DEFAULT_THROTTLE_FULL_VOLTAGE.in(Volts);

  /** Bus voltage at or below which {@code powerThrottleScale == minThrottleScale}. */
  private double throttleMinVolts = DEFAULT_THROTTLE_MIN_VOLTAGE.in(Volts);

  /** Minimum throttle scale in {@code (0.0, 1.0]}. */
  private double minThrottleScale = DEFAULT_MIN_THROTTLE_SCALE;

  /** Cumulative amp-hours consumed since creation or {@link #reset()}. */
  private double consumedAmpHours = 0.0;

  /** Last update timestamp in seconds, or {@link Double#NaN} before the first sample. */
  private double lastTimestampSec = Double.NaN;

  /** Accumulated continuous time below the warning voltage threshold. */
  private double warnElapsedSec = 0.0;

  /** Accumulated continuous time below the emergency voltage threshold. */
  private double emergencyElapsedSec = 0.0;

  /** Whether the previous update was below {@link #brownoutVoltageVolts}. */
  private boolean inBrownout = false;

  /** Number of distinct brownout entries observed since creation or {@link #reset()}. */
  private int brownoutCount = 0;

  /** Creates a power monitor with competition defaults. */
  public PowerHealthMonitor() {}

  /**
   * Sets the nominal battery capacity used for {@code estimatedBatteryPercent}.
   *
   * @param capacityAmpHours Positive battery capacity in amp-hours (default {@code 18.0}).
   * @return {@link PowerHealthMonitor} for chaining.
   */
  public PowerHealthMonitor withBatteryCapacityAmpHours(double capacityAmpHours) {
    if (!Double.isFinite(capacityAmpHours) || capacityAmpHours <= 0.0) {
      throw new IllegalArgumentException("capacityAmpHours must be finite and positive");
    }
    this.batteryCapacityAmpHours = capacityAmpHours;
    return this;
  }

  /**
   * Sets the total PDH/PDP current alert threshold.
   *
   * @param threshold Positive total current threshold.
   * @return {@link PowerHealthMonitor} for chaining.
   */
  public PowerHealthMonitor withTotalCurrentAlert(Current threshold) {
    Objects.requireNonNull(threshold, "threshold must not be null");
    double amps = threshold.in(Amps);
    if (!Double.isFinite(amps) || amps <= 0.0) {
      throw new IllegalArgumentException("totalCurrentAlert must be finite and positive");
    }
    this.totalCurrentAlertAmps = amps;
    return this;
  }

  /**
   * Sets the single-channel PDH/PDP current alert threshold.
   *
   * @param threshold Positive per-channel current threshold.
   * @return {@link PowerHealthMonitor} for chaining.
   */
  public PowerHealthMonitor withChannelCurrentAlert(Current threshold) {
    Objects.requireNonNull(threshold, "threshold must not be null");
    double amps = threshold.in(Amps);
    if (!Double.isFinite(amps) || amps <= 0.0) {
      throw new IllegalArgumentException("channelCurrentAlert must be finite and positive");
    }
    this.channelCurrentAlertAmps = amps;
    return this;
  }

  /**
   * Configures debounced low-voltage warning and emergency alert thresholds.
   *
   * @param warnVoltage Warning voltage threshold (must be {@code >= emergencyVoltage}).
   * @param emergencyVoltage Emergency voltage threshold (must be positive).
   * @param warnDebounce Continuous duration below {@code warnVoltage} required to alert.
   * @param emergencyDebounce Continuous duration below {@code emergencyVoltage} required to alert.
   * @return {@link PowerHealthMonitor} for chaining.
   */
  public PowerHealthMonitor withLowVoltageThresholds(
      Voltage warnVoltage, Voltage emergencyVoltage, Time warnDebounce, Time emergencyDebounce) {
    Objects.requireNonNull(warnVoltage, "warnVoltage must not be null");
    Objects.requireNonNull(emergencyVoltage, "emergencyVoltage must not be null");
    Objects.requireNonNull(warnDebounce, "warnDebounce must not be null");
    Objects.requireNonNull(emergencyDebounce, "emergencyDebounce must not be null");
    double warnV = warnVoltage.in(Volts);
    double emergV = emergencyVoltage.in(Volts);
    double warnSec = warnDebounce.in(Seconds);
    double emergSec = emergencyDebounce.in(Seconds);
    if (!Double.isFinite(warnV)
        || !Double.isFinite(emergV)
        || emergV <= 0.0
        || warnV < emergV) {
      throw new IllegalArgumentException(
          "Low-voltage thresholds must be finite, positive, with warnVoltage >= emergencyVoltage");
    }
    if (!Double.isFinite(warnSec)
        || !Double.isFinite(emergSec)
        || warnSec < 0.0
        || emergSec < 0.0) {
      throw new IllegalArgumentException("Debounce durations must be finite and non-negative");
    }
    this.lowVoltageWarnVolts = warnV;
    this.lowVoltageEmergencyVolts = emergV;
    this.lowVoltageWarnDebounceSec = warnSec;
    this.lowVoltageEmergencyDebounceSec = emergSec;
    return this;
  }

  /**
   * Sets the brownout voltage threshold used to count brownout transitions.
   *
   * @param brownoutVoltage Positive brownout voltage threshold.
   * @return {@link PowerHealthMonitor} for chaining.
   */
  public PowerHealthMonitor withBrownoutVoltage(Voltage brownoutVoltage) {
    Objects.requireNonNull(brownoutVoltage, "brownoutVoltage must not be null");
    double volts = brownoutVoltage.in(Volts);
    if (!Double.isFinite(volts) || volts <= 0.0) {
      throw new IllegalArgumentException("brownoutVoltage must be finite and positive");
    }
    this.brownoutVoltageVolts = volts;
    return this;
  }

  /**
   * Configures the linear low-voltage load-shedding throttle ramp.
   *
   * @param fullPowerVoltage Bus voltage at or above which {@code powerThrottleScale == 1.0}.
   * @param minPowerVoltage Bus voltage at or below which {@code powerThrottleScale == minScale}.
   * @param minScale Minimum throttle scale in {@code (0.0, 1.0]}.
   * @return {@link PowerHealthMonitor} for chaining.
   */
  public PowerHealthMonitor withThrottleVoltageRange(
      Voltage fullPowerVoltage, Voltage minPowerVoltage, double minScale) {
    Objects.requireNonNull(fullPowerVoltage, "fullPowerVoltage must not be null");
    Objects.requireNonNull(minPowerVoltage, "minPowerVoltage must not be null");
    double fullV = fullPowerVoltage.in(Volts);
    double minV = minPowerVoltage.in(Volts);
    if (!Double.isFinite(fullV)
        || !Double.isFinite(minV)
        || minV <= 0.0
        || fullV <= minV) {
      throw new IllegalArgumentException(
          "Throttle voltages must be finite, positive, with fullPowerVoltage > minPowerVoltage");
    }
    if (!Double.isFinite(minScale) || minScale <= 0.0 || minScale > 1.0) {
      throw new IllegalArgumentException("minScale must be finite and in (0.0, 1.0]");
    }
    this.throttleFullVolts = fullV;
    this.throttleMinVolts = minV;
    this.minThrottleScale = minScale;
    return this;
  }

  /**
   * Updates the power monitor without per-channel current readings.
   *
   * @param timestampSeconds Monotonic timestamp in seconds.
   * @param busVoltageVolts Measured battery/bus voltage in volts.
   * @param totalCurrentAmps Measured total PDH/PDP current in amps.
   * @return Updated {@link PowerStatus}.
   */
  public PowerStatus update(
      double timestampSeconds, double busVoltageVolts, double totalCurrentAmps) {
    return update(timestampSeconds, busVoltageVolts, totalCurrentAmps, null);
  }

  /**
   * Updates the power monitor with bus voltage, total current, and optional per-channel currents.
   *
   * @param timestampSeconds Monotonic timestamp in seconds.
   * @param busVoltageVolts Measured battery/bus voltage in volts.
   * @param totalCurrentAmps Measured total PDH/PDP current in amps.
   * @param channelCurrentsAmps Optional per-channel current array in amps (may be {@code null}).
   * @return Updated {@link PowerStatus}.
   */
  public PowerStatus update(
      double timestampSeconds,
      double busVoltageVolts,
      double totalCurrentAmps,
      double[] channelCurrentsAmps) {
    double dt = 0.0;
    if (Double.isFinite(timestampSeconds)) {
      if (!Double.isNaN(lastTimestampSec)) {
        // Clamp dt to [0, 0.25 s] so pauses/breakpoints never corrupt amp-hour integration.
        dt = Math.max(0.0, Math.min(MAX_INTEGRATION_DT_SECONDS, timestampSeconds - lastTimestampSec));
      }
      lastTimestampSec = timestampSeconds;
    }

    double safeVolts = Double.isFinite(busVoltageVolts) ? Math.max(0.0, busVoltageVolts) : 0.0;
    double safeTotalAmps = Double.isFinite(totalCurrentAmps) ? Math.max(0.0, totalCurrentAmps) : 0.0;

    // Integrate amp-hours: I (A) * dt (s) / 3600 (s/h)
    consumedAmpHours += (safeTotalAmps * dt) / SECONDS_PER_HOUR;
    double estPct =
        Math.max(
            0.0, Math.min(PERCENT_SCALE, (FULL_THROTTLE_SCALE - consumedAmpHours / batteryCapacityAmpHours) * PERCENT_SCALE));

    double maxChannelAmps = 0.0;
    int hottestChannel = -1;
    if (channelCurrentsAmps != null) {
      for (int i = 0; i < channelCurrentsAmps.length; i++) {
        double c = channelCurrentsAmps[i];
        if (Double.isFinite(c) && c > maxChannelAmps) {
          maxChannelAmps = c;
          hottestChannel = i;
        }
      }
    }

    if (safeVolts < lowVoltageWarnVolts) {
      warnElapsedSec += dt;
    } else {
      warnElapsedSec = 0.0;
    }
    if (safeVolts < lowVoltageEmergencyVolts) {
      emergencyElapsedSec += dt;
    } else {
      emergencyElapsedSec = 0.0;
    }

    boolean brownoutNow = safeVolts < brownoutVoltageVolts;
    if (brownoutNow && !inBrownout) {
      brownoutCount++;
    }
    inBrownout = brownoutNow;

    double throttle;
    if (safeVolts >= throttleFullVolts) {
      throttle = FULL_THROTTLE_SCALE;
    } else if (safeVolts <= throttleMinVolts) {
      throttle = minThrottleScale;
    } else {
      double frac = (safeVolts - throttleMinVolts) / (throttleFullVolts - throttleMinVolts);
      throttle = minThrottleScale + frac * (FULL_THROTTLE_SCALE - minThrottleScale);
    }

    return new PowerStatus(
        safeVolts,
        safeTotalAmps,
        maxChannelAmps,
        hottestChannel,
        consumedAmpHours,
        estPct,
        throttle,
        safeTotalAmps >= totalCurrentAlertAmps,
        maxChannelAmps >= channelCurrentAlertAmps,
        warnElapsedSec >= lowVoltageWarnDebounceSec,
        emergencyElapsedSec >= lowVoltageEmergencyDebounceSec,
        brownoutNow,
        brownoutCount);
  }

  /** Resets integrated amp-hours, debounce timers, and brownout counters. */
  public void reset() {
    consumedAmpHours = 0.0;
    lastTimestampSec = Double.NaN;
    warnElapsedSec = 0.0;
    emergencyElapsedSec = 0.0;
    inBrownout = false;
    brownoutCount = 0;
  }
}
