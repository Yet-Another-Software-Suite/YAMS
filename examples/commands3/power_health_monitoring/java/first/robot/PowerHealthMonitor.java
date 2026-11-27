// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package first.robot;

import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.Seconds;
import static org.wpilib.units.Units.Volts;

import java.util.Objects;
import org.wpilib.units.measure.Current;
import org.wpilib.units.measure.Time;
import org.wpilib.units.measure.Voltage;

/**
 * Tracks robot battery health with non-finite sample rejection, Schmitt-trigger hysteresis on
 * warning/emergency/brownout transitions, explicit-initial-SoC coulomb counting, and smooth
 * proportional throttle scaling.
 */
public class PowerHealthMonitor {
  private static final double FULL_THROTTLE_SCALE = 1.0;
  private static final double MIN_SOC = 0.0;
  private static final double MAX_SOC = 1.0;
  private static final double SECONDS_PER_HOUR = 3600.0;

  /** High-level debounced power rail state. */
  public enum PowerState {
    NOMINAL,
    LOW_VOLTAGE_WARNING,
    LOW_VOLTAGE_EMERGENCY,
    BROWNOUT
  }

  /** Immutable configuration for {@link PowerHealthMonitor}. */
  public record Config(
      Voltage warningVoltage,
      Voltage emergencyVoltage,
      Voltage brownoutRailVoltage,
      Voltage hysteresisVoltage,
      Time warningDebounce,
      Time emergencyDebounce,
      Voltage throttleStartVoltage,
      Voltage throttleFloorVoltage,
      double minThrottleScale,
      double batteryCapacityAmpHours,
      double initialStateOfCharge) {
    public Config {
      Objects.requireNonNull(warningVoltage, "warningVoltage must not be null");
      Objects.requireNonNull(emergencyVoltage, "emergencyVoltage must not be null");
      Objects.requireNonNull(brownoutRailVoltage, "brownoutRailVoltage must not be null");
      Objects.requireNonNull(hysteresisVoltage, "hysteresisVoltage must not be null");
      Objects.requireNonNull(warningDebounce, "warningDebounce must not be null");
      Objects.requireNonNull(emergencyDebounce, "emergencyDebounce must not be null");
      Objects.requireNonNull(throttleStartVoltage, "throttleStartVoltage must not be null");
      Objects.requireNonNull(throttleFloorVoltage, "throttleFloorVoltage must not be null");

      if (hysteresisVoltage.in(Volts) < 0.0) {
        throw new IllegalArgumentException("hysteresisVoltage must be non-negative");
      }
      if (warningDebounce.in(Seconds) < 0.0 || emergencyDebounce.in(Seconds) < 0.0) {
        throw new IllegalArgumentException("debounce durations must be non-negative");
      }
      if (throttleStartVoltage.in(Volts) <= throttleFloorVoltage.in(Volts)) {
        throw new IllegalArgumentException("throttleStartVoltage must exceed throttleFloorVoltage");
      }
      if (!Double.isFinite(minThrottleScale)
          || minThrottleScale < 0.0
          || minThrottleScale > FULL_THROTTLE_SCALE) {
        throw new IllegalArgumentException("minThrottleScale must be within [0.0, 1.0]");
      }
      if (!Double.isFinite(batteryCapacityAmpHours) || batteryCapacityAmpHours <= 0.0) {
        throw new IllegalArgumentException("batteryCapacityAmpHours must be positive");
      }
      if (!Double.isFinite(initialStateOfCharge)
          || initialStateOfCharge < MIN_SOC
          || initialStateOfCharge > MAX_SOC) {
        throw new IllegalArgumentException("initialStateOfCharge must be within [0.0, 1.0]");
      }
    }
  }

  private final double warningVolts;
  private final double emergencyVolts;
  private final double brownoutRailVolts;
  private final double hysteresisVolts;
  private final double warningDebounceSeconds;
  private final double emergencyDebounceSeconds;
  private final double throttleStartVolts;
  private final double throttleFloorVolts;
  private final double minThrottleScale;
  private final double batteryCapacityAmpHours;
  private final double initialStateOfCharge;

  private boolean hasLastTimestamp;
  private double lastTimestampSeconds;
  private double lastValidVoltageVolts;
  private double lastValidCurrentAmps;
  private double consumedAmpHours;

  private double warningTimerSeconds;
  private double emergencyTimerSeconds;
  private boolean warningActive;
  private boolean emergencyActive;
  private boolean inBrownout;
  private int brownoutCount;

  private double throttleScale = FULL_THROTTLE_SCALE;
  private PowerState currentState = PowerState.NOMINAL;

  /** Creates a new PowerHealthMonitor from the supplied configuration. */
  public PowerHealthMonitor(Config config) {
    Objects.requireNonNull(config, "config must not be null");
    this.warningVolts = config.warningVoltage().in(Volts);
    this.emergencyVolts = config.emergencyVoltage().in(Volts);
    this.brownoutRailVolts = config.brownoutRailVoltage().in(Volts);
    this.hysteresisVolts = config.hysteresisVoltage().in(Volts);
    this.warningDebounceSeconds = config.warningDebounce().in(Seconds);
    this.emergencyDebounceSeconds = config.emergencyDebounce().in(Seconds);
    this.throttleStartVolts = config.throttleStartVoltage().in(Volts);
    this.throttleFloorVolts = config.throttleFloorVoltage().in(Volts);
    this.minThrottleScale = config.minThrottleScale();
    this.batteryCapacityAmpHours = config.batteryCapacityAmpHours();
    this.initialStateOfCharge = config.initialStateOfCharge();
    this.lastValidVoltageVolts = this.throttleStartVolts;
  }

  /**
   * Updates the power monitor with a new timestamped voltage and total current measurement.
   * Non-finite timestamps or voltages are ignored so sensor glitches never trip false brownouts or
   * drop drive throttle.
   */
  public PowerState update(Time timestamp, Voltage batteryVoltage, Current totalCurrent) {
    Objects.requireNonNull(timestamp, "timestamp must not be null");
    Objects.requireNonNull(batteryVoltage, "batteryVoltage must not be null");
    Objects.requireNonNull(totalCurrent, "totalCurrent must not be null");

    double nowSeconds = timestamp.in(Seconds);
    double volts = batteryVoltage.in(Volts);
    double amps = totalCurrent.in(Amps);

    if (!Double.isFinite(nowSeconds) || !Double.isFinite(volts)) {
      return currentState;
    }

    double dtSeconds = hasLastTimestamp ? Math.max(0.0, nowSeconds - lastTimestampSeconds) : 0.0;
    lastTimestampSeconds = nowSeconds;
    hasLastTimestamp = true;
    lastValidVoltageVolts = volts;

    if (Double.isFinite(amps)) {
      lastValidCurrentAmps = Math.max(0.0, amps);
      if (dtSeconds > 0.0 && lastValidCurrentAmps > 0.0) {
        consumedAmpHours += lastValidCurrentAmps * (dtSeconds / SECONDS_PER_HOUR);
      }
    }

    if (volts <= brownoutRailVolts) {
      if (!inBrownout) {
        inBrownout = true;
        brownoutCount++;
      }
    } else if (volts >= brownoutRailVolts + hysteresisVolts) {
      inBrownout = false;
    }

    if (volts < emergencyVolts) {
      emergencyTimerSeconds += dtSeconds;
      if (emergencyTimerSeconds >= emergencyDebounceSeconds) {
        emergencyActive = true;
      }
    } else if (volts >= emergencyVolts + hysteresisVolts) {
      emergencyTimerSeconds = 0.0;
      emergencyActive = false;
    }

    if (volts < warningVolts) {
      warningTimerSeconds += dtSeconds;
      if (warningTimerSeconds >= warningDebounceSeconds) {
        warningActive = true;
      }
    } else if (volts >= warningVolts + hysteresisVolts) {
      warningTimerSeconds = 0.0;
      warningActive = false;
    }

    throttleScale = computeThrottleScale(volts);
    currentState = resolveState();
    return currentState;
  }

  /** Resets timers, brownout counter, and coulomb-counted consumption back to initial state. */
  public void reset() {
    hasLastTimestamp = false;
    lastTimestampSeconds = 0.0;
    lastValidVoltageVolts = throttleStartVolts;
    lastValidCurrentAmps = 0.0;
    consumedAmpHours = 0.0;
    warningTimerSeconds = 0.0;
    emergencyTimerSeconds = 0.0;
    warningActive = false;
    emergencyActive = false;
    inBrownout = false;
    brownoutCount = 0;
    throttleScale = FULL_THROTTLE_SCALE;
    currentState = PowerState.NOMINAL;
  }

  public PowerState getState() {
    return currentState;
  }

  public double getThrottleScale() {
    return throttleScale;
  }

  public Voltage getLastValidVoltage() {
    return Volts.of(lastValidVoltageVolts);
  }

  public Current getLastValidCurrent() {
    return Amps.of(lastValidCurrentAmps);
  }

  public int getBrownoutCount() {
    return brownoutCount;
  }

  public double getConsumedAmpHours() {
    return consumedAmpHours;
  }

  /**
   * Returns the estimated remaining state of charge in {@code [0.0, initialStateOfCharge]} based on
   * coulomb counting from the explicitly configured {@code initialStateOfCharge}.
   */
  public double getRemainingStateOfCharge() {
    double remaining = initialStateOfCharge - (consumedAmpHours / batteryCapacityAmpHours);
    return Math.clamp(remaining, MIN_SOC, MAX_SOC);
  }

  /** Returns a human-readable severity label for dashboards using an exhaustive enum switch. */
  public String getSeveritySummary() {
    return switch (currentState) {
      case NOMINAL -> "Nominal bus voltage";
      case LOW_VOLTAGE_WARNING -> "Low voltage warning: scaling drive throttle";
      case LOW_VOLTAGE_EMERGENCY -> "Emergency sag: minimum throttle active";
      case BROWNOUT -> "Brownout rail reached";
    };
  }

  private double computeThrottleScale(double volts) {
    double fraction = (volts - throttleFloorVolts) / (throttleStartVolts - throttleFloorVolts);
    double rawScale = minThrottleScale + (FULL_THROTTLE_SCALE - minThrottleScale) * fraction;
    return Math.clamp(rawScale, minThrottleScale, FULL_THROTTLE_SCALE);
  }

  private PowerState resolveState() {
    if (inBrownout) {
      return PowerState.BROWNOUT;
    }
    if (emergencyActive) {
      return PowerState.LOW_VOLTAGE_EMERGENCY;
    }
    if (warningActive) {
      return PowerState.LOW_VOLTAGE_WARNING;
    }
    return PowerState.NOMINAL;
  }
}
