---
description: >-
  Walkthrough of the power health monitoring example robot in Commands v2,
  Commands v3, and C++.
icon: code
---

# The power health monitoring example

This example pairs a standard YAMS `SwerveDrive` with a standalone `PowerHealthMonitor` and YAMS battery discharge simulation. No core YAMS classes are modified.

## Where the code lives

* **Commands v2:** [`examples/commands2/power_health_monitoring/`](https://github.com/Yet-Another-Software-Suite/YAMS/tree/master/examples/commands2/power_health_monitoring)
* **Commands v3:** [`examples/commands3/power_health_monitoring/`](https://github.com/Yet-Another-Software-Suite/YAMS/tree/master/examples/commands3/power_health_monitoring)

## What each file does

| Concept | Explained in | Where in the example |
| --- | --- | --- |
| Hysteresis thresholds, debounce & throttle scaling | [How the monitor works](./#how-the-monitor-works) | `PowerHealthMonitor.java` (`update(Time, Voltage, Current)`) |
| Enabling simulated battery discharge | [Battery discharge simulation](../../how-to-simulation-and-logging/how-do-i-enable-battery-discharge-simulation.md) | `Robot.java` (`BatterySim.enableDischarge(...)`) |
| Summing motor currents & applying throttle scale | [How the monitor works](./#how-the-monitor-works) | `SwerveSubsystem.java` (v2) / `SwerveMechanism.java` (v3) |
| Unit + seeded fuzz tests (NaN, hysteresis, SoC) | [Why a naive voltage check fails](./#why-a-naive-voltage-check-fails-on-a-real-robot) | `PowerHealthMonitorTest.java` & `PowerHealthMonitoringTest.java` |

## Updating the monitor and scaling drive output

{% tabs %}
{% tab title="Java, Commands v2" %}
{% code title="PowerHealthMonitor.java (fragment)" %}
```java
public PowerState update(Time timestamp, Voltage batteryVoltage, Current totalCurrent) {
  double nowSeconds = timestamp.in(Seconds);
  double volts = batteryVoltage.in(Volts);
  double amps = totalCurrent.in(Amps);

  // Ignore non-finite timestamps or voltages so a glitch never triggers a false brownout.
  if (!Double.isFinite(nowSeconds) || !Double.isFinite(volts)) {
    return currentState;
  }

  double dtSeconds = hasLastTimestamp ? Math.max(0.0, nowSeconds - lastTimestampSeconds) : 0.0;
  lastTimestampSeconds = nowSeconds;
  hasLastTimestamp = true;
  lastValidVoltageVolts = volts;

  if (Double.isFinite(amps) && amps > 0.0 && dtSeconds > 0.0) {
    consumedAmpHours += amps * (dtSeconds / SECONDS_PER_HOUR);
  }

  // Schmitt-trigger hysteresis on brownout, emergency, and warning states.
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
```
{% endcode %}

{% code title="SwerveSubsystem.java (fragment)" %}
```java
@Override
public void periodic() {
  drive.updateTelemetry();
  powerMonitor.update(
      Seconds.of(Timer.getTimestamp()),
      RobotController.getMeasureBatteryVoltage(),
      getTotalMotorCurrent());

  double scale = powerMonitor.getThrottleScale();
  // Publish PowerHealth/* telemetry for AdvantageScope and driver alerts.
  voltagePub.set(powerMonitor.getLastValidVoltage().in(Volts));
  throttleScalePub.set(scale);
  socPub.set(powerMonitor.getRemainingStateOfCharge());
  brownoutCountPub.set(powerMonitor.getBrownoutCount());
  statePub.set(powerMonitor.getState().name());
}
```
{% endcode %}
{% endtab %}

{% tab title="Java, Commands v3" %}
{% code title="SwerveMechanism.java (fragment)" %}
```java
public void periodic() {
  drive.updateTelemetry();
  powerMonitor.update(
      Seconds.of(Timer.getTimestamp()),
      RobotController.getMeasureBatteryVoltage(),
      getTotalMotorCurrent());

  voltagePub.set(powerMonitor.getLastValidVoltage().in(Volts));
  throttleScalePub.set(powerMonitor.getThrottleScale());
  socPub.set(powerMonitor.getRemainingStateOfCharge());
  brownoutCountPub.set(powerMonitor.getBrownoutCount());
  statePub.set(powerMonitor.getState().name());
}

public Command driveFieldRelative(
    DoubleSupplier vx, DoubleSupplier vy, DoubleSupplier omega) {
  return run(() -> {
        double scale = powerMonitor.getThrottleScale();
        drive.setFieldRelativeChassisSpeeds(
            new ChassisVelocities(
                vx.getAsDouble() * scale,
                vy.getAsDouble() * scale,
                omega.getAsDouble() * scale));
      })
      .named("Drive with Power Health Throttle");
}
```
{% endcode %}
{% endtab %}

{% tab title="C++" %}
{% code title="PowerHealthMonitor.hpp (fragment)" %}
```cpp
#include <algorithm>
#include <cmath>
#include <wpi/units/current.hpp>
#include <wpi/units/time.hpp>
#include <wpi/units/voltage.hpp>

class PowerHealthMonitor {
 public:
  double UpdateThrottleScale(wpi::units::volt_t batteryVoltage) {
    double v = batteryVoltage.value();
    if (!std::isfinite(v)) {
      return m_throttleScale;
    }
    double frac = (v - m_throttleFloorV) / (m_throttleStartV - m_throttleFloorV);
    m_throttleScale = std::clamp(
        m_minThrottle + (1.0 - m_minThrottle) * frac, m_minThrottle, 1.0);
    return m_throttleScale;
  }

 private:
  double m_throttleStartV = 11.0;
  double m_throttleFloorV = 9.0;
  double m_minThrottle = 0.35;
  double m_throttleScale = 1.0;
};
```
{% endcode %}
{% endtab %}
{% endtabs %}

## Tuning for your robot

1. **Match your hardware brownout rail:** set `BROWNOUT_RAIL_VOLTAGE` in `Constants.PowerHealthConstants` to match whatever you pass to `RobotController.setBrownoutVoltages(...)`.
2. **Set `INITIAL_STATE_OF_CHARGE` realistically:** keep `1.0` for fresh match batteries, or pass a lower starting SoC (e.g., `0.80`) when testing on a partially discharged practice pack.
3. **Choose `MIN_THROTTLE_SCALE`:** `0.35` keeps the robot controllable during deep sag without letting it stall completely; lower it only if your match strategy prioritizes subsystem power (e.g., endgame climb) over drive speed.
4. **Log in AdvantageScope:** graph `PowerHealth/VoltageVolts`, `PowerHealth/ThrottleScale`, `PowerHealth/RemainingSoC`, and `PowerHealth/State` alongside your motor supply currents.

## If it does not work

* **Voltage never drops in simulation:** confirm `BatterySim.enableDischarge(...)` is called in `Robot()` and `drive.simIterate()` runs inside `simulationPeriodic()`.
* **State of charge barely moves on SPARK MAX / Flex:** SPARK wrappers report stator current rather than supply current (`getSupplyCurrent()` is empty); the example falls back to stator current, which is higher than battery draw at low duty cycles. For exact battery Ah accounting on a real robot, pass `PowerDistribution.getTotalCurrent()` into `powerMonitor.update(...)`.
* **Throttle feels sluggish on a fresh battery:** raise `THROTTLE_START_VOLTAGE` only if your battery rests well above `11.5 V` under acceleration; on a healthy battery, `11.0 V` leaves full-speed driving completely untouched during normal play.
