# BatterySim

**Namespace:** `yams::motorcontrollers::simulation`\
**Header:** `#include <yams/motorcontrollers/simulation/BatterySim.hpp>`\
**Doxygen:** [`BatterySim`](https://doxygen.yassrobotics.com/yams/classyams_1_1motorcontrollers_1_1simulation_1_1BatterySim.html)

`BatterySim` models a single, shared robot battery in simulation. Every simulated `SmartMotorController` (via its `ArmSimSupplier`, `ElevatorSimSupplier`, or `DCMotorSimSupplier`, or directly from `SparkWrapper`/`NovaWrapper`/`A301Wrapper`/`TalonFXWrapper`/`TalonFXSWrapper`) registers its own supply current draw here every loop. `BatterySim` combines the currently-registered draw of every mechanism into one loaded-voltage calculation (using `wpi::sim::BatterySim::Calculate`), which the sim suppliers write to the simulated input voltage (`wpi::sim::RoboRioSim::SetVInVoltage(...)`), so voltage sag reflects the whole robot's load, not just one mechanism in isolation.

{% hint style="info" %}
This class is entirely static and requires no setup for basic voltage sag under combined load: every built-in sim supplier and hardware wrapper registers with it automatically.
{% endhint %}

{% hint style="info" %}
The current draw registered here comes directly out of each mechanism's physics simulation, which derives current from the torque needed to produce a given acceleration. An unrealistic moment of inertia understates that current, and therefore the voltage sag `BatterySim` computes. Set a real MOI via [`SmartMotorControllerConfig::WithMOI(...)`](../motor-controllers/smart-motor-controller-config.md) for more realistic results.
{% endhint %}

## Static fields

| Field               | Type                 | Description                                                                                     |
| ------------------- | -------------------- | ----------------------------------------------------------------------------------------------- |
| `BatteryVoltage`    | `wpi::units::volt_t` | Nominal open-circuit battery voltage used when discharge simulation is disabled. Default `12V`. |
| `BatteryResistance` | `wpi::units::ohm_t`  | Nominal internal resistance used when discharge simulation is disabled. Default `0.020` ohm.    |

## Voltage calculation

```cpp
static wpi::units::volt_t CalculateVoltage(const void* id, wpi::units::ampere_t current);
```

Registers `current` under `id` and returns the resulting loaded battery voltage across every registered id. `id` should be a stable identity for the calling mechanism; YAMS's built-in sim suppliers and hardware wrappers pass the address of the owning `SmartMotorController`, so each motor controller counts once.

## Managing registered draws

```cpp
static void RemoveCurrent(const void* id);
static void Reset();
static bool HasCurrent(const void* id);
```

| Method          | Description                                                                                               |
| --------------- | --------------------------------------------------------------------------------------------------------- |
| `RemoveCurrent` | Stops counting the given consumer's draw. `SmartMotorController` calls this for itself when it is closed. |
| `Reset`         | Clears every tracked draw and resets discharge simulation to a fresh, fully charged, disabled state.      |
| `HasCurrent`    | Whether a draw is currently tracked for `id`.                                                             |

## Discharge simulation

By default, `BatterySim` holds a constant nominal voltage and resistance, enough to model sag under instantaneous combined load, but not a battery weakening over a match. Enable discharge modeling to layer state-of-charge tracking on top: current draw is integrated into amp-hours consumed over time (using `wpi::Timer::GetTimestamp()`, derated by discharge rate; see [Capacity Derating](battery-sim.md#capacity-derating) below), and the open-circuit voltage droops along a fixed discharge curve (flat through most of the charge, sagging quickly near depletion) while internal resistance rises as the battery empties.

```cpp
static void EnableDischarge(double batteryCapacityAmpHours, wpi::units::volt_t nominalVoltage,
                            wpi::units::ohm_t nominalResistance);
static void DisableDischarge();
static void ResetDischarge();
static double GetStateOfCharge();
```

| Method             | Description                                                                                        |
| ------------------ | -------------------------------------------------------------------------------------------------- |
| `EnableDischarge`  | Enables discharge modeling with the given capacity and nominal (fully-charged) voltage/resistance. |
| `DisableDischarge` | Disables discharge modeling; reverts to a constant `BatteryVoltage`/`BatteryResistance`.           |
| `ResetDischarge`   | Resets the simulated battery back to a full charge and clears the discharge-integration timestamp. |
| `GetStateOfCharge` | Current state of charge, from `0` (empty) to `1` (full).                                           |

{% hint style="warning" %}
Discharge simulation only affects `CalculateVoltage(...)`. It has no effect on a real robot, where the SystemCore's measured battery voltage reflects actual hardware.
{% endhint %}

## Custom discharge curves

`EnableDischarge(...)` sags voltage along a built-in curve that models a typical FRC sealed lead-acid battery: roughly flat through most of the charge, then dropping off quickly near depletion. Not every battery behaves that way. Call `ReplaceSOCInterpolation(...)` **before** `EnableDischarge(...)` to swap in a curve that matches the battery you're actually trying to model.

```cpp
static void ReplaceSOCInterpolation(const std::map<double, double>& socToVoltage);
```

Reach for this when:

* **You're modeling a well-used competition battery.** An old battery sags earlier and harder than a fresh one; a flatter, lower curve reproduces that instead of assuming every match starts with a fresh battery.
* **You measured a real curve.** If you've put a battery on a load tester and have actual voltage-vs-state-of-charge data, feeding that in directly gives the most accurate brownout predictions possible.

```cpp
#include <map>
#include <yams/motorcontrollers/simulation/BatterySim.hpp>

// Model a well-used competition battery that sags earlier and more severely than a new one.
std::map<double, double> wornBatteryCurve{
    {0.00, 8.0},  {0.05, 9.5},  {0.10, 10.5}, {0.20, 11.2}, {0.40, 11.6},
    {0.60, 11.9}, {0.80, 12.2}, {0.90, 12.4}, {1.00, 12.6},
};

yams::motorcontrollers::simulation::BatterySim::ReplaceSOCInterpolation(wornBatteryCurve);
// Pair with a reduced usable capacity and higher resistance to match a worn battery.
yams::motorcontrollers::simulation::BatterySim::EnableDischarge(
    15.0, wpi::units::volt_t{12.6}, wpi::units::ohm_t{0.028});
```

{% hint style="info" %}
Keys and values should span the full `[0, 1]` state-of-charge range. Querying outside the range you defined returns the nearest endpoint's voltage instead of extrapolating, so a table missing the low or high end will not sag realistically there.
{% endhint %}

## Capacity derating

Sealed lead-acid batteries deliver noticeably fewer amp-hours the faster they're discharged (the Peukert effect), unlike lithium chemistries, which stay close to their rated capacity across a wide range of discharge currents. A battery rated for 18 Ah at a light 0.9 A draw might only deliver \~11 Ah at a sustained 54 A draw, which is well within normal FRC match currents. `EnableDischarge(...)` derates the amp-hours consumed by a discharge-current → capacity-fraction table so the modeled state of charge drops faster under heavy sustained load, matching this behavior instead of assuming the full rated capacity is available at any current.

```cpp
static void ReplaceCapacityDerating(const std::map<double, double>& currentToCapacityFraction);
```

| Method                    | Description                                                                                                 |
| ------------------------- | ----------------------------------------------------------------------------------------------------------- |
| `ReplaceCapacityDerating` | Replaces the discharge-current (Amps) → capacity-fraction `[0, 1]` table used to derate amp-hours consumed. |

The default table is averaged from discharge testing across five FRC battery manufacturers, see [Detailed FRC Battery Comparison for 2026](https://www.chiefdelphi.com/t/detailed-frc-battery-comparison-for-2026/508077):

| Discharge Current | Capacity Fraction |
| ----------------- | ----------------- |
| 0.9 A             | 1.000             |
| 18 A              | 0.758             |
| 27 A              | 0.718             |
| 36 A              | 0.679             |
| 45 A              | 0.639             |
| 54 A              | 0.599             |

Reach for `ReplaceCapacityDerating(...)` if you have measured discharge-rate-vs-capacity data for your specific battery rather than the averaged multi-manufacturer defaults. Call it before `EnableDischarge(...)` so discharge simulation uses the new curve from the start.

```cpp
#include <map>
#include <yams/motorcontrollers/simulation/BatterySim.hpp>

std::map<double, double> measuredDerating{
    {0.9, 1.000}, {20.0, 0.80}, {40.0, 0.65}, {60.0, 0.55},
};

yams::motorcontrollers::simulation::BatterySim::ReplaceCapacityDerating(measuredDerating);
yams::motorcontrollers::simulation::BatterySim::EnableDischarge(
    18.0, wpi::units::volt_t{12.9}, wpi::units::ohm_t{0.020});
```

{% hint style="info" %}
Discharge currents outside the range you define clamp to the nearest endpoint's fraction instead of extrapolating, the same as `ReplaceSOCInterpolation(...)`.
{% endhint %}

## Example

```cpp
#include <yams/motorcontrollers/simulation/BatterySim.hpp>

Robot::Robot() {
  // Model an 18Ah battery starting at a full 12.9V, 20 mOhm nominal.
  yams::motorcontrollers::simulation::BatterySim::EnableDischarge(
      18.0, wpi::units::volt_t{12.9}, wpi::units::ohm_t{0.020});
}

void Robot::UtilityInit() {
  // Start every test run from a full charge.
  yams::motorcontrollers::simulation::BatterySim::ResetDischarge();
}
```

## Related pages

* [SmartMotorController](../motor-controllers/smart-motor-controller.md)
* [BatterySim (Java)](../../java-reference/sensor/battery-sim.md)
* [Simulation](../../guides/simulation.md)
