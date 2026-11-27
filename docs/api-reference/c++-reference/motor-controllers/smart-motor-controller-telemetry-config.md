# SmartMotorControllerTelemetryConfig

**Namespace:** `yams::telemetry`\
**Header:** `#include <yams/telemetry/SmartMotorControllerTelemetryConfig.hpp>`\
**Doxygen:** [`SmartMotorControllerTelemetryConfig`](https://doxygen.yassrobotics.com/yams/classyams_1_1telemetry_1_1SmartMotorControllerTelemetryConfig.html)

Fine-grained telemetry configuration for a `SmartMotorController`. Every field is **disabled by default**. Opt-in using `WithTelemetryVerbosity()` for a preset bundle, individual `With*()` methods for specific fields, or both to supplement a preset.

For most use-cases, passing a name and verbosity to `SmartMotorControllerConfig::WithTelemetry(const std::string&, TelemetryVerbosity)` is sufficient. Use this class only when you need to override which specific fields are published or to route telemetry to DataLog instead of NetworkTables.

## Type alias

```cpp
using TelemetryVerbosity =
    yams::motorcontrollers::SmartMotorControllerConfig::TelemetryVerbosity;
```

## Constructor

```cpp
SmartMotorControllerTelemetryConfig()
```

## Verbosity presets

`WithTelemetryVerbosity(TelemetryVerbosity)` enables a cumulative set of fields. Each higher level includes everything from the levels below it.

| Level    | Fields enabled                                                                                                                                                                                                                                                                                                             |
| -------- | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `NONE`   | Nothing                                                                                                                                                                                                                                                                                                                    |
| `LOW`    | Setpoint position/velocity, measurement position/velocity/acceleration, mechanism position/velocity/acceleration, rotor position/velocity, external encoder position/velocity, active closed loop slot                                                                                                                     |
| `MEDIUM` | Everything in LOW, plus: output voltage, stator current, supply current                                                                                                                                                                                                                                                    |
| `HIGH`   | Everything in MEDIUM, plus: all boolean status flags (including motor and encoder inversion), tunable setpoints and closed loop slot, PID gains (kP/kI/kD), or the LQR's values with an LQR, feedforward gains (kS/kV/kG/kA), current limits, ramp rates, soft limit values, motor temperature, setpoint feedforward force |

{% hint style="info" %}
`HIGH` also publishes the tunable values that drive live tuning. When the motor controller's config has a subsystem (`WithSubsystem(this)`), a `Live Tuning` command is registered automatically at `HIGH`; see [SmartMotorController](smart-motor-controller.md#telemetry-and-live-tuning).
{% endhint %}

### LQR fields

With an [LQR](../math/lqr-controller.md) as the closed loop controller, `HIGH` publishes the LQR's values in place of kP, kI and kD. They are tunable, and only appear when the config has an LQR, and only the ones its type uses. They have no named builder method; enable them one by one with `WithCustom(DoubleTelemetryField, true)`.

| `DoubleTelemetryField` | Key                               | Units                              | Used by       |
| ---------------------- | --------------------------------- | ---------------------------------- | ------------- |
| `LQRQelmsPosition`     | `closedloop/lqr/qelms/position`   | Degrees, or meters for an elevator | Arm, elevator |
| `LQRQelmsVelocity`     | `closedloop/lqr/qelms/velocity`   | RPM, or m/s for an elevator        | All           |
| `LQRRelms`             | `closedloop/lqr/relms`            | Volts                              | All           |
| `LQRModelPosition`     | `closedloop/lqr/model/position`   | Degrees, or meters for an elevator | Arm, elevator |
| `LQRModelVelocity`     | `closedloop/lqr/model/velocity`   | RPM, or m/s for an elevator        | All           |
| `LQREncoderPosition`   | `closedloop/lqr/encoder/position` | Degrees, or meters for an elevator | Arm, elevator |
| `LQREncoderVelocity`   | `closedloop/lqr/encoder/velocity` | RPM                                | Flywheel      |
| `LQRMaxVoltage`        | `closedloop/lqr/maxVoltage`       | Volts                              | All           |

They start at the values in the `LQRConfig`. Live tuning a change rebuilds the LQR from all of them, keeping its state estimate; a value that is not above 0 leaves the LQR as it was, with a warning.

## Builder methods: Output channels

All methods return `SmartMotorControllerTelemetryConfig&` for chaining.

| Method                                                 | Description                                                               |
| ------------------------------------------------------ | ------------------------------------------------------------------------- |
| `WithDataLogName(const std::string& dataLogName)`      | Enables DataLog recording; writes all enabled fields to this prefix path. |
| `WithNetworkTables(bool enabled)`                      | Enables or disables NT4 publishing. Default: `true`.                      |
| `WithoutNetworkTables()`                               | Disables NT4 publishing. Useful during competition matches.               |
| `WithTelemetryVerbosity(TelemetryVerbosity verbosity)` | Enables a preset bundle (`LOW`, `MEDIUM`, or `HIGH`).                     |

## Builder methods: Boolean fields

Each method enables a single boolean status flag. Flags are automatically suppressed if the motor controller or configuration does not support them.

| Method                      | What it logs                                                   |
| --------------------------- | -------------------------------------------------------------- |
| `WithMechanismLowerLimit()` | Whether the mechanism is at or past its lower soft limit.      |
| `WithMechanismUpperLimit()` | Whether the mechanism is at or past its upper soft limit.      |
| `WithTemperatureLimit()`    | Whether the motor is in an over-temperature condition.         |
| `WithVelocityControl()`     | Whether velocity closed loop mode is currently active.         |
| `WithElevatorFeedforward()` | Whether an elevator feedforward is in use.                     |
| `WithArmFeedforward()`      | Whether an arm feedforward is in use.                          |
| `WithSimpleFeedforward()`   | Whether a simple motor feedforward is in use.                  |
| `WithMotionProfile()`       | Whether a trapezoidal or exponential motion profile is active. |

## Builder methods: Numeric fields

| Method                      | What it logs                                                                                                                                                            |
| --------------------------- | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `WithSetpointPosition()`    | Current position setpoint in mechanism units.                                                                                                                           |
| `WithSetpointVelocity()`    | Current velocity setpoint in mechanism units.                                                                                                                           |
| `WithSetpointForce()`       | Feedforward force applied with the velocity setpoint (N).                                                                                                               |
| `WithClosedLoopPeriod()`    | How often the closed loop runs, `closedloop/period`, in seconds: 1 ms on the motor controller, or the closed loop control period on the SystemCore. Included at `HIGH`. |
| `WithOutputVoltage()`       | Motor output voltage (V).                                                                                                                                               |
| `WithStatorCurrent()`       | Stator (output) current (A).                                                                                                                                            |
| `WithTemperature()`         | Motor temperature.                                                                                                                                                      |
| `WithMeasurementPosition()` | Linear position via mechanism circumference (m). Requires `WithMechanismCircumference`.                                                                                 |
| `WithMeasurementVelocity()` | Linear velocity in m/s. Requires `WithMechanismCircumference`.                                                                                                          |
| `WithMechanismPosition()`   | Mechanism angular position (rotations).                                                                                                                                 |
| `WithMechanismVelocity()`   | Mechanism angular velocity (rotations/s).                                                                                                                               |
| `WithRotorPosition()`       | Raw rotor position (rotations).                                                                                                                                         |
| `WithRotorVelocity()`       | Raw rotor velocity (rotations/s).                                                                                                                                       |

## Builder methods: Custom fields

`WithCustom` enables or disables any field from the `yams::telemetry::DoubleTelemetryField` and `yams::telemetry::BooleanTelemetryField` enums (declared in `yams/telemetry/SmartMotorControllerTelemetry.hpp`), including fields that have no dedicated `With*()` method such as `SupplyCurrent`, `kP` or `TrapezoidalProfileMaxVelocity`.

```cpp
SmartMotorControllerTelemetryConfig& WithCustom(DoubleTelemetryField field, bool enabled);
SmartMotorControllerTelemetryConfig& WithCustom(BooleanTelemetryField field, bool enabled);
SmartMotorControllerTelemetryConfig& WithCustom(const std::vector<DoubleTelemetryField>& fields, bool enabled);
SmartMotorControllerTelemetryConfig& WithCustom(const std::vector<BooleanTelemetryField>& fields, bool enabled);
```

## Getters

| Method                                       | Returns                      | Description                                                  |
| -------------------------------------------- | ---------------------------- | ------------------------------------------------------------ |
| `GetDataLogName() const`                     | `std::optional<std::string>` | DataLog prefix, empty when DataLog recording is not enabled. |
| `GetNT4Enabled() const`                      | `bool`                       | Whether NT4 publishing is enabled.                           |
| `GetDoubleFields(SmartMotorController& smc)` | map of double fields         | Field map used internally by the telemetry system.           |
| `GetBoolFields(SmartMotorController& smc)`   | map of boolean fields        | Field map used internally by the telemetry system.           |

## Example

```cpp
#include <yams/telemetry/SmartMotorControllerTelemetryConfig.hpp>
#include <yams/motorcontrollers/SmartMotorControllerConfig.hpp>

using Cfg    = yams::motorcontrollers::SmartMotorControllerConfig;
using TelCfg = yams::telemetry::SmartMotorControllerTelemetryConfig;
using yams::telemetry::DoubleTelemetryField;

// Preset-based configuration applied via SmartMotorControllerConfig (simplest approach)
Cfg motorConfig;
motorConfig.WithTelemetry("ShoulderMotor", Cfg::TelemetryVerbosity::HIGH);

// Fine-grained override applied after constructing the motor controller
smc.WithTelemetry(
    TelCfg{}
        .WithTelemetryVerbosity(Cfg::TelemetryVerbosity::MEDIUM)
        .WithMechanismPosition()
        .WithMechanismVelocity()
        .WithTemperature()
        .WithArmFeedforward()
        .WithCustom(DoubleTelemetryField::SupplyCurrent, false)
        .WithDataLogName("/Robot/Arm/Motor")
        .WithoutNetworkTables());   // log-only during matches
```

{% hint style="info" %}
Fields that are not applicable to the current motor controller or configuration are silently suppressed at runtime.
{% endhint %}

{% hint style="info" %}
`WithTelemetryVerbosity()` is cumulative: combining it with individual `With*()` methods produces the union of all requested fields.
{% endhint %}

## Related pages

* [SmartMotorController (C++)](smart-motor-controller.md)
* [SmartMotorControllerConfig (C++)](smart-motor-controller-config.md)
* [SmartMotorControllerTelemetryConfig (Java)](../../java-reference/motor-controllers/smart-motor-controller-telemetry-config.md)
