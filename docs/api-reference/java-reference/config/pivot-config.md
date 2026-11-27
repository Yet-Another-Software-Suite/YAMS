# PivotConfig

**Package:** `yams.core.mechanisms.config`\
**Javadoc:** [`PivotConfig`](https://javadocs.yassrobotics.com/yams/yams/core/mechanisms/config/PivotConfig.html)

`PivotConfig` configures a rotational mechanism in the XY plane (e.g., a turret or shooter hood). For mechanisms rotating in the XZ plane, use [ArmConfig](arm-config.md) instead.

All builder methods return the `PivotConfig` instance for chaining.

See also: [Pivot](../mechanisms/pivot.md)

***

## Constructor

```java
PivotConfig()
PivotConfig(PivotConfig cfg)  // Copy constructor
PivotConfig clone()           // Deep copy
```

***

## Builder methods

> **All methods are optional.** Unlike `ArmConfig`, `PivotConfig` has no simulation-required parameters: a pivot does not use arm length in its physics model.

| Method                                                        | Parameters                                             | Description                                                                                                                                                                         |
| ------------------------------------------------------------- | ------------------------------------------------------ | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `withHardLimits(Angle min, Angle max)`                        | `min`: lower bound, `max`: upper bound                 | Sets the physical rotation limits. The motor controller will not command positions outside these angles. Optional.                                                                  |
| `withTelemetry(String name, TelemetryVerbosity verbosity)`    | `name`: NT key prefix, `verbosity`: `LOW`/`MID`/`HIGH` | Enables NetworkTables telemetry for this mechanism. Optional.                                                                                                                       |
| `withSimColor(Color8Bit color)`                               | `color`: RGB color                                     | Sets the Mechanism2d visualization color. Defaults to orange if not set. Optional.                                                                                                  |
| `withMechanismPositionConfig(MechanismPositionConfig config)` | `config`: position config                              | Where the mechanism is on the robot, for `getPose3d()`, and the plane it moves in. See [MechanismPositionConfig](mechanism-position-config.md). Defaults to the XY plane. Optional. |

***

## Getters

| Method                         | Returns                        | Description                                                                     |
| ------------------------------ | ------------------------------ | ------------------------------------------------------------------------------- |
| `getTelemetryName()`           | `Optional<String>`             | The configured NT prefix, or empty if telemetry is not set.                     |
| `getTelemetryVerbosity()`      | `Optional<TelemetryVerbosity>` | The configured verbosity, or empty.                                             |
| `getLowerHardLimit()`          | `Optional<Angle>`              | Lower hard limit angle, or empty if not set.                                    |
| `getUpperHardLimit()`          | `Optional<Angle>`              | Upper hard limit angle, or empty if not set.                                    |
| `getSimColor()`                | `Color8Bit`                    | The visualization color. Always returns a value; defaults to orange.            |
| `getMechanismPositionConfig()` | `MechanismPositionConfig`      | The position/plane configuration. Always returns a value; defaults to XY plane. |

***

## Example

```java
PivotConfig config = new PivotConfig()
    .withHardLimits(Degrees.of(-180), Degrees.of(180))
    .withTelemetry("Turret", TelemetryVerbosity.MID);
```
