# ArmConfig

**Package:** `yams.core.mechanisms.config`\
**Javadoc:** [`ArmConfig`](https://javadocs.yassrobotics.com/yams/yams/core/mechanisms/config/ArmConfig.html)

`ArmConfig` configures a single-jointed arm mechanism. All builder methods return the `ArmConfig` instance for chaining.

See also: [Arm](../mechanisms/arm.md)

***

## Constructor

```java
ArmConfig()
ArmConfig clone()  // Deep copy
```

***

## Builder methods

> **Required for simulation:** `withLength()` and `withHardLimits()` must be set; `Arm` throws `ArmConfigurationException` in simulation without them. All other methods are optional.
>
> The moment of inertia and starting position are no longer part of `ArmConfig`. Set them on the motor controller config with `SmartMotorControllerConfig.withMomentOfInertia(...)` and `SmartMotorControllerConfig.withStartingPosition(Angle)`.

| Method                                                        | Parameters                                             | Description                                                                                                                                                                                                                        |
| ------------------------------------------------------------- | ------------------------------------------------------ | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `withLength(Distance distance)`                               | `distance`: arm link length                            | **Required for simulation.** Sets the physical length of the arm link. Used by `SingleJointedArmSim`. Without this, the physics simulation cannot model the arm.                                                                   |
| `withHardLimits(Angle min, Angle max)`                        | `min`: lower bound, `max`: upper bound                 | **Required for simulation.** Sets the physical rotation limits used by `SingleJointedArmSim` and the Mechanism2d visualization. For limits enforced by the motor controller, use `SmartMotorControllerConfig.withSoftLimits(...)`. |
| `withTelemetry(String name, TelemetryVerbosity verbosity)`    | `name`: NT key prefix, `verbosity`: `LOW`/`MID`/`HIGH` | Enables NetworkTables telemetry for this mechanism. Optional.                                                                                                                                                                      |
| `withSimColor(Color8Bit color)`                               | `color`: RGB color                                     | Sets the Mechanism2d visualization color. Defaults to orange if not set. Optional.                                                                                                                                                 |
| `withMechanismPositionConfig(MechanismPositionConfig config)` | `config`: position config                              | Where the mechanism is on the robot, for `getPose3d()`, and the plane it moves in. See [MechanismPositionConfig](mechanism-position-config.md). Optional.                                                                          |

***

## Getters

| Method                         | Returns                        | Description                                                          |
| ------------------------------ | ------------------------------ | -------------------------------------------------------------------- |
| `getTelemetryName()`           | `Optional<String>`             | The configured NT prefix, or empty if telemetry is not set.          |
| `getTelemetryVerbosity()`      | `Optional<TelemetryVerbosity>` | The configured verbosity, or empty.                                  |
| `getLength()`                  | `Optional<Distance>`           | The arm length, or empty if not configured.                          |
| `getLowerHardLimit()`          | `Optional<Angle>`              | Lower hard limit angle, or empty.                                    |
| `getUpperHardLimit()`          | `Optional<Angle>`              | Upper hard limit angle, or empty.                                    |
| `getSimColor()`                | `Color8Bit`                    | The visualization color. Always returns a value; defaults to orange. |
| `getMechanismPositionConfig()` | `MechanismPositionConfig`      | The position/plane configuration.                                    |

***

## Example

```java
ArmConfig config = new ArmConfig()
    .withLength(Meters.of(0.6))
    .withHardLimits(Degrees.of(-90), Degrees.of(90))
    .withTelemetry("Arm", TelemetryVerbosity.HIGH);
```
