# FlyWheelConfig

**Package:** `yams.core.mechanisms.config`\
**Javadoc:** [`FlyWheelConfig`](https://javadocs.yassrobotics.com/yams/yams/core/mechanisms/config/FlyWheelConfig.html)

`FlyWheelConfig` configures a velocity-controlled spinning mechanism: shooters, and also intakes, rollers, feeders and indexers (even when driven open loop).

All builder methods return the `FlyWheelConfig` instance for chaining.

See also: [FlyWheel](../mechanisms/flywheel.md)

{% hint style="info" %}
**Simulation Note:** Moment of inertia is not configured on `FlyWheelConfig`. Set it via `SmartMotorControllerConfig.withMomentOfInertia(Distance length, Mass mass)`.
{% endhint %}

***

## Constructor

```java
FlyWheelConfig()
FlyWheelConfig clone()  // Deep copy
```

***

## Builder methods

| Method                                                        | Parameters                                             | Description                                                                                                                                               |
| ------------------------------------------------------------- | ------------------------------------------------------ | --------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `withDiameter(Distance diameter)`                             | `diameter`: wheel/roller outer diameter                | Wheel size used for angular-to-linear velocity conversion (`getLinearVelocity`, `getAngularVelocity`) and the visualization.                              |
| `withSpeedometerSimulation(AngularVelocity maxVelocity)`      | `maxVelocity`: maximum simulated velocity              | Enables a simulated speedometer gauge capped at `maxVelocity`.                                                                                            |
| `withSpeedometerSimulation()`                                 | _(none)_                                               | Enables the speedometer with the previously configured max velocity. Throws a `FlyWheelConfigurationException` if max velocity was not set first.         |
| `disableSpeedometerSimulation()`                              | _(none)_                                               | Disables the speedometer simulation.                                                                                                                      |
| `withSimColor(Color8Bit color)`                               | `color`: RGB color                                     | Sets the Mechanism2d visualization color. Optional; defaults to orange.                                                                                   |
| `withMechanismPositionConfig(MechanismPositionConfig config)` | `config`: position config                              | Where the mechanism is on the robot, for `getPose3d()`, and the plane it moves in. See [MechanismPositionConfig](mechanism-position-config.md). Optional. |
| `withTelemetry(String name, TelemetryVerbosity verbosity)`    | `name`: NT key prefix, `verbosity`: `LOW`/`MID`/`HIGH` | Enables NetworkTables telemetry for this mechanism. Optional.                                                                                             |

***

## Getters

| Method                                        | Returns                        | Description                                                                  |
| --------------------------------------------- | ------------------------------ | ---------------------------------------------------------------------------- |
| `getTelemetryName()`                          | `Optional<String>`             | The configured NT prefix, or empty if telemetry is not set.                  |
| `getTelemetryVerbosity()`                     | `Optional<TelemetryVerbosity>` | The configured verbosity, or empty.                                          |
| `getDiameter()`                               | `Optional<Distance>`           | The wheel diameter, or empty if not configured.                              |
| `getCircumference()`                          | `Distance`                     | The wheel circumference. Requires `withDiameter`.                            |
| `getLinearVelocity(AngularVelocity velocity)` | `LinearVelocity`               | Converts a wheel angular velocity to surface speed. Requires `withDiameter`. |
| `getAngularVelocity(LinearVelocity velocity)` | `AngularVelocity`              | Converts a surface speed to wheel angular velocity. Requires `withDiameter`. |
| `isUsingSpeedometerSimulation()`              | `boolean`                      | Whether the speedometer simulation is enabled.                               |
| `getSpeedometerMaxVelocity()`                 | `Optional<AngularVelocity>`    | The speedometer maximum velocity, or empty.                                  |
| `getSimColor()`                               | `Color8Bit`                    | The visualization color. Defaults to orange.                                 |
| `getMechanismPositionConfig()`                | `MechanismPositionConfig`      | The position/plane configuration.                                            |

***

## Example

```java
FlyWheelConfig config = new FlyWheelConfig()
    .withDiameter(Inches.of(4))
    .withSpeedometerSimulation(RPM.of(6000))
    .withTelemetry("Shooter", TelemetryVerbosity.HIGH);
```

{% @github-files/github-code-block url="https://github.com/Yet-Another-Software-Suite/YAMS/blob/master/examples/commands2/simple_robot-2027/src/main/java/first/robot/subsystems/ShooterSubsystem.java#L49-L53" %}
