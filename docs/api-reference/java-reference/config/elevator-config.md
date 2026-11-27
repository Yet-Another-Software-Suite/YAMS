# ElevatorConfig

**Package:** `yams.core.mechanisms.config`\
**Javadoc:** [`ElevatorConfig`](https://javadocs.yassrobotics.com/yams/yams/core/mechanisms/config/ElevatorConfig.html)

`ElevatorConfig` configures a linear elevator mechanism. All builder methods return the `ElevatorConfig` instance for chaining.

See also: [Elevator](../mechanisms/elevator.md)

***

## Constructor

```java
ElevatorConfig()
ElevatorConfig clone()  // Deep copy
```

***

## Builder methods

| Method                                                        | Parameters                                                    | Description                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            |
| ------------------------------------------------------------- | ------------------------------------------------------------- | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `withCarriageWeight(Mass mass)`                               | `mass`: carriage mass                                         | **Required for simulation.** Used by `ElevatorSim` to model gravity. `Elevator` throws `ElevatorConfigurationException` in simulation without it.                                                                                                                                                                                                                                                                                                                                                                                                                                      |
| `withHardLimits(Distance min, Distance max)`                  | `min`: bottom limit, `max`: top limit                         | **Required for simulation**, unless the stages are set with `withStages(...)`, which give the height from 0 to the stage heights' total. Sets the physical height range used by `ElevatorSim` and the Mechanism2d visualization. For limits enforced by the motor controller, use `SmartMotorControllerConfig.withSoftLimits(...)`.                                                                                                                                                                                                                                                    |
| `withAngle(Angle angle)`                                      | `angle`: angle from horizontal                                | The elevator's angle in its movement plane, which the carriage moves along, for the visualization and `getPose3d()`. Defaults to 90 degrees (vertical). Optional.                                                                                                                                                                                                                                                                                                                                                                                                                      |
| `withHorizontalElevator()`                                    | _(none)_                                                      | Disables gravity simulation. Useful for horizontal linear sliders. Optional.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           |
| `withStages(Distance... stageHeights)`                        | `stageHeights`: each moving stage's travel, the carriage last | The elevator's moving stages, for `Elevator.getCascadePose3d()` and `getContinuousPose3d(boolean)`: one height for each stage that moves, how far it travels relative to the stage below it, from the stage nearest the fixed stage to the carriage. 1 stage, an elevator with only a carriage, is valid. The heights must add up to the maximum height from `withHardLimits(...)`, or the `Elevator` throws `ElevatorConfigurationException`; without hard limits, they set the height. Throws `IllegalArgumentException` for no heights, or a height that is not positive. Optional. |
| `withTelemetry(String name, TelemetryVerbosity verbosity)`    | `name`: NT key prefix, `verbosity`: `LOW`/`MID`/`HIGH`        | Enables NetworkTables telemetry for this mechanism. Optional.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          |
| `withSimColor(Color8Bit color)`                               | `color`: RGB color                                            | Sets the Mechanism2d visualization color. Optional; defaults to orange.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                |
| `withMechanismPositionConfig(MechanismPositionConfig config)` | `config`: position config                                     | Where the mechanism is on the robot, for `getPose3d()`, and the plane it moves in. See [MechanismPositionConfig](mechanism-position-config.md). Optional.                                                                                                                                                                                                                                                                                                                                                                                                                              |

> **Note:** Drum radius (spool size) is configured on `SmartMotorControllerConfig` via `.withDrumRadius(Distance radius)`, `.withDrumRadius(Distance chainPitch, int teeth)` or `.withMechanismCircumference(Distance)`. Cascading elevator stages are configured via `SmartMotorControllerConfig.withCascadingElevatorStages(int stages)`. The starting height is set with `SmartMotorControllerConfig.withStartingPosition(Distance)` and must lie within the hard limits in simulation.

***

## Getters

| Method                         | Returns                        | Description                                                                                          |
| ------------------------------ | ------------------------------ | ---------------------------------------------------------------------------------------------------- |
| `getTelemetryName()`           | `Optional<String>`             | The configured NT prefix, or empty if telemetry is not set.                                          |
| `getTelemetryVerbosity()`      | `Optional<TelemetryVerbosity>` | The configured verbosity, or empty.                                                                  |
| `getCarriageMass()`            | `Optional<Mass>`               | The carriage mass, or empty if not configured.                                                       |
| `getMinimumHeight()`           | `Optional<Distance>`           | Lower hard limit, or 0 when the stages are set without hard limits, or empty.                        |
| `getMaximumHeight()`           | `Optional<Distance>`           | Upper hard limit, or the stage heights' total when the stages are set without hard limits, or empty. |
| `getAngle()`                   | `Angle`                        | The elevator angle. Defaults to 90 degrees.                                                          |
| `getIsElevatorHorizontal()`    | `boolean`                      | `true` if `withHorizontalElevator()` was called.                                                     |
| `getStages()`                  | `OptionalInt`                  | The number of moving stages, one for each height, or empty if `withStages(...)` was not called.      |
| `getStageHeights()`            | `List<Distance>`               | Each moving stage's travel, or an empty list.                                                        |
| `getStageHeightTotal()`        | `Optional<Distance>`           | How far the stages travel together, the sum of the stage heights, or empty.                          |
| `getSimColor()`                | `Color8Bit`                    | The visualization color. Defaults to orange.                                                         |
| `getMechanismPositionConfig()` | `MechanismPositionConfig`      | The position/plane configuration.                                                                    |

***

## Example

```java
ElevatorConfig config = new ElevatorConfig()
    .withCarriageWeight(Kilograms.of(4.5))
    .withHardLimits(Meters.of(0.0), Meters.of(1.2))
    .withTelemetry("Elevator", TelemetryVerbosity.HIGH);
```

{% @github-files/github-code-block url="https://github.com/Yet-Another-Software-Suite/YAMS/blob/master/examples/commands2/simple_robot-2027/src/main/java/first/robot/subsystems/ElevatorSubsystem.java#L82-L89" %}
