# Elevator

**Package:** `yams.commands2.mechanisms` (Commands v2) or `yams.commands3.mechanisms` (Commands v3)\
**Extends:** `yams.core.mechanisms.positional.Elevator` (which extends `SmartPositionalMechanism`)\
**Implements:** `CommandMechanism`\
**Javadoc:** [`Elevator`](https://javadocs.yassrobotics.com/yams/yams/commands2/mechanisms/Elevator.html) (Commands v2), [`Elevator`](https://javadocs.yassrobotics.com/yams/yams/commands3/mechanisms/Elevator.html) (Commands v3)

Controls a linear elevator mechanism. Position is expressed as carriage height above the zero reference point.

Robot code imports `Elevator` from `yams.commands2.mechanisms` (or `yams.commands3.mechanisms`). The base class `yams.core.mechanisms.positional.Elevator` holds the state, simulation and telemetry. Its constructor is protected, so it is not constructed directly.

```java
import yams.commands2.config.SmartMotorControllerConfig;
import yams.commands2.mechanisms.Elevator;
import yams.core.mechanisms.config.ElevatorConfig;
```

## Constructor

```java
Elevator(ElevatorConfig config, SmartMotorController smc)
```

| Name     | Type                   | Description                                       |
| -------- | ---------------------- | ------------------------------------------------- |
| `config` | `ElevatorConfig`       | Elevator geometry, limits, and control parameters |
| `smc`    | `SmartMotorController` | Configured motor controller driving the elevator  |

The commands require the `Subsystem` (Commands v2) or `Mechanism` (Commands v3) passed to the `SmartMotorControllerConfig` constructor, so build the motor controller with `yams.commands2.config.SmartMotorControllerConfig` or `yams.commands3.config.SmartMotorControllerConfig` to match the `Elevator` you import. The `SmartMotorControllerConfig` must set `withMechanismCircumference(Distance)` so heights can be converted to motor rotations.

In simulation the constructor throws `ElevatorConfigurationException` if `ElevatorConfig.withCarriageWeight` or `ElevatorConfig.withHardLimits` is missing, and `SmartMotorControllerConfigurationException` if `SmartMotorControllerConfig.withStartingPosition` is not set or is outside the hard limits. See [ElevatorConfig](../config/elevator-config.md) and [SmartMotorControllerConfig](../motor-controllers/smart-motor-controller-config.md).

## State queries

These come from `yams.core.mechanisms.positional.Elevator` and work outside of commands.

| Method                                     | Returns                | Description                                                                      |
| ------------------------------------------ | ---------------------- | -------------------------------------------------------------------------------- |
| `getHeight()`                              | `Distance`             | Current carriage height from the encoder                                         |
| `getVelocity()`                            | `LinearVelocity`       | Current carriage velocity                                                        |
| `getMeasurementSetpoint()`                 | `Optional<Distance>`   | The closed loop position setpoint as a height, if there is one                   |
| `isNear(Distance height, Distance within)` | `boolean`              | `true` when the current height is within `within` of `height`                    |
| `isGte(Distance height)`                   | `boolean`              | `true` when the current height is greater than or equal to `height`              |
| `isLte(Distance height)`                   | `boolean`              | `true` when the current height is less than or equal to `height`                 |
| `isBetween(Distance start, Distance end)`  | `boolean`              | `true` when the current height is in `[start, end]`                              |
| `isAtMax()`                                | `boolean`              | `true` at or above the upper soft limit if set, otherwise the maximum hard limit |
| `isAtMin()`                                | `boolean`              | `true` at or below the lower soft limit if set, otherwise the minimum hard limit |
| `getConfig()`                              | `ElevatorConfig`       | The `ElevatorConfig` used to build this elevator                                 |
| `getMotorController()`                     | `SmartMotorController` | The underlying motor controller                                                  |

`isAtMax()` and `isAtMin()` throw `ElevatorConfigurationException` when neither a soft limit nor a hard limit is configured.

## Command factories

Commands that run indefinitely must be interrupted or composed with a terminating condition.

| Method                                                 | Returns   | Description                                                                     |
| ------------------------------------------------------ | --------- | ------------------------------------------------------------------------------- |
| `setHeight(Distance height)`                           | `Command` | Holds `height` using closed loop control; runs indefinitely                     |
| `setHeight(Supplier<Distance> height)`                 | `Command` | Holds the supplied height, re-evaluated each loop; runs indefinitely            |
| `run(Distance height)`                                 | `Command` | Same as `setHeight(Distance)`                                                   |
| `run(Supplier<Distance> height)`                       | `Command` | Same as `setHeight(Supplier<Distance>)`                                         |
| `runTo(Distance height, Distance tolerance)`           | `Command` | Sets the target and ends once within `tolerance` (debounced 0.1 s)              |
| `runTo(Supplier<Distance> height, Distance tolerance)` | `Command` | Reads the supplier once at start, then behaves like `runTo(Distance, Distance)` |
| `set(double dutycycle)`                                | `Command` | Open loop duty cycle; runs indefinitely                                         |
| `set(Supplier<Double> dutycycle)`                      | `Command` | Open loop duty cycle from a supplier; re-evaluated each loop                    |
| `setVoltage(Voltage volts)`                            | `Command` | Open loop fixed voltage; runs indefinitely                                      |
| `setVoltage(Supplier<Voltage> volts)`                  | `Command` | Open loop voltage from a supplier; re-evaluated each loop                       |

`set` and `setVoltage` come from the `CommandMechanism` interface. They stop the closed loop controller while running and restart it when the command ends.

### `setHeight`

```java
public Command setHeight(Distance height)
public Command setHeight(Supplier<Distance> height)
```

| Name     | Type                              | Description                     |
| -------- | --------------------------------- | ------------------------------- |
| `height` | `Distance` / `Supplier<Distance>` | Desired carriage height to hold |

Returns `Command`.

### `runTo`

```java
public Command runTo(Distance height, Distance tolerance)
public Command runTo(Supplier<Distance> height, Distance tolerance)
```

| Name        | Type                              | Description                                       |
| ----------- | --------------------------------- | ------------------------------------------------- |
| `height`    | `Distance` / `Supplier<Distance>` | Desired carriage height                           |
| `tolerance` | `Distance`                        | Acceptable height error for the command to finish |

Returns `Command`.

### `set`

```java
public Command set(double dutycycle)
public Command set(Supplier<Double> dutycycle)
```

| Name        | Type                          | Description                   |
| ----------- | ----------------------------- | ----------------------------- |
| `dutycycle` | `double` / `Supplier<Double>` | Motor duty cycle in `[-1, 1]` |

Returns `Command`.

### `setVoltage`

```java
public Command setVoltage(Voltage volts)
public Command setVoltage(Supplier<Voltage> volts)
```

| Name    | Type                            | Description    |
| ------- | ------------------------------- | -------------- |
| `volts` | `Voltage` / `Supplier<Voltage>` | Voltage output |

Returns `Command`.

## Triggers

| Method                                   | Returns   | Description                                                         |
| ---------------------------------------- | --------- | ------------------------------------------------------------------- |
| `near(Distance height, Distance within)` | `Trigger` | Active when the current height is within `within` of `height`       |
| `gte(Distance height)`                   | `Trigger` | Active when the current height is greater than or equal to `height` |
| `lte(Distance height)`                   | `Trigger` | Active when the current height is less than or equal to `height`    |
| `between(Distance start, Distance end)`  | `Trigger` | Active when the current height is in `[start, end]`                 |
| `max()`                                  | `Trigger` | Active while `isAtMax()` is `true`                                  |
| `min()`                                  | `Trigger` | Active while `isAtMin()` is `true`                                  |

{% hint style="info" %}
The `isNear(...)` Trigger factory was renamed to `near(...)`. `isNear(...)` still exists, but returns a `boolean` (see [State Queries](elevator.md#state-queries)).
{% endhint %}

```java
elevator.near(Meters.of(1.2), Inches.of(1)).onTrue(arm.setAngle(Degrees.of(45)));
```

## Commands v2 and Commands v3

Both packages have the same method names and parameters. The difference is the return types and what the commands require:

|                | Commands v2 (`yams.commands2.mechanisms.Elevator`)            | Commands v3 (`yams.commands3.mechanisms.Elevator`)            |
| -------------- | ------------------------------------------------------------- | ------------------------------------------------------------- |
| `Command` type | `org.wpilib.command2.Command`                                 | `org.wpilib.command3.Command`                                 |
| `Trigger` type | `org.wpilib.command2.button.Trigger`                          | `org.wpilib.command3.Trigger`                                 |
| Requirement    | The `Subsystem` from `getSubsystem()`                         | The `Mechanism` from `getMechanism()`                         |
| Motor config   | `yams.commands2.config.SmartMotorControllerConfig(Subsystem)` | `yams.commands3.config.SmartMotorControllerConfig(Mechanism)` |

In Commands v3 the commands are coroutines, so your own commands can `await` or `fork` them:

```java
public Command hold(Distance height)
{
  return run(coroutine -> coroutine.await(m_elevator.setHeight(height)))
      .withPriority(Command.LOWEST_PRIORITY)
      .named("Elevator Hold");
}
```

## Visualization and simulation

| Method                                       | Returns                | Description                                                                                                                                                                                                                                                                                                                               |
| -------------------------------------------- | ---------------------- | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `getMechanismLigament()`                     | `MechanismLigament2d`  | The ligament used in the Mechanism2d visualization (simulation only)                                                                                                                                                                                                                                                                      |
| `getMechanismRoot()`                         | `MechanismRoot2d`      | The root of the Mechanism2d visualization (simulation only)                                                                                                                                                                                                                                                                               |
| `getMechanismWindow()`                       | `Mechanism2d`          | The Mechanism2d window (simulation only)                                                                                                                                                                                                                                                                                                  |
| `getPose3d()`                                | `Pose3d`               | The carriage, moved by its height along the elevator's angle, relative to the robot, from its [`MechanismPositionConfig`](../config/mechanism-position-config.md). Works on the robot and in simulation                                                                                                                                   |
| `getCascadePose3d()`                         | `List<Pose3d>`         | Each moving stage's pose for a cascade elevator, whose stages all move at once, each by its share of the carriage's height. From the stage nearest the fixed stage to the carriage, which is `getPose3d()`. Needs 2 or more stages from `ElevatorConfig.withStages(...)`, or throws `ElevatorConfigurationException`                      |
| `getContinuousPose3d(boolean carriageFirst)` | `List<Pose3d>`         | Each moving stage's pose for a continuous elevator, whose stages move one at a time. `true`: the carriage moves the whole of its travel first, then the stage below it, and so on. `false`: the stage nearest the fixed stage moves first, and the carriage last. In the same order and with the same requirement as `getCascadePose3d()` |
| `getMotor()`                                 | `SmartMotorController` | The underlying motor controller                                                                                                                                                                                                                                                                                                           |
| `getName()`                                  | `String`               | The elevator telemetry name                                                                                                                                                                                                                                                                                                               |
| `simIterate()`                               | `void`                 | Advances the elevator physics simulation by one loop and updates the visualization                                                                                                                                                                                                                                                        |
| `updateTelemetry()`                          | `void`                 | Publishes current state to NetworkTables                                                                                                                                                                                                                                                                                                  |
| `visualizationUpdate()`                      | `void`                 | Syncs the Mechanism2d ligaments with the current height and setpoint, and logs the Mechanism2d again so it moves in the Sim GUI and dashboards                                                                                                                                                                                            |

`simIterate()` uses `ElevatorSim` internally, with the drum radius derived from the `SmartMotorControllerConfig` mechanism circumference. Gravity is simulated unless `ElevatorConfig.withHorizontalElevator()` is set.

Call `updateTelemetry()` from `periodic()` and `simIterate()` from `simulationPeriodic()`. `simIterate()` already calls `visualizationUpdate()`.

## Examples

{% @github-files/github-code-block url="https://github.com/Yet-Another-Software-Suite/YAMS/blob/master/examples/commands2/simple_robot-2027/src/main/java/first/robot/subsystems/ElevatorSubsystem.java" %}

{% @github-files/github-code-block url="https://github.com/Yet-Another-Software-Suite/YAMS/blob/master/examples/commands2/simple_robot-2027/src/main/java/first/robot/subsystems/ExponentiallyProfiledElevatorSubsystem.java" %}

Commands v3:

{% @github-files/github-code-block url="https://github.com/Yet-Another-Software-Suite/YAMS/blob/master/examples/commands3/simple_robot-2027/src/main/java/first/robot/mechanisms/ElevatorMechanism.java" %}

## See also

* [ElevatorConfig](../config/elevator-config.md)
* [SmartMotorControllerConfig](../motor-controllers/smart-motor-controller-config.md)
* [C++ Elevator](../../c++-reference/mechanisms/elevator.md)
