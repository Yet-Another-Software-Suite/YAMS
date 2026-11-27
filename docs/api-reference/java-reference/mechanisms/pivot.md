# Pivot

**Package:** `yams.commands2.mechanisms` (Commands v2) or `yams.commands3.mechanisms` (Commands v3)\
**Extends:** `yams.core.mechanisms.positional.Pivot` (which extends `SmartPositionalMechanism`)\
**Implements:** `CommandMechanism`\
**Javadoc:** [`Pivot`](https://javadocs.yassrobotics.com/yams/yams/commands2/mechanisms/Pivot.html) (Commands v2), [`Pivot`](https://javadocs.yassrobotics.com/yams/yams/commands3/mechanisms/Pivot.html) (Commands v3)

Controls rotation around a vertical axis, suitable for shooter hoods, turrets, or any mechanism rotating in the XY plane. For a gravity-affected arm rotating in the XZ plane, use [Arm](arm.md) instead.

Robot code imports `Pivot` from `yams.commands2.mechanisms` (or `yams.commands3.mechanisms`). The base class `yams.core.mechanisms.positional.Pivot` holds the state, simulation and telemetry. Its constructor is protected, so it is not constructed directly.

```java
import yams.commands2.config.SmartMotorControllerConfig;
import yams.commands2.mechanisms.Pivot;
import yams.core.mechanisms.config.PivotConfig;
```

## Constructor

```java
Pivot(PivotConfig config, SmartMotorController smc)
```

| Name     | Type                   | Description                                    |
| -------- | ---------------------- | ---------------------------------------------- |
| `config` | `PivotConfig`          | Pivot geometry, limits, and control parameters |
| `smc`    | `SmartMotorController` | Configured motor controller driving the pivot  |

The commands require the `Subsystem` (Commands v2) or `Mechanism` (Commands v3) passed to the `SmartMotorControllerConfig` constructor, so build the motor controller with `yams.commands2.config.SmartMotorControllerConfig` or `yams.commands3.config.SmartMotorControllerConfig` to match the `Pivot` you import.

In simulation the constructor throws `PivotConfigurationException` if `PivotConfig.withHardLimits` is missing, if `SmartMotorControllerConfig.withStartingPosition` is not set, or if the starting position is outside the hard limits. See [PivotConfig](../config/pivot-config.md) and [SmartMotorControllerConfig](../motor-controllers/smart-motor-controller-config.md).

## State queries

These come from `yams.core.mechanisms.positional.Pivot` and work outside of commands.

| Method                              | Returns                | Description                                                                    |
| ----------------------------------- | ---------------------- | ------------------------------------------------------------------------------ |
| `getAngle()`                        | `Angle`                | Current mechanism angle from the encoder                                       |
| `isNear(Angle angle, Angle within)` | `boolean`              | `true` when the current angle is within `within` of `angle`                    |
| `isGte(Angle angle)`                | `boolean`              | `true` when the current angle is greater than or equal to `angle`              |
| `isLte(Angle angle)`                | `boolean`              | `true` when the current angle is less than or equal to `angle`                 |
| `isBetween(Angle start, Angle end)` | `boolean`              | `true` when the current angle is in `[start, end]`                             |
| `isAtMax()`                         | `boolean`              | `true` at or above the upper soft limit if set, otherwise the upper hard limit |
| `isAtMin()`                         | `boolean`              | `true` at or below the lower soft limit if set, otherwise the lower hard limit |
| `getPivotConfig()`                  | `PivotConfig`          | The `PivotConfig` used to build this pivot                                     |
| `getMotorController()`              | `SmartMotorController` | The underlying motor controller                                                |
| `getMechanismSetpoint()`            | `Optional<Angle>`      | The current closed loop position setpoint, if any                              |

`isAtMax()` and `isAtMin()` throw `PivotConfigurationException` when neither a soft limit nor a hard limit is configured.

## Command factories

Commands that run indefinitely must be interrupted or composed with a terminating condition.

| Method                                          | Returns   | Description                                                               |
| ----------------------------------------------- | --------- | ------------------------------------------------------------------------- |
| `setAngle(Angle angle)`                         | `Command` | Holds `angle` using closed loop control; runs indefinitely                |
| `setAngle(Supplier<Angle> angle)`               | `Command` | Holds the supplied angle, re-evaluated each loop; runs indefinitely       |
| `run(Angle angle)`                              | `Command` | Same as `setAngle(Angle)`                                                 |
| `run(Supplier<Angle> angle)`                    | `Command` | Same as `setAngle(Supplier<Angle>)`                                       |
| `runTo(Angle angle, Angle tolerance)`           | `Command` | Sets the target and ends once within `tolerance` (debounced 0.1 s)        |
| `runTo(Supplier<Angle> angle, Angle tolerance)` | `Command` | Reads the supplier once at start, then behaves like `runTo(Angle, Angle)` |
| `set(double dutycycle)`                         | `Command` | Open loop duty cycle; runs indefinitely                                   |
| `set(Supplier<Double> dutycycle)`               | `Command` | Open loop duty cycle from a supplier; re-evaluated each loop              |
| `setVoltage(Voltage volts)`                     | `Command` | Open loop fixed voltage; runs indefinitely                                |
| `setVoltage(Supplier<Voltage> volts)`           | `Command` | Open loop voltage from a supplier; re-evaluated each loop                 |

`set` and `setVoltage` come from the `CommandMechanism` interface. They stop the closed loop controller while running and restart it when the command ends.

### `setAngle`

```java
public Command setAngle(Angle angle)
public Command setAngle(Supplier<Angle> angle)
```

| Name    | Type                        | Description                 |
| ------- | --------------------------- | --------------------------- |
| `angle` | `Angle` / `Supplier<Angle>` | Desired pivot angle to hold |

Returns `Command`.

### `runTo`

```java
public Command runTo(Angle angle, Angle tolerance)
public Command runTo(Supplier<Angle> angle, Angle tolerance)
```

| Name        | Type                        | Description                                        |
| ----------- | --------------------------- | -------------------------------------------------- |
| `angle`     | `Angle` / `Supplier<Angle>` | Desired pivot angle                                |
| `tolerance` | `Angle`                     | Acceptable angular error for the command to finish |

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

| Method                            | Returns   | Description                                                       |
| --------------------------------- | --------- | ----------------------------------------------------------------- |
| `near(Angle angle, Angle within)` | `Trigger` | Active when the current angle is within `within` of `angle`       |
| `gte(Angle angle)`                | `Trigger` | Active when the current angle is greater than or equal to `angle` |
| `lte(Angle angle)`                | `Trigger` | Active when the current angle is less than or equal to `angle`    |
| `between(Angle start, Angle end)` | `Trigger` | Active when the current angle is in `[start, end]`                |
| `max()`                           | `Trigger` | Active while `isAtMax()` is `true`                                |
| `min()`                           | `Trigger` | Active while `isAtMin()` is `true`                                |

{% hint style="info" %}
The `isNear(...)` Trigger factory was renamed to `near(...)`. `isNear(...)` still exists, but returns a `boolean` (see [State Queries](pivot.md#state-queries)).
{% endhint %}

```java
turret.near(Degrees.of(80), Degrees.of(2)).onTrue(shooter.run(RPM.of(3000)));
```

## Commands v2 and Commands v3

Both packages have the same method names and parameters. The difference is the return types and what the commands require:

|                | Commands v2 (`yams.commands2.mechanisms.Pivot`)               | Commands v3 (`yams.commands3.mechanisms.Pivot`)               |
| -------------- | ------------------------------------------------------------- | ------------------------------------------------------------- |
| `Command` type | `org.wpilib.command2.Command`                                 | `org.wpilib.command3.Command`                                 |
| `Trigger` type | `org.wpilib.command2.button.Trigger`                          | `org.wpilib.command3.Trigger`                                 |
| Requirement    | The `Subsystem` from `getSubsystem()`                         | The `Mechanism` from `getMechanism()`                         |
| Motor config   | `yams.commands2.config.SmartMotorControllerConfig(Subsystem)` | `yams.commands3.config.SmartMotorControllerConfig(Mechanism)` |

In Commands v3 the commands are coroutines, so your own commands can `await` or `fork` them:

```java
public Command aimAt(Angle angle)
{
  return run(coroutine -> coroutine.await(turret.runTo(angle, Degrees.of(1))))
      .named("Turret Aim");
}
```

## Visualization and simulation

| Method                   | Returns                | Description                                                                                                                                                                      |
| ------------------------ | ---------------------- | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `getMechanismLigament()` | `MechanismLigament2d`  | The ligament used in the Mechanism2d visualization (simulation only)                                                                                                             |
| `getMechanismRoot()`     | `MechanismRoot2d`      | The root of the Mechanism2d visualization (simulation only)                                                                                                                      |
| `getMechanismWindow()`   | `Mechanism2d`          | The Mechanism2d window (simulation only)                                                                                                                                         |
| `getPose3d()`            | `Pose3d`               | The pivot's axis, turned to its angle, relative to the robot, from its [`MechanismPositionConfig`](../config/mechanism-position-config.md). Works on the robot and in simulation |
| `getMotor()`             | `SmartMotorController` | The underlying motor controller                                                                                                                                                  |
| `getName()`              | `String`               | The pivot telemetry name                                                                                                                                                         |
| `simIterate()`           | `void`                 | Advances the pivot physics simulation by one loop and updates the visualization                                                                                                  |
| `updateTelemetry()`      | `void`                 | Publishes current state to NetworkTables                                                                                                                                         |
| `visualizationUpdate()`  | `void`                 | Syncs the Mechanism2d ligaments with the current angle and setpoint, and logs the Mechanism2d again so it moves in the Sim GUI and dashboards                                    |

Call `updateTelemetry()` from `periodic()` and `simIterate()` from `simulationPeriodic()`. `simIterate()` already calls `visualizationUpdate()`.

## Examples

The `simple_robot-2027` example includes a Turret subsystem implemented with `Pivot`:

{% @github-files/github-code-block url="https://github.com/Yet-Another-Software-Suite/YAMS/blob/master/examples/commands2/simple_robot-2027/src/main/java/first/robot/subsystems/TurretSubsystem.java" %}

Commands v3:

{% @github-files/github-code-block url="https://github.com/Yet-Another-Software-Suite/YAMS/blob/master/examples/commands3/simple_robot-2027/src/main/java/first/robot/mechanisms/TurretMechanism.java" %}

## See also

* [PivotConfig](../config/pivot-config.md)
* [SmartMotorControllerConfig](../motor-controllers/smart-motor-controller-config.md)
* [C++ Pivot](../../c++-reference/mechanisms/pivot.md)
