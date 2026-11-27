# FlyWheel

**Package:** `yams.commands2.mechanisms` (Commands v2) or `yams.commands3.mechanisms` (Commands v3)\
**Extends:** `yams.core.mechanisms.velocity.FlyWheel` (which extends `SmartVelocityMechanism`)\
**Implements:** `CommandMechanism`\
**Javadoc:** [`FlyWheel`](https://javadocs.yassrobotics.com/yams/yams/commands2/mechanisms/FlyWheel.html) (Commands v2), [`FlyWheel`](https://javadocs.yassrobotics.com/yams/yams/commands3/mechanisms/FlyWheel.html) (Commands v3)

`FlyWheel` controls a continuously spinning wheel or roller at a target angular velocity. It is not position-controlled; hard limits do not apply. Use `FlyWheel` for shooters and also for intakes, rollers, feeders and indexers, even when they only run open loop with `set(double)`.

Robot code imports `FlyWheel` from `yams.commands2.mechanisms` (or `yams.commands3.mechanisms`). The base class `yams.core.mechanisms.velocity.FlyWheel` holds the state, simulation and telemetry. Its constructor is protected, so it is not constructed directly.

```java
import yams.commands2.config.SmartMotorControllerConfig;
import yams.commands2.mechanisms.FlyWheel;
import yams.core.mechanisms.config.FlyWheelConfig;
```

**See also:** [FlyWheelConfig](../config/flywheel-config.md) | [C++ FlyWheel](../../c++-reference/mechanisms/flywheel.md)

***

## Constructor

```java
FlyWheel(FlyWheelConfig config, SmartMotorController smc)
```

| Parameter | Type                   | Description                                                      |
| --------- | ---------------------- | ---------------------------------------------------------------- |
| `config`  | `FlyWheelConfig`       | Flywheel configuration (diameter, telemetry, simulation options) |
| `smc`     | `SmartMotorController` | Motor controller driving the flywheel                            |

The commands require the `Subsystem` (Commands v2) or `Mechanism` (Commands v3) passed to the `SmartMotorControllerConfig` constructor, so build the motor controller with `yams.commands2.config.SmartMotorControllerConfig` or `yams.commands3.config.SmartMotorControllerConfig` to match the `FlyWheel` you import. The simulation uses the moment of inertia from `SmartMotorControllerConfig.withMomentOfInertia`.

***

## State queries

These come from `yams.core.mechanisms.velocity.FlyWheel` and work outside of commands.

| Method                                                  | Returns                     | Description                                                                       |
| ------------------------------------------------------- | --------------------------- | --------------------------------------------------------------------------------- |
| `getSpeed()`                                            | `AngularVelocity`           | Current wheel velocity from the encoder                                           |
| `getLinearVelocity()`                                   | `LinearVelocity`            | Current surface speed. Throws `FlyWheelConfigurationException` without a diameter |
| `getMechanismSetpointVelocity()`                        | `Optional<AngularVelocity>` | The current closed loop velocity setpoint, if any                                 |
| `isNear(AngularVelocity speed, AngularVelocity within)` | `boolean`                   | `true` when the current speed is within `within` of `speed`                       |
| `isGte(AngularVelocity speed)`                          | `boolean`                   | `true` when the current speed is greater than or equal to `speed`                 |
| `isLte(AngularVelocity speed)`                          | `boolean`                   | `true` when the current speed is less than or equal to `speed`                    |
| `isBetween(AngularVelocity start, AngularVelocity end)` | `boolean`                   | `true` when the current speed is in `[start, end]`                                |
| `getShooterConfig()`                                    | `FlyWheelConfig`            | The `FlyWheelConfig` used to build this flywheel                                  |
| `getMotorController()`                                  | `SmartMotorController`      | The underlying motor controller                                                   |

***

## Command factories

### `run(AngularVelocity velocity)`

```java
public Command run(AngularVelocity velocity)
```

Spins the wheel at `velocity` indefinitely using closed loop velocity control.

| Name       | Type              | Description            |
| ---------- | ----------------- | ---------------------- |
| `velocity` | `AngularVelocity` | Desired wheel velocity |

**Returns:** `Command`: runs until interrupted.

***

### `run(LinearVelocity speed)`

```java
public Command run(LinearVelocity speed)
```

Spins the wheel at a surface speed indefinitely. The speed is converted with the `FlyWheelConfig` diameter.

| Name    | Type             | Description           |
| ------- | ---------------- | --------------------- |
| `speed` | `LinearVelocity` | Desired surface speed |

**Returns:** `Command`: runs until interrupted. Throws `FlyWheelConfigurationException` if the diameter is not configured.

***

### `run(Supplier<T> velocity)`

```java
public <T> Command run(Supplier<T> velocity)
```

Spins the wheel at the supplied velocity, re-evaluated each loop. `T` must be `AngularVelocity` or `LinearVelocity`; the type is checked once, when the command is created.

| Name       | Type          | Description                                            |
| ---------- | ------------- | ------------------------------------------------------ |
| `velocity` | `Supplier<T>` | Supplier of an `AngularVelocity` or a `LinearVelocity` |

**Returns:** `Command`: runs until interrupted. Throws `IllegalArgumentException` if the supplied value is neither type, and `FlyWheelConfigurationException` for a `LinearVelocity` without a configured diameter.

***

### `runTo(AngularVelocity velocity, AngularVelocity tolerance)`

```java
public Command runTo(AngularVelocity velocity, AngularVelocity tolerance)
public Command runTo(Supplier<AngularVelocity> velocity, AngularVelocity tolerance)
public Command runTo(LinearVelocity velocity, LinearVelocity tolerance)
public Command runTo(Supplier<LinearVelocity> velocity, LinearVelocity tolerance)
```

Sets the velocity setpoint and ends once the wheel is within `tolerance` (debounced 0.1 s). The setpoint stays active after the command ends, unless a default command replaces it. Supplier overloads read the supplier once when the command starts. `LinearVelocity` overloads need the `FlyWheelConfig` diameter.

| Name        | Type                                 | Description                                      |
| ----------- | ------------------------------------ | ------------------------------------------------ |
| `velocity`  | `AngularVelocity` / `LinearVelocity` | Desired wheel velocity                           |
| `tolerance` | `AngularVelocity` / `LinearVelocity` | Acceptable speed error for the command to finish |

**Returns:** `Command`: ends when the wheel reaches the target speed.

***

### `set(double dutycycle)`

```java
public Command set(double dutycycle)
public Command set(Supplier<Double> dutycycle)
```

Open loop duty cycle, runs until interrupted. The closed loop controller is stopped while it runs and restarted when it ends. This is the usual command for open loop rollers and intakes.

| Name        | Type                          | Description                            |
| ----------- | ----------------------------- | -------------------------------------- |
| `dutycycle` | `double` / `Supplier<Double>` | Motor output in the range \[-1.0, 1.0] |

**Returns:** `Command`

***

### `setVoltage(Voltage volts)`

```java
public Command setVoltage(Voltage volts)
public Command setVoltage(Supplier<Voltage> volts)
```

Open loop voltage, runs until interrupted. The closed loop controller is stopped while it runs and restarted when it ends.

| Name    | Type                            | Description           |
| ------- | ------------------------------- | --------------------- |
| `volts` | `Voltage` / `Supplier<Voltage>` | Applied motor voltage |

**Returns:** `Command`

***

## Triggers

### `near(AngularVelocity speed, AngularVelocity within)`

```java
public Trigger near(AngularVelocity speed, AngularVelocity within)
```

Active when the current wheel speed is within `within` of `speed`.

| Name     | Type              | Description                  |
| -------- | ----------------- | ---------------------------- |
| `speed`  | `AngularVelocity` | Reference speed              |
| `within` | `AngularVelocity` | Maximum acceptable deviation |

**Returns:** `Trigger`

{% hint style="info" %}
The `isNear(...)` Trigger factory was renamed to `near(...)`. `isNear(...)` still exists, but returns a `boolean`. There is no `atSpeed()` trigger; use `near(...)` with your target speed.
{% endhint %}

```java
shooter.near(RPM.of(3000), RPM.of(50)).onTrue(feeder.set(0.8));
```

***

### `gte(AngularVelocity speed)`

```java
public Trigger gte(AngularVelocity speed)
```

Active when the current wheel speed is greater than or equal to `speed`.

**Returns:** `Trigger`

***

### `lte(AngularVelocity speed)`

```java
public Trigger lte(AngularVelocity speed)
```

Active when the current wheel speed is less than or equal to `speed`.

**Returns:** `Trigger`

***

### `between(AngularVelocity start, AngularVelocity end)`

```java
public Trigger between(AngularVelocity start, AngularVelocity end)
```

Active when the current wheel speed is in the inclusive range `[start, end]`.

| Name    | Type              | Description              |
| ------- | ----------------- | ------------------------ |
| `start` | `AngularVelocity` | Lower bound of the range |
| `end`   | `AngularVelocity` | Upper bound of the range |

**Returns:** `Trigger`

***

## Commands v2 and Commands v3

Both packages have the same method names and parameters. The difference is the return types and what the commands require:

|                | Commands v2 (`yams.commands2.mechanisms.FlyWheel`)            | Commands v3 (`yams.commands3.mechanisms.FlyWheel`)            |
| -------------- | ------------------------------------------------------------- | ------------------------------------------------------------- |
| `Command` type | `org.wpilib.command2.Command`                                 | `org.wpilib.command3.Command`                                 |
| `Trigger` type | `org.wpilib.command2.button.Trigger`                          | `org.wpilib.command3.Trigger`                                 |
| Requirement    | The `Subsystem` from `getSubsystem()`                         | The `Mechanism` from `getMechanism()`                         |
| Motor config   | `yams.commands2.config.SmartMotorControllerConfig(Subsystem)` | `yams.commands3.config.SmartMotorControllerConfig(Mechanism)` |

In Commands v3 the commands are coroutines, so your own commands can `await` or `fork` them:

```java
public Command hold(AngularVelocity speed)
{
  return run(coroutine -> coroutine.await(shooter.run(speed)))
      .withPriority(Command.LOWEST_PRIORITY)
      .named("Shooter Hold");
}
```

***

## Visualization and simulation

| Method                   | Returns                | Description                                                                                                                                                                                    |
| ------------------------ | ---------------------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `getMotor()`             | `SmartMotorController` | The underlying motor controller                                                                                                                                                                |
| `getMechanismLigament()` | `MechanismLigament2d`  | The ligament used in the Mechanism2d visualization (simulation only)                                                                                                                           |
| `getMechanismRoot()`     | `MechanismRoot2d`      | The root of the Mechanism2d visualization (simulation only)                                                                                                                                    |
| `getMechanismWindow()`   | `Mechanism2d`          | The Mechanism2d window (simulation only)                                                                                                                                                       |
| `getPose3d()`            | `Pose3d`               | The flywheel's axle, turned by the wheel's position, relative to the robot, from its [`MechanismPositionConfig`](../config/mechanism-position-config.md). Works on the robot and in simulation |
| `getName()`              | `String`               | The flywheel telemetry name                                                                                                                                                                    |
| `simIterate()`           | `void`                 | Advances the flywheel physics simulation by one loop and updates the visualization                                                                                                             |
| `updateTelemetry()`      | `void`                 | Publishes the current flywheel state to NetworkTables                                                                                                                                          |
| `visualizationUpdate()`  | `void`                 | Updates the Mechanism2d visualization, and logs the Mechanism2d again so it moves in the Sim GUI and dashboards                                                                                |

Call `updateTelemetry()` from `periodic()` and `simIterate()` from `simulationPeriodic()`. `simIterate()` already calls `visualizationUpdate()`.

***

## Examples

{% @github-files/github-code-block url="https://github.com/Yet-Another-Software-Suite/YAMS/blob/master/examples/commands2/simple_robot-2027/src/main/java/first/robot/subsystems/ShooterSubsystem.java" %}

Commands v3:

{% @github-files/github-code-block url="https://github.com/Yet-Another-Software-Suite/YAMS/blob/master/examples/commands3/simple_robot-2027/src/main/java/first/robot/mechanisms/ShooterMechanism.java" %}
