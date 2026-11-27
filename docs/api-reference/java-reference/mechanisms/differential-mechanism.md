# DifferentialMechanism

**Package:** `yams.commands2.mechanisms` (Commands v2) or `yams.commands3.mechanisms` (Commands v3)\
**Extends:** `yams.core.mechanisms.positional.DifferentialMechanism` (which extends `SmartPositionalMechanism`)\
**Javadoc:** [`DifferentialMechanism`](https://javadocs.yassrobotics.com/yams/yams/commands2/mechanisms/DifferentialMechanism.html) (Commands v2), [`DifferentialMechanism`](https://javadocs.yassrobotics.com/yams/yams/commands3/mechanisms/DifferentialMechanism.html) (Commands v3)

`DifferentialMechanism` mixes two motor outputs to produce independent tilt and twist motion. For a position target, the left motor goes to `tilt + twist * twistRatio` and the right motor to `tilt - twist * twistRatio`, where `twistRatio` is the twist gearing from the `DifferentialMechanismConfig`. This is common in wrist mechanisms that need to tilt and rotate simultaneously.

Unlike other mechanisms, `DifferentialMechanism` takes a `DifferentialMechanismConfig` that already holds both motor controllers.

Robot code imports `DifferentialMechanism` from `yams.commands2.mechanisms` (or `yams.commands3.mechanisms`). The base class `yams.core.mechanisms.positional.DifferentialMechanism` holds the state, simulation and telemetry. Its constructor is protected, so it is not constructed directly.

```java
import yams.commands2.config.SmartMotorControllerConfig;
import yams.commands2.mechanisms.DifferentialMechanism;
import yams.core.mechanisms.config.DifferentialMechanismConfig;
```

**See also:** [DifferentialMechanismConfig](../config/differential-mechanism-config.md)

***

## Constructor

```java
DifferentialMechanism(DifferentialMechanismConfig diffConfig)
```

| Parameter    | Type                          | Description                                                                                                  |
| ------------ | ----------------------------- | ------------------------------------------------------------------------------------------------------------ |
| `diffConfig` | `DifferentialMechanismConfig` | Configuration containing both motor controllers, gearing, and starting positions for the tilt and twist axes |

Throws `DifferentialMechanismConfigurationException` if the starting tilt or twist angle or the length is not set, if the two motor controller configs were built with different `Subsystem`s (Commands v2) or `Mechanism`s (Commands v3), or, in simulation, if the moment of inertia is missing.

***

## State queries

These come from `yams.core.mechanisms.positional.DifferentialMechanism` and work outside of commands.

| Method                             | Returns                       | Description                                                                                          |
| ---------------------------------- | ----------------------------- | ---------------------------------------------------------------------------------------------------- |
| `getTiltPosition()`                | `Angle`                       | Current tilt angle (from the configured tilt supplier if set, otherwise computed from both motors)   |
| `getTwistPosition()`               | `Angle`                       | Current twist angle (from the configured twist supplier if set, otherwise computed from both motors) |
| `getLeftMotorController()`         | `SmartMotorController`        | Left motor controller                                                                                |
| `getRightMotorController()`        | `SmartMotorController`        | Right motor controller                                                                               |
| `getDifferentialMechanismConfig()` | `DifferentialMechanismConfig` | The config used to build this mechanism                                                              |

`isAtMax()` and `isAtMin()` are not supported and always throw a `RuntimeException`.

***

## Command factories

### `setPosition(Angle tilt, Angle twist)`

```java
public Command setPosition(Angle tilt, Angle twist)
public Command setPosition(Supplier<Angle> tilt, Supplier<Angle> twist)
```

Drives to target tilt and twist angles simultaneously. Supplier overloads are re-evaluated each loop. Runs indefinitely until interrupted.

| Name    | Type                        | Description         |
| ------- | --------------------------- | ------------------- |
| `tilt`  | `Angle` / `Supplier<Angle>` | Desired tilt angle  |
| `twist` | `Angle` / `Supplier<Angle>` | Desired twist angle |

**Returns:** `Command`: runs until interrupted.

***

### `run(Angle tilt, Angle twist)`

```java
public Command run(Angle tilt, Angle twist)
public Command run(Supplier<Angle> tilt, Supplier<Angle> twist)
```

Same as `setPosition` with the same arguments.

**Returns:** `Command`: runs until interrupted.

***

### `set(double twist, double tilt)`

```java
public Command set(double twist, double tilt)
```

Open loop duty cycle. Note the argument order: twist first, then tilt. The left motor gets `tilt - twist` and the right motor `tilt + twist`. The closed loop controllers are stopped while it runs and restarted when it ends.

| Name    | Type     | Description      |
| ------- | -------- | ---------------- |
| `twist` | `double` | Twist duty cycle |
| `tilt`  | `double` | Tilt duty cycle  |

**Returns:** `Command`: runs until interrupted.

***

## Triggers

`DifferentialMechanism` has no Trigger factories. Wrap the state queries in a `Trigger` yourself:

```java
new Trigger(() -> diffy.getTiltPosition().isNear(Degrees.of(45), Degrees.of(2)))
    .onTrue(intake.set(0.5));
```

***

## Commands v2 and Commands v3

Both packages have the same method names and parameters. Commands v2 returns `org.wpilib.command2.Command` and requires the shared `Subsystem` (`getSubsystem()`); Commands v3 returns `org.wpilib.command3.Command` and requires the shared `Mechanism` (`getMechanism()`). In Commands v3, use `org.wpilib.command3.Trigger` in the example above.

***

## Visualization & simulation

| Method                  | Returns       | Description                                                                                                                                                          |
| ----------------------- | ------------- | -------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `simIterate()`          | `void`        | Advances mechanism physics simulation one loop and updates the visualization; call in `simulationPeriodic()`                                                         |
| `updateTelemetry()`     | `void`        | Publishes current state to NetworkTables; call in `periodic()`                                                                                                       |
| `visualizationUpdate()` | `void`        | Updates the Mechanism2d visualization (called by `simIterate()`), and logs the Mechanism2d again so it moves in the Sim GUI and dashboards                           |
| `getMechanismWindow()`  | `Mechanism2d` | The Mechanism2d window (simulation only)                                                                                                                             |
| `getPose3d()`           | `Pose3d`      | The mechanism's axis, tilted, then twisted about its own length, relative to the robot, from its [`MechanismPositionConfig`](../config/mechanism-position-config.md) |
| `getName()`             | `String`      | The mechanism telemetry name                                                                                                                                         |

***

## Examples

{% @github-files/github-code-block url="https://github.com/Yet-Another-Software-Suite/YAMS/blob/master/examples/commands2/simple_robot-2027/src/main/java/first/robot/subsystems/DiffyMechSubsystem.java" %}

Commands v3:

{% @github-files/github-code-block url="https://github.com/Yet-Another-Software-Suite/YAMS/blob/master/examples/commands3/simple_robot-2027/src/main/java/first/robot/mechanisms/DiffyMechanism.java" %}
