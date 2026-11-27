# DoubleJointedArm

**Package:** `yams.commands2.mechanisms` (Commands v2) or `yams.commands3.mechanisms` (Commands v3)\
**Extends:** `yams.core.mechanisms.positional.DoubleJointedArm` (which extends `SmartPositionalMechanism`)\
**Javadoc:** [`DoubleJointedArm`](https://javadocs.yassrobotics.com/yams/yams/commands2/mechanisms/DoubleJointedArm.html) (Commands v2), [`DoubleJointedArm`](https://javadocs.yassrobotics.com/yams/yams/commands3/mechanisms/DoubleJointedArm.html) (Commands v3)

`DoubleJointedArm` controls a serial two-joint arm: a lower link (shoulder) and an upper link (elbow). Each joint has its own `SmartMotorController`. The class provides both direct joint angle control and inverse kinematics to a Cartesian target.

Robot code imports `DoubleJointedArm` from `yams.commands2.mechanisms` (or `yams.commands3.mechanisms`). The base class `yams.core.mechanisms.positional.DoubleJointedArm` holds the state, kinematics, simulation and telemetry. Its constructor is protected, so it is not constructed directly.

```java
import yams.commands2.config.SmartMotorControllerConfig;
import yams.commands2.mechanisms.DoubleJointedArm;
import yams.core.mechanisms.config.ArmConfig;
```

**See also:** [ArmConfig](../config/arm-config.md)

***

## Constructor

```java
DoubleJointedArm(
    ArmConfig lowerConfig, SmartMotorController lowerSMC,
    ArmConfig upperConfig, SmartMotorController upperSMC
)
```

| Parameter     | Type                   | Description                                                        |
| ------------- | ---------------------- | ------------------------------------------------------------------ |
| `lowerConfig` | `ArmConfig`            | Configuration for the lower (shoulder) joint, including arm length |
| `lowerSMC`    | `SmartMotorController` | Motor controller for the lower joint                               |
| `upperConfig` | `ArmConfig`            | Configuration for the upper (elbow) joint, including arm length    |
| `upperSMC`    | `SmartMotorController` | Motor controller for the upper joint                               |

Throws `DoubleJointedArmConfigurationException` if:

* either `SmartMotorControllerConfig` has no `withStartingPosition`,
* either `ArmConfig` has no `withLength`,
* the two motor controller configs were built with different `Subsystem`s (Commands v2) or `Mechanism`s (Commands v3),
* in simulation, either `ArmConfig` is missing `withHardLimits`.

***

## State queries

These come from `yams.core.mechanisms.positional.DoubleJointedArm` and work outside of commands.

| Method                                                            | Returns                | Description                                                               |
| ----------------------------------------------------------------- | ---------------------- | ------------------------------------------------------------------------- |
| `getLowerAngle()`                                                 | `Angle`                | Current lower joint (shoulder) angle                                      |
| `getUpperAngle()`                                                 | `Angle`                | Current upper joint (elbow) angle                                         |
| `getPosition()`                                                   | `Translation2d`        | Current end-effector position from forward kinematics                     |
| `getAnglesForPosition(Translation2d translation, boolean invert)` | `Pair<Angle, Angle>`   | Inverse kinematics: shoulder angle then elbow angle for a target position |
| `isNear(Translation2d target, Distance tolerance)`                | `boolean`              | `true` when the end-effector is within `tolerance` of `target`            |
| `getLowerMotorController()`                                       | `SmartMotorController` | Lower (shoulder) motor controller                                         |
| `getUpperMotorController()`                                       | `SmartMotorController` | Upper (elbow) motor controller                                            |
| `getLowerArmConfig()`                                             | `ArmConfig`            | Lower (shoulder) `ArmConfig`                                              |
| `getUpperArmConfig()`                                             | `ArmConfig`            | Upper (elbow) `ArmConfig`                                                 |

`isAtMax()` and `isAtMin()` are not supported and always throw a `RuntimeException`.

***

## Command factories

### `setAngle(Angle lowerAngle, Angle upperAngle)`

```java
public Command setAngle(Angle lowerAngle, Angle upperAngle)
```

Drives both joints to the target angles simultaneously. Pass `null` to leave a joint alone. Runs indefinitely until interrupted.

| Name         | Type    | Description                                 |
| ------------ | ------- | ------------------------------------------- |
| `lowerAngle` | `Angle` | Target angle for the lower (shoulder) joint |
| `upperAngle` | `Angle` | Target angle for the upper (elbow) joint    |

**Returns:** `Command`: runs until interrupted.

***

### `setPosition(Translation2d translation, boolean invert)`

```java
public Command setPosition(Translation2d translation, boolean invert)
```

Uses inverse kinematics to move the end-effector to `translation` (X away from the root, Y up). The angles are computed when the command starts, then it behaves like `setAngle`. Runs indefinitely until interrupted.

| Name          | Type            | Description                   |
| ------------- | --------------- | ----------------------------- |
| `translation` | `Translation2d` | Desired end-effector position |
| `invert`      | `boolean`       | Invert the elbow direction    |

**Returns:** `Command`: runs until interrupted.

***

### `run`

```java
public Command run(Translation2d translation2d, boolean invert)
public Command run(Supplier<Translation2d> translation, Supplier<Boolean> invert)
```

Like `setPosition`, but recomputes the inverse kinematics every loop, so the target can change while the command runs.

**Returns:** `Command`: runs until interrupted.

***

### `runTo`

```java
public Command runTo(Translation2d translation2d, boolean invert, Distance tolerance)
public Command runTo(Supplier<Translation2d> translation, Supplier<Boolean> invert, Distance tolerance)
```

Moves to the target with inverse kinematics, ending when the end-effector is within `tolerance` of the target position. If the subsystem has a default command it takes over afterwards.

| Name          | Type                                        | Description                                                                            |
| ------------- | ------------------------------------------- | -------------------------------------------------------------------------------------- |
| `translation` | `Translation2d` / `Supplier<Translation2d>` | Desired end-effector position                                                          |
| `invert`      | `boolean` / `Supplier<Boolean>`             | Invert the elbow direction                                                             |
| `tolerance`   | `Distance`                                  | Maximum acceptable distance between current and target position for the command to end |

**Returns:** `Command`: ends when within tolerance.

***

### `set(Double lowerDutycycle, Double upperDutycycle)`

```java
public Command set(Double lowerDutycycle, Double upperDutycycle)
```

Open loop duty cycle for each joint. Pass `null` to leave a joint alone. The closed loop controller of each driven joint is stopped while it runs and restarted when it ends.

**Returns:** `Command`: runs until interrupted.

***

## Triggers

`DoubleJointedArm` has no Trigger factories. Wrap the boolean check in a `Trigger` yourself:

```java
new Trigger(() -> jointedArm.isNear(new Translation2d(0.5, 0.8), Inches.of(1)))
    .onTrue(intake.set(0.5));
```

***

## Commands v2 and Commands v3

Both packages have the same method names and parameters. Commands v2 returns `org.wpilib.command2.Command` and requires the shared `Subsystem` (`getSubsystem()`); Commands v3 returns `org.wpilib.command3.Command` and requires the shared `Mechanism` (`getMechanism()`). In Commands v3, `setPosition` is a requirement-free command that `await`s `setAngle`, and the `Trigger` in the example above is `org.wpilib.command3.Trigger`.

***

## Visualization & simulation

| Method                  | Returns         | Description                                                                                                                                                |
| ----------------------- | --------------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `simIterate()`          | `void`          | Advances joint physics simulation one loop and updates the visualization; call in `simulationPeriodic()`                                                   |
| `updateTelemetry()`     | `void`          | Publishes current state to NetworkTables; call in `periodic()`                                                                                             |
| `visualizationUpdate()` | `void`          | Updates the Mechanism2d visualization (called by `simIterate()`), and logs the Mechanism2d again so it moves in the Sim GUI and dashboards                 |
| `getMechanismWindow()`  | `Mechanism2d`   | The Mechanism2d window (simulation only)                                                                                                                   |
| `getPose3d()`           | `Pose3d`        | The lower arm's axis, turned to its angle, relative to the robot, from the lower arm's [`MechanismPositionConfig`](../config/mechanism-position-config.md) |
| `getUpperPose3d()`      | `Pose3d`        | The upper arm at the end of the lower arm, turned to its angle                                                                                             |
| `getPosition()`         | `Translation2d` | The end of the upper arm, from the lower arm's axis                                                                                                        |
| `getName()`             | `String`        | The mechanism telemetry name                                                                                                                               |

***

## Examples

{% @github-files/github-code-block url="https://github.com/Yet-Another-Software-Suite/YAMS/blob/master/examples/commands2/simple_robot-2027/src/main/java/first/robot/subsystems/DoubleJointedArmSubsystem.java" %}

Commands v3:

{% @github-files/github-code-block url="https://github.com/Yet-Another-Software-Suite/YAMS/blob/master/examples/commands3/simple_robot-2027/src/main/java/first/robot/mechanisms/DoubleJointedArmMechanism.java" %}
