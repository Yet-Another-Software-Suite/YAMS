---
icon: share-nodes
---

# Shared options

`ArmConfig`, `ElevatorConfig`, `PivotConfig` and `FlyWheelConfig` share these options. They work the same way on each.

## `withTelemetry`

Names the mechanism and chooses how much it publishes. This is the mechanism's own name: the motor inside it has its own [`withTelemetry`](../editor/config/motor-basics.md#withtelemetry).

<figure><img src="../../.gitbook/assets/telemetry-tree.png" alt="Two NetworkTables trees for a mechanism: the read-only values under NT Mechanisms and the tunable values under NT Tuning"><figcaption><p>Where a mechanism's values appear in NetworkTables.</p></figcaption></figure>

```java
.withTelemetry("Arm", TelemetryVerbosity.HIGH)
```

The name is the `<Mechanism>` part of every NetworkTables path, and the name [live tuning](../../how-to-tuning/live-tuning/) uses. [Telemetry](../../understanding/telemetry.md) explains the paths and verbosity levels.

**Used in:** simulation and on the real robot.

**Available on:** `ArmConfig`, `ElevatorConfig`, `PivotConfig`, `FlyWheelConfig`.

## `withHardLimits`

Where the mechanism physically cannot go any further.

<figure><img src="../../.gitbook/assets/mech-hard-limits.png" alt="A bar of a mechanism&#x27;s travel. The hard limits at each end are physical stops. The soft limits sit a little inside them and are where the motor controller keeps the mechanism. withHardLimits is set on the mechanism config and the simulated mechanism cannot pass them. withSoftLimits is set on the motor config and the real motor controller will not drive past them."><figcaption><p>Hard limits are physical stops. Soft limits sit inside them.</p></figcaption></figure>

```java
new ArmConfig().withHardLimits(Degrees.of(-30), Degrees.of(110));
new ElevatorConfig().withHardLimits(Meters.of(0), Meters.of(1.3));
new PivotConfig().withHardLimits(Degrees.of(-180), Degrees.of(180));
```

The simulated mechanism stops dead at them, as the real one would against metal. They do not stop the real motor: that is what [`withSoftLimits`](../editor/config/limits-and-protection.md#withsoftlimits) is for. Set both, with the soft limits a little inside the hard ones.

**Used in:** simulation, where they are **required** for an `Arm`, `Elevator` or `Pivot` to be created, and on the real robot by `max()`, `min()`, `isAtMax()` and `isAtMin()` when no soft limit is set. On the real robot they never stop the motor.

**Available on:** `ArmConfig` and `PivotConfig` (angles), `ElevatorConfig` (distances). **Not on** `FlyWheelConfig`: a flywheel has no end to its travel.

## `withSimColor`

The color of the mechanism's line in its `Mechanism2d` window. The default is orange.

```java
.withSimColor(new Color8Bit(Color.BLUE))
```

**Used in:** simulation only.

**Available on:** `ArmConfig`, `ElevatorConfig`, `PivotConfig`, `FlyWheelConfig`.

## `withMechanismPositionConfig`

Places the mechanism on a drawing of the robot, so several mechanisms line up when you view them together.

<figure><img src="../../.gitbook/assets/mech-window.png" alt="A Mechanism2d window showing a robot outline, the mechanism as an orange line and its setpoint as a grey line. withSimColor sets the color of the mechanism&#x27;s line. withMechanismPositionConfig sets where it sits on the robot with withRelativePosition, the size of the window with withMaxRobotLength and withMaxRobotHeight, and the plane it moves in with withMovementPlane. None of these change how the mechanism moves."><figcaption><p>The Mechanism2d window, and what each option changes in it.</p></figcaption></figure>

```java
.withMechanismPositionConfig(new MechanismPositionConfig()
    .withRelativePosition(new Translation3d(0.2, 0.0, 0.3)) // 20 cm forward, 30 cm up
    .withMaxRobotLength(Meters.of(0.85))
    .withMaxRobotHeight(Meters.of(1.2))
    .withMovementPlane(MechanismPositionConfig.Plane.XZ))
```

| Option                                     | What it sets                                                                            |
| ------------------------------------------ | --------------------------------------------------------------------------------------- |
| `withRelativePosition`                     | Where the mechanism's root is on the robot.                                             |
| `withMaxRobotLength`, `withMaxRobotHeight` | The size of the window. Without them it is twice the mechanism's length.                |
| `withMovementPlane`                        | The plane the mechanism moves in: `XZ` (seen from the side, the default), `YZ` or `XY`. |

**Used in:** simulation only. It changes the drawing and the mechanism's reported 3D position, never how the mechanism moves.

**Available on:** `ArmConfig`, `ElevatorConfig`, `PivotConfig`, `FlyWheelConfig`.

## Code reference

* [`MechanismPositionConfig`](https://github.com/Yet-Another-Software-Suite/YAMS/blob/master/yams/java/yams/core/mechanisms/config/MechanismPositionConfig.java)
