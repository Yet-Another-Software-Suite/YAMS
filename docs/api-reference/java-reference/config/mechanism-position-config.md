# MechanismPositionConfig

**Package:** `yams.core.mechanisms.config`\
**Javadoc:** [`MechanismPositionConfig`](https://javadocs.yassrobotics.com/yams/yams/core/mechanisms/config/MechanismPositionConfig.html)

`MechanismPositionConfig` places a mechanism on the robot: where it is mounted, how it is turned there, the plane it moves in, and the mechanism it is mounted on, if any. Each mechanism's `getPose3d()` is worked out from it, to publish as a component of an AdvantageScope robot model. It also sizes and places the mechanism's `Mechanism2d` in simulation. All builder methods return the `MechanismPositionConfig` for chaining.

Give it to a mechanism with `withMechanismPositionConfig(...)` on [ArmConfig](arm-config.md), [ElevatorConfig](elevator-config.md), [PivotConfig](pivot-config.md), [FlyWheelConfig](flywheel-config.md) or [DifferentialMechanismConfig](differential-mechanism-config.md).

See also: [How do I show my mechanisms on a 3D robot model?](https://yams.yassrobotics.com/documentation/how-to-simulation-and-logging/how-do-i-show-my-mechanisms-on-a-3d-robot-model)

***

## Coordinates

Positions are in meters from the robot's origin, which AdvantageScope puts at the center of the robot, on the floor: +X forward, +Y left and +Z up.

A part that turns, such as an arm, a pivot or a flywheel, turns about the axis normal to its movement plane:

| Plane          | At 0, the part points | A positive angle turns it | Example                           |
| -------------- | --------------------- | ------------------------- | --------------------------------- |
| `XZ` (default) | +X, forward           | Toward +Z, up             | An arm that swings over the front |
| `YZ`           | +Y, left              | Toward +Z, up             | An arm that swings over the side  |
| `XY`           | +X, forward           | Toward +Y, left           | A turret. A `Pivot`'s default     |

A part that slides, such as an elevator's carriage, moves along its angle in the movement plane: 0 along +X (+Y in the YZ plane), 90° along +Z (+Y in the XY plane).

***

## Constructor

```java
MechanismPositionConfig()
```

The movement plane is XZ. A `PivotConfig` gives its pivot an XY plane unless you give it your own `MechanismPositionConfig`.

***

## Builder methods

| Method                                                 | Parameters                                | Description                                                                                                                                                |
| ------------------------------------------------------ | ----------------------------------------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `withRelativePosition(Translation3d robotToMechanism)` | `robotToMechanism`: where it is mounted   | Where the mechanism is mounted, from the robot, or from its parent: an arm's or pivot's axis, an elevator's carriage at zero height, or a flywheel's axle. |
| `withRelativePosition(Transform3d robotToMechanism)`   | `robotToMechanism`: where, and how turned | Also sets how the mechanism is turned where it is mounted, such as an arm on the side of the robot facing left.                                            |
| `withParent(Supplier<Pose3d> parentPose)`              | `parentPose`: the pose it is mounted on   | Mounts the mechanism on another, such as an arm on `elevator::getPose3d`, so it moves with it. Its relative position is then from the parent's pose.       |
| `withMovementPlane(Plane plane)`                       | `plane`: `XZ`, `YZ` or `XY`               | The plane the mechanism moves in. Default `XZ`.                                                                                                            |
| `withMaxRobotLength(Distance robotLength)`             | `robotLength`: the robot's length         | Sizes the `Mechanism2d` window. Defaults to twice the mechanism's length.                                                                                  |
| `withMaxRobotHeight(Distance robotHeight)`             | `robotHeight`: the robot's height         | Sizes the `Mechanism2d` window. Defaults to twice the mechanism's length.                                                                                  |

***

## Poses

These do the math for each mechanism's `getPose3d()`, and can place a part YAMS has no mechanism for.

| Method                                              | Returns      | Description                                                                                                          |
| --------------------------------------------------- | ------------ | -------------------------------------------------------------------------------------------------------------------- |
| `getMountPose()`                                    | `Pose3d`     | Where the mechanism is mounted: its parent's pose, or the robot's origin, moved and turned by its relative position. |
| `getRotatedPose(Angle angle)`                       | `Pose3d`     | The mount pose, turned by the angle about the axis normal to the movement plane.                                     |
| `getTranslatedPose(Distance distance, Angle angle)` | `Pose3d`     | The mount pose, moved by the distance along the angle in the movement plane, and not turned.                         |
| `rotationInPlane(Angle angle)`                      | `Rotation3d` | The rotation of the angle about the axis normal to the movement plane.                                               |

***

## Getters

| Method                  | Returns                   | Description                                          |
| ----------------------- | ------------------------- | ---------------------------------------------------- |
| `getRelativePosition()` | `Optional<Translation3d>` | Where the mechanism is mounted, or empty if not set. |
| `getMovementPlane()`    | `Plane`                   | The plane the mechanism moves in.                    |

***

## Example

```java
// An elevator 0.1 m behind the robot's center, its carriage 0.15 m off the floor at zero height.
Elevator elevator = new Elevator(new ElevatorConfig()
    // ...
    .withMechanismPositionConfig(new MechanismPositionConfig()
        .withRelativePosition(new Translation3d(-0.1, 0, 0.15))), elevatorMotor);

// An arm on the carriage, its axis 0.2 m above it. It moves with the carriage.
Arm arm = new Arm(new ArmConfig()
    // ...
    .withMechanismPositionConfig(new MechanismPositionConfig()
        .withParent(elevator::getPose3d)
        .withRelativePosition(new Translation3d(0, 0, 0.2))), armMotor);

Telemetry.log("Robot/Components", new Pose3d[] {elevator.getPose3d(), arm.getPose3d()}, Pose3d.struct);
```
