# MechanismPositionConfig

**Header:** `yams/mechanisms/config/MechanismPositionConfig.hpp` **Namespace:** `yams::mechanisms::config`\
**Doxygen:** [`MechanismPositionConfig`](https://doxygen.yassrobotics.com/yams/classyams_1_1mechanisms_1_1config_1_1MechanismPositionConfig.html)

`MechanismPositionConfig` places a mechanism on the robot: where it is mounted, how it is turned there, the plane it moves in, and the mechanism it is mounted on, if any. Each mechanism's `GetPose3d()` is worked out from it, to publish as a component of an AdvantageScope robot model. All builder methods return `*this` for chaining.

Give it to a mechanism with `WithMechanismPositionConfig(...)` on [ArmConfig](arm-config.md), [ElevatorConfig](elevator-config.md), [PivotConfig](pivot-config.md) or [FlyWheelConfig](flywheel-config.md).

See also: [How do I show my mechanisms on a 3D robot model?](https://yams.yassrobotics.com/documentation/how-to-simulation-and-logging/how-do-i-show-my-mechanisms-on-a-3d-robot-model)

***

## Coordinates

Positions are in meters from the robot's origin, which AdvantageScope puts at the center of the robot, on the floor: +X forward, +Y left and +Z up.

| Plane                 | At 0, the part points | A positive angle turns it | Example                           |
| --------------------- | --------------------- | ------------------------- | --------------------------------- |
| `Plane::XZ` (default) | +X, forward           | Toward +Z, up             | An arm that swings over the front |
| `Plane::YZ`           | +Y, left              | Toward +Z, up             | An arm that swings over the side  |
| `Plane::XY`           | +X, forward           | Toward +Y, left           | A turret. A `Pivot`'s default     |

A part that slides, such as an elevator's carriage, moves along its angle in the movement plane: 0 along +X (+Y in the YZ plane), 90° along +Z (+Y in the XY plane).

***

## Builder methods

| Method                                                                   | Description                                                                                                                                                |
| ------------------------------------------------------------------------ | ---------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `WithRelativePosition(const wpi::math::Translation3d& robotToMechanism)` | Where the mechanism is mounted, from the robot, or from its parent: an arm's or pivot's axis, an elevator's carriage at zero height, or a flywheel's axle. |
| `WithRelativePosition(const wpi::math::Transform3d& robotToMechanism)`   | Also sets how the mechanism is turned where it is mounted.                                                                                                 |
| `WithParent(std::function<wpi::math::Pose3d()> parentPose)`              | Mounts the mechanism on another, such as an arm on an elevator's carriage, so it moves with it. Its relative position is then from the parent's pose.      |
| `WithMovementPlane(Plane plane)`                                         | The plane the mechanism moves in. Default `Plane::XZ`.                                                                                                     |

***

## Poses

```cpp
wpi::math::Pose3d GetMountPose() const;
wpi::math::Pose3d GetRotatedPose(wpi::units::radian_t angle) const;
wpi::math::Pose3d GetTranslatedPose(wpi::units::meter_t distance, wpi::units::radian_t angle) const;
wpi::math::Rotation3d RotationInPlane(wpi::units::radian_t angle) const;
```

`GetMountPose()` is the parent's pose, or the robot's origin, moved and turned by the relative position. `GetRotatedPose(...)` turns it about the axis normal to the movement plane, and `GetTranslatedPose(...)` moves it along an angle in the movement plane without turning it.

## Getters

```cpp
std::optional<wpi::math::Translation3d> GetRelativePosition() const;
Plane GetMovementPlane() const;
```

***

## Example

```cpp
using yams::mechanisms::config::MechanismPositionConfig;

// An elevator 0.1 m behind the robot's center, its carriage 0.15 m off the floor at zero height.
m_elevatorConfig.WithMechanismPositionConfig(
    MechanismPositionConfig{}.WithRelativePosition(wpi::math::Translation3d{-0.1_m, 0_m, 0.15_m}));

// An arm on the carriage, its axis 0.2 m above it. It moves with the carriage.
m_armConfig.WithMechanismPositionConfig(
    MechanismPositionConfig{}
        .WithParent([this] { return m_elevator->GetPose3d(); })
        .WithRelativePosition(wpi::math::Translation3d{0_m, 0_m, 0.2_m}));
```
