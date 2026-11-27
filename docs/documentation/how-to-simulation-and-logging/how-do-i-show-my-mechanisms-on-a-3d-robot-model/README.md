---
description: >-
  Publish each mechanism's Pose3d, so AdvantageScope moves the matching part of
  your robot's CAD model as the real mechanism moves.
icon: cube
---

# How do I show my mechanisms on a 3D robot model?

AdvantageScope can show your robot's CAD model on the 3D field, with each mechanism moving as the real one does: the arm swinging, the elevator rising and the turret turning. Each moving part is a separate model, called a component, and your robot code publishes a `Pose3d` for each one. Every YAMS mechanism gives you that pose with `getPose3d()`.

{% hint style="info" %}
`getPose3d()` is in YAMS releases after v2026.10.15. It replaces `getRelativeMechanismPosition()`.
{% endhint %}

{% hint style="success" %}
Only want to see your mechanisms in 3D in simulation, without a CAD model? In simulation, every YAMS mechanism publishes a `Mechanism2d` under `Mechanisms/<name>/mechanism`, and AdvantageScope can [show a `Mechanism2d` on the 3D field](https://docs.advantagescope.org/tab-reference/3d-field#2d-mechanisms) with no setup. `getPose3d()` also works on the real robot.
{% endhint %}

AdvantageScope's video walks through setting up a custom robot model and its components:

{% embed url="https://www.youtube.com/watch?v=unX1PsPi0VA" %}
Configuring Custom Robot Models for AdvantageScope
{% endembed %}

[AdvantageScope's custom assets page](https://docs.advantagescope.org/more-features/custom-assets) has the full reference.

{% stepper %}
{% step %}
### Tell YAMS where each mechanism is

Give each mechanism a `MechanismPositionConfig`: where it is mounted on the robot, and the plane it moves in. Positions are in meters from the robot's origin, which AdvantageScope puts at the center of the robot, **on the floor**: +X forward, +Y left and +Z up.

| Mechanism          | Where it is mounted         | Its pose                                                                                                                                                                                                                        |
| ------------------ | --------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `Arm`, `Pivot`     | The axis it turns about     | The axis, turned to its angle                                                                                                                                                                                                   |
| `Elevator`         | The carriage at zero height | The carriage, moved by its height along the elevator's angle. For each stage of a multi-stage elevator, see [How do I show an elevator's stages on a 3D robot model?](how-do-i-show-an-elevators-stages-on-a-3d-robot-model.md) |
| `FlyWheel`         | Its axle                    | The axle, turned by the wheel's position                                                                                                                                                                                        |
| `DoubleJointedArm` | The lower arm's axis        | `getPose3d()` for the lower arm, `getUpperPose3d()` for the upper arm at the elbow                                                                                                                                              |

A part turns in its movement plane: **XZ** by default (it points forward at 0° and a positive angle turns it up, like an arm over the front), **YZ** for an arm over the side (it points left at 0°), or **XY** for a turret (it points forward at 0° and a positive angle turns it left). A `Pivot` is in the XY plane unless you give it a `MechanismPositionConfig`.

{% tabs %}
{% tab title="Java" %}
One YAMS mechanism per subsystem, so these are fragments of three subsystems. Each one exposes its mechanism's pose with a `getPose3d()` method.

{% code title="ElevatorSubsystem.java (fragment)" %}
```java
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Translation3d;
import yams.core.mechanisms.config.MechanismPositionConfig;

// An elevator 0.1 m behind the robot's center, its carriage 0.15 m off the floor at zero height.
private final Elevator elevator = new Elevator(new ElevatorConfig()
    // ... the rest of the config
    .withMechanismPositionConfig(new MechanismPositionConfig()
        .withRelativePosition(new Translation3d(-0.1, 0, 0.15))), elevatorMotor);

public Pose3d getPose3d() {
  return elevator.getPose3d();
}
```
{% endcode %}

{% code title="ArmSubsystem.java (fragment)" %}
```java
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Translation3d;
import yams.core.mechanisms.config.MechanismPositionConfig;

private final Arm arm;

// The arm rides on the elevator's carriage, so it is given the elevator subsystem.
public ArmSubsystem(ElevatorSubsystem elevator) {
  // Its axis is 0.2 m above the carriage's reference point, and it moves with the carriage.
  arm = new Arm(new ArmConfig()
      // ... the rest of the config
      .withMechanismPositionConfig(new MechanismPositionConfig()
          .withParent(elevator::getPose3d)
          .withRelativePosition(new Translation3d(0, 0, 0.2))), armMotor);
}

public Pose3d getPose3d() {
  return arm.getPose3d();
}
```
{% endcode %}

{% code title="TurretSubsystem.java (fragment)" %}
```java
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.math.geometry.Translation3d;
import yams.core.mechanisms.config.MechanismPositionConfig;
import yams.core.mechanisms.config.MechanismPositionConfig.Plane;

// A turret 0.3 m off the floor, turned 180° so that its 0° faces the back of the robot.
private final Pivot turret = new Pivot(new PivotConfig()
    // ... the rest of the config
    .withMechanismPositionConfig(new MechanismPositionConfig()
        .withMovementPlane(Plane.XY)
        .withRelativePosition(new Transform3d(new Translation3d(0, 0, 0.3), new Rotation3d(0, 0, Math.PI)))), turretMotor);

public Pose3d getPose3d() {
  return turret.getPose3d();
}
```
{% endcode %}
{% endtab %}

{% tab title="C++" %}
```cpp
using yams::mechanisms::config::MechanismPositionConfig;

// An elevator 0.1 m behind the robot's center, its carriage 0.15 m off the floor at zero height.
m_elevatorConfig.WithMechanismPositionConfig(
    MechanismPositionConfig{}.WithRelativePosition(wpi::math::Translation3d{-0.1_m, 0_m, 0.15_m}));

// An arm on the elevator's carriage, its axis 0.2 m above the carriage's reference point.
m_armConfig.WithMechanismPositionConfig(
    MechanismPositionConfig{}
        .WithParent([this] { return m_elevator->GetPose3d(); })
        .WithRelativePosition(wpi::math::Translation3d{0_m, 0_m, 0.2_m}));

// A turret 0.3 m off the floor, turned 180 degrees so that its 0 degrees faces the back of the robot.
m_turretConfig.WithMechanismPositionConfig(
    MechanismPositionConfig{}
        .WithMovementPlane(MechanismPositionConfig::Plane::XY)
        .WithRelativePosition(wpi::math::Transform3d{wpi::math::Translation3d{0_m, 0_m, 0.3_m},
                                                     wpi::math::Rotation3d{0_rad, 0_rad, 180_deg}}));
```
{% endtab %}
{% endtabs %}

`withRelativePosition(Transform3d)` turns the mechanism where it is mounted, such as an arm on the side of the robot facing left. With `withParent(...)`, the relative position is from the parent's pose, such as the elevator's carriage, rather than from the robot.

**Check it:** the code builds.
{% endstep %}

{% step %}
### Publish the poses

Publish every component's pose in one array, in the same order as the components in the model (`model_0.glb`, `model_1.glb`, ...). Publish it every loop.

{% tabs %}
{% tab title="Java" %}
Commands v2: add a method to `RobotContainer`, where all three subsystems are, and call it from `Robot.robotPeriodic()`. Commands v3: put the call in `Robot.robotPeriodic()`.

{% code title="RobotContainer.java (fragment)" %}
```java
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.telemetry.Telemetry;

/** Publishes every component's pose. Call it every loop from Robot.robotPeriodic(). */
public void publishComponents() {
  Telemetry.log("Robot/Components",
                new Pose3d[] {elevator.getPose3d(), arm.getPose3d(), turret.getPose3d()},
                Pose3d.struct);
}
```
{% endcode %}
{% endtab %}

{% tab title="C++" %}
{% code title="cpp/RobotContainer.cpp (fragment)" %}
```cpp
#include <array>
#include <span>
#include <wpi/telemetry/Telemetry.hpp>

// In a method you call every loop from Robot::RobotPeriodic().
std::array<wpi::math::Pose3d, 3> components{m_elevator->GetPose3d(), m_arm->GetPose3d(),
                                            m_turret->GetPose3d()};
wpi::telemetry::Log("Robot/Components", std::span<const wpi::math::Pose3d>{components});
```
{% endcode %}
{% endtab %}
{% endtabs %}

With AdvantageKit, record the same array with `Logger.recordOutput("Robot/Components", poses)`.

**Check it:** start the simulation. `Robot/Components` shows in AdvantageScope's sidebar as an array of `Pose3d`, one for each component.
{% endstep %}

{% step %}
### Export the model

Export the robot from CAD without the moving parts as `model.glb`, and each moving part as its own file: `model_0.glb`, `model_1.glb`, and so on, in the order you publish their poses. Export every part where it is when the robot is at its starting position. [Converting Onshape & STEP Files to glTF](https://docs.advantagescope.org/more-features/custom-assets/gltf-convert) explains how.

Put them in a folder named `Robot_<name>`, such as `Robot_Dozer`, with a `config.json`, in AdvantageScope's assets folder (`App`/`AdvantageScope` > `Show Assets Folder`). `App`/`AdvantageScope` > `Use Custom Assets Folder` can load them from your robot project instead, so they are kept with your code.

```json
{
  "name": "Dozer",
  "rotations": [],
  "position": [0, 0, 0],
  "cameras": [],
  "components": [
    { "zeroedRotations": [], "zeroedPosition": [0, 0, 0] },
    { "zeroedRotations": [], "zeroedPosition": [0, 0, 0] },
    { "zeroedRotations": [], "zeroedPosition": [0, 0, 0] }
  ]
}
```

**Check it:** restart AdvantageScope. Your robot's name is in the 3D Field tab's robot list, and the robot model is drawn.
{% endstep %}

{% step %}
### Line each part up with its pose

AdvantageScope moves each component from where its `zeroedRotations` and `zeroedPosition` put it, by the pose you publish. Set them so that each part sits at the origin as YAMS's pose for it at zero expects:

| Component            | Zeroed at the origin                                                                       |
| -------------------- | ------------------------------------------------------------------------------------------ |
| An arm or pivot      | Its axis at the origin, pointing along +X at 0° (+Y in the YZ plane)                       |
| An elevator carriage | The carriage's reference point, the point you gave as its relative position, at the origin |
| A flywheel           | Its axle at the origin                                                                     |
| A part on another    | As for its own kind: its relative position is from its parent, not the robot               |

To set them:

1. Publish a zeroed robot pose every loop, such as `Telemetry.log("Robot/ZeroPose", Pose2d.ZERO, Pose2d.struct);`, select it as the robot's pose in AdvantageScope, and pick the "Axes" field, which shows the origin.
2. Adjust the model's `rotations`, then its `position`, until the robot is upright, facing +X, with its center on the origin and its wheels on the floor.
3. Publish an array of zeroed poses, one for each component, such as `Telemetry.log("Robot/ZeroComponents", new Pose3d[] {Pose3d.ZERO, Pose3d.ZERO, Pose3d.ZERO}, Pose3d.struct);`, and select it as the component poses.
4. For each component, adjust its `zeroedRotations`, then its `zeroedPosition`, until it sits at the origin as the table above says.
5. Publish the real poses, as in [Publish the poses](./#publish-the-poses). Each part now moves with its mechanism.

**Check it:** with the zeroed poses, every part sits where the table above says.
{% endstep %}
{% endstepper %}

## Check it

Start the simulation, open AdvantageScope's 3D Field tab with your robot model, and move each mechanism. Each part should move with its mechanism and stay attached to the robot, and the arm should rise with the elevator's carriage.

## If it did not work

* **A part is drawn 90° off.** An arm's pose is turned by its angle from 0° pointing forward, so an arm that reads 0° straight down is drawn 90° off. Set the arm's angle so 0° is level, as YAMS's [arm feedforward expects](../../understanding/arms.md), rather than turning the model to make up for it.
* **The robot or the parts float or sink.** AdvantageScope's robot origin is on the floor. Measure each mechanism's relative position from the floor, not the bellypan, and line the robot model up with the floor as in [Line each part up with its pose](./#line-each-part-up-with-its-pose).
* **A part moves with another mechanism's pose.** The poses are in a different order from the components: AdvantageScope gives the first pose to `model_0.glb`, the second to `model_1.glb`, and so on.
* **A turret swings up instead of turning.** A new `MechanismPositionConfig` is in the XZ plane, so set `withMovementPlane(Plane.XY)` on the one you give a turret.
* **A part on another mechanism does not move with it.** It needs `withParent(...)`, such as an arm on an elevator's carriage. Without it, its relative position is from the robot, and it stays put as the carriage rises.
* **Every part is off by the same amount all the time.** It was exported away from its starting position. Export every part where it is at the robot's starting position.

## Read more

* Elevator stages: [How do I show an elevator's stages on a 3D robot model?](how-do-i-show-an-elevators-stages-on-a-3d-robot-model.md)
* Record a match and replay it: [How do I view my DataLog in AdvantageScope?](../how-do-i-view-my-datalog-in-advantagescope.md)
