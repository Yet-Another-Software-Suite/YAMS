---
description: >-
  Walkthrough of the collision and jerk detection example robot in Commands v2,
  Commands v3, and C++.
icon: code
---

# The collision and jerk detection example

This example pairs a standard YAMS `SwerveDrive` and `Pigeon2` IMU with a standalone `CollisionJerkMonitor`. No core YAMS classes are modified.

## Where the code lives

* **Commands v2:** [`examples/commands2/collision_jerk_detection/`](https://github.com/Yet-Another-Software-Suite/YAMS/tree/master/examples/commands2/collision_jerk_detection)
* **Commands v3:** [`examples/commands3/collision_jerk_detection/`](https://github.com/Yet-Another-Software-Suite/YAMS/tree/master/examples/commands3/collision_jerk_detection)

## What each file does

| Concept | Explained in | Where in the example |
| --- | --- | --- |
| 3D gyro-rotated gravity subtraction | [Why `| \|\|a\|\| - 1 g |` fails](./#why--a---1-g--fails) | `CollisionJerkMonitor.java` (`removeGravity(...)`) |
| 3D linear jerk & NaN-safe bump latch | [How the monitor works](./#how-the-monitor-works) | `CollisionJerkMonitor.java` (`update(...)`) |
| Feeding `Pigeon2` attitude & accel into the monitor | [How the monitor works](./#how-the-monitor-works) | `SwerveSubsystem.java` (v2) / `SwerveMechanism.java` (v3) |
| Unit + seeded fuzz tests (0.9 g side hit, 1.2 g drive, NaN latch) | [Why `| \|\|a\|\| - 1 g |` fails](./#why--a---1-g--fails) | `CollisionJerkMonitorTest.java` & `CollisionJerkDetectionTest.java` |

## Subtracting 3D gravity and detecting jerk spikes

{% tabs %}
{% tab title="Java, Commands v2" %}
{% code title="CollisionJerkMonitor.java (fragment)" %}
```java
public static Translation3d removeGravity(
    Rotation3d attitude,
    LinearAcceleration accelX,
    LinearAcceleration accelY,
    LinearAcceleration accelZ) {
  Translation3d gravityInBodyGs =
      new Translation3d(0.0, 0.0, ONE_G).rotateBy(new Rotation3d(attitude.getQuaternion().inverse()));
  return new Translation3d(
      accelX.in(Gs) - gravityInBodyGs.getX(),
      accelY.in(Gs) - gravityInBodyGs.getY(),
      accelZ.in(Gs) - gravityInBodyGs.getZ());
}

public boolean update(
    Time timestamp,
    Rotation3d attitude,
    LinearAcceleration accelX,
    LinearAcceleration accelY,
    LinearAcceleration accelZ) {
  double nowSeconds = timestamp.in(Seconds);
  double ax = accelX.in(Gs);
  double ay = accelY.in(Gs);
  double az = accelZ.in(Gs);

  // Preserve bumpedRecently latch when a non-finite IMU sample arrives.
  if (!Double.isFinite(nowSeconds)
      || !Double.isFinite(ax)
      || !Double.isFinite(ay)
      || !Double.isFinite(az)) {
    return bumpedRecently;
  }

  Translation3d linearAccelGs = removeGravity(attitude, accelX, accelY, accelZ);
  // Compute 3D linear jerk (g/s) and update latch...
  return bumpedRecently;
}
```
{% endcode %}

{% code title="SwerveSubsystem.java (fragment)" %}
```java
@Override
public void periodic() {
  drive.updateTelemetry();
  collisionMonitor.update(
      Seconds.of(Timer.getTimestamp()),
      drive.getGyroRotation3d(),
      Gs.of(gyro.getAccelerationX().getValueAsDouble()),
      Gs.of(gyro.getAccelerationY().getValueAsDouble()),
      Gs.of(gyro.getAccelerationZ().getValueAsDouble()));

  jerkPub.set(collisionMonitor.getLastJerkGsPerSecond());
  linearAccelPub.set(collisionMonitor.getLastLinearAccelGs());
  bumpedPub.set(collisionMonitor.isBumpedRecently());
  collisionCountPub.set(collisionMonitor.getCollisionCount());
}
```
{% endcode %}
{% endtab %}

{% tab title="Java, Commands v3" %}
{% code title="SwerveMechanism.java (fragment)" %}
```java
public void periodic() {
  drive.updateTelemetry();
  collisionMonitor.update(
      Seconds.of(Timer.getTimestamp()),
      drive.getGyroRotation3d(),
      Gs.of(gyro.getAccelerationX().getValueAsDouble()),
      Gs.of(gyro.getAccelerationY().getValueAsDouble()),
      Gs.of(gyro.getAccelerationZ().getValueAsDouble()));

  jerkPub.set(collisionMonitor.getLastJerkGsPerSecond());
  linearAccelPub.set(collisionMonitor.getLastLinearAccelGs());
  bumpedPub.set(collisionMonitor.isBumpedRecently());
  collisionCountPub.set(collisionMonitor.getCollisionCount());
}
```
{% endcode %}
{% endtab %}

{% tab title="C++" %}
{% code title="CollisionJerkMonitor.hpp (fragment)" %}
```cpp
#include <wpi/math/geometry/Rotation3d.hpp>
#include <wpi/math/geometry/Translation3d.hpp>
#include <wpi/units/acceleration.hpp>

wpi::math::Translation3d RemoveGravityGs(
    const wpi::math::Rotation3d& attitude,
    wpi::units::standard_gravity_t ax,
    wpi::units::standard_gravity_t ay,
    wpi::units::standard_gravity_t az) {
  wpi::math::Translation3d gBody =
      wpi::math::Translation3d{0_m, 0_m, 1_m}.RotateBy(-attitude);
  return wpi::math::Translation3d{
      wpi::units::meter_t{ax.value() - gBody.X().value()},
      wpi::units::meter_t{ay.value() - gBody.Y().value()},
      wpi::units::meter_t{az.value() - gBody.Z().value()}};
}
```
{% endcode %}
{% endtab %}
{% endtabs %}

## Tuning for your robot

1. **Verify IMU sign convention at rest:** with the robot sitting flat on carpet, `Collision/LinearAccelGs` should read near `0.0 g`. If it reads `~2.0 g`, your IMU reports gravity as `-1 g` along Z—negate the Z axis before passing it into `update(...)`.
2. **Record hard driving first:** drive full-speed figure-eights and hard stops while logging `Collision/JerkGsPerSecond` in AdvantageScope. Set `JERK_THRESHOLD_GS_PER_SECOND` comfortably above your peak clean-driving jerk (typically `20–30 g/s` on a 50 Hz loop).
3. **Set `BUMP_LATCH_DURATION`:** `0.50 s` is long enough for drivers to feel controller rumble or for a vision/scoring gate to wait for chassis bounce to settle.

## If it does not work

* **Linear acceleration reads ~2 g on flat ground:** your IMU's Z-axis points downward (NED frame) instead of upward (NWU). Configure the gyro's mount pose (see [How do I use a gyro mounted on its side?](../../how-to-motor-controllers-and-sensors/how-do-i-use-a-gyro-mounted-on-its-side.md)) or negate the raw Z reading.
* **False triggers when crossing bumps at speed:** bump impacts create real vertical (`Z`) jerk. If you only want horizontal wall/robot hits, compute jerk from the XY components of `removeGravity(...)` instead of all three axes.
