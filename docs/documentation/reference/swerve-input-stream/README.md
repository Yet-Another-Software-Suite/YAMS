---
icon: gamepad
---

# Swerve Input Stream

`SwerveInputStream` bridges raw joystick axes (each in the range `[-1, 1]`) and the field-relative `ChassisVelocities` (WPILib 2027's name for `ChassisSpeeds`) that [`SwerveDrive.setFieldRelativeChassisSpeeds(...)`](../../understanding/swerve-drive/) expects. It implements `Supplier<ChassisVelocities>`, so once built it is read with `get()` every loop by the command that drives the robot.

Import it from the package that matches your vendordep: `yams.commands2.swerve.SwerveInputStream` (Commands v2) or `yams.commands3.swerve.SwerveInputStream` (Commands v3). The API is the same in both. In C++ it is `yams::mechanisms::swerve::utility::SwerveInputStream<NumModules>` from `yams/mechanisms/swerve/utility/SwerveInputStream.hpp`, with the same methods in PascalCase (`Of`, `WithDeadband`, `Get`, ...). In the examples on this page, `driverController` is the `CommandGamepad` from the [Swerve Drive tutorial](../../tutorials/swerve-drive/). Its buttons, such as `driverController.rightBumper()`, are `Trigger`s, which can be passed wherever a `BooleanSupplier` is expected.

<figure><img src="../../.gitbook/assets/swerve-input-stream-sticks.gif" alt="Animation of two controller sticks, the ChassisVelocities they produce, and a robot on a field. Left stick forward gives positive vx and the robot moves away from the driver station. Left stick left gives positive vy and the robot moves left. A diagonal push gives both. Right stick left gives a positive omega and the robot turns counter-clockwise"><figcaption><p>How each stick movement becomes a field relative velocity.</p></figcaption></figure>

{% hint style="warning" %}
The output is **field relative**, even with `withRobotRelative()`. Drive with it through `setFieldRelativeChassisSpeeds(...)`, not `SwerveDrive.drive(...)`, which expects robot-relative speeds. If you do want `drive(...)`, convert it first: `drive.drive(() -> input.get().toRobotRelative(drive.getRotation()))`. `getRotation()` is the odometry heading that `setFieldRelativeChassisSpeeds` uses too.
{% endhint %}

This page covers building a stream and shaping the sticks. Two more pages cover the rest:

* [Heading and aim modes](swerve-input-stream-aim.md): facing a heading, aiming at a target, and driving without turning.
* [Input stream telemetry and live tuning](swerve-input-stream-telemetry.md): what a stream publishes, and changing it from a dashboard.

## Building a stream

`SwerveInputStream.of(drive, x, y)` is the base factory, translation axes only. `SwerveInputStream.of(drive, x, y, rot)` (or the matching constructor) adds an angular velocity axis, and the `new SwerveInputStream(drive, x, y, headingX, headingY)` constructor adds heading axes for point-and-hold control instead. `drive` is your YAMS `SwerveDrive`. From there, `with*()` methods layer on deadbanding, scaling and a non-linear response.

These examples follow the [Swerve Drive tutorial](../../tutorials/swerve-drive/): the subsystem or mechanism builds the stream, because only it can see the YAMS `SwerveDrive`, and a `driveFieldRelative` command drives with it.

{% tabs %}
{% tab title="Commands v2" %}
{% code title="subsystems/SwerveSubsystem.java" %}
```java
/** Turn three controller axes into field relative chassis speeds. */
public SwerveInputStream getChassisSpeedsSupplier(DoubleSupplier translationX, DoubleSupplier translationY,
                                                  DoubleSupplier rotation) {
  return SwerveInputStream.of(drive, translationX, translationY)
      .withControllerRotationAxis(rotation)
      .withDeadband(0.05)
      .withScaleTranslation(0.8)
      .withScaleRotation(0.6)
      .withMaximumLinearVelocity(MetersPerSecond.of(4.5))
      .withMaximumAngularVelocity(DegreesPerSecond.of(360))
      .withCubeTranslationControllerAxis();
}

/** Drive with field relative chassis speeds. */
public Command driveFieldRelative(Supplier<ChassisVelocities> speedsSupplier) {
  return run(() -> drive.setFieldRelativeChassisSpeeds(speedsSupplier.get()))
      .withName("Field Oriented Drive");
}
```
{% endcode %}

{% code title="RobotContainer.java: in the constructor" %}
```java
// SwerveInputStream output is field relative.
drive.setDefaultCommand(drive.driveFieldRelative(
    drive.getChassisSpeedsSupplier(
        () -> -driverController.getLeftY(),
        () -> -driverController.getLeftX(),
        () -> -driverController.getRightX())));
```
{% endcode %}
{% endtab %}

{% tab title="Commands v3" %}
{% code title="mechanisms/SwerveMechanism.java" %}
```java
/** Turn three controller axes into field relative chassis speeds. */
public SwerveInputStream getChassisSpeedsSupplier(DoubleSupplier translationX, DoubleSupplier translationY,
                                                  DoubleSupplier rotation) {
  return SwerveInputStream.of(drive, translationX, translationY)
      .withControllerRotationAxis(rotation)
      .withDeadband(0.05)
      .withScaleTranslation(0.8)
      .withScaleRotation(0.6)
      .withMaximumLinearVelocity(MetersPerSecond.of(4.5))
      .withMaximumAngularVelocity(DegreesPerSecond.of(360))
      .withCubeTranslationControllerAxis();
}

/** Drive with field relative chassis speeds. */
public Command driveFieldRelative(Supplier<ChassisVelocities> speedsSupplier) {
  return run(coroutine -> {
    while (true) {
      drive.setFieldRelativeChassisSpeeds(speedsSupplier.get());
      coroutine.yield();
    }
  }).named("Field Oriented Drive");
}
```
{% endcode %}

{% code title="ExampleTeleop.java: in the constructor" %}
```java
// SwerveInputStream output is field relative.
robot.drive.setDefaultCommand(robot.drive.driveFieldRelative(
    robot.drive.getChassisSpeedsSupplier(
        () -> -robot.exampleController.getLeftY(),
        () -> -robot.exampleController.getLeftX(),
        () -> -robot.exampleController.getRightX())));
```
{% endcode %}
{% endtab %}

{% tab title="C++" %}
{% code title="cpp/subsystems/SwerveSubsystem.cpp" %}
```cpp
// m_inputStream is a std::optional<SwerveInputStream<4>> member of the subsystem,
// because the drive command reads it every loop.
wpi::cmd::CommandPtr SwerveSubsystem::DriveWithSticks(std::function<double()> translationX,
                                                      std::function<double()> translationY,
                                                      std::function<double()> rotation) {
  m_inputStream.emplace(
      utility::SwerveInputStream<4>::Of(m_drive.value(), translationX, translationY)
          .WithControllerRotationAxis(rotation)
          .WithDeadband(0.05)
          .WithScaleTranslation(0.8));

  // SwerveInputStream output is field relative.
  return Run([this] { m_drive->SetFieldRelativeChassisSpeeds(m_inputStream->Get()); })
      .WithName("Field Oriented Drive");
}
```
{% endcode %}

{% code title="cpp/RobotContainer.cpp: in the constructor" %}
```cpp
drive.SetDefaultCommand(drive.DriveWithSticks(
    [this] { return -driverController.GetLeftY(); },
    [this] { return -driverController.GetLeftX(); },
    [this] { return -driverController.GetRightX(); }));
```
{% endcode %}

C++ has `Of(drive, x, y)` plus constructors taking a rotation axis or two heading axes.
{% endtab %}
{% endtabs %}

The single-line examples on the rest of this page call a method on `input`, a stream like the one `getChassisSpeedsSupplier` returns. Keep a reference to the stream if you want to change it later.

## Mutations apply immediately

<figure><img src="../../.gitbook/assets/sis-mutations.gif" alt="Animation of a robot aiming at a speaker target. When the left bumper is pressed the stream&#x27;s aim target is changed to the amp, and the robot turns to face the amp without the drive command being touched"><figcaption><p>Changing the aim target from a button. The drive command keeps running.</p></figcaption></figure>

Every `with*()` method mutates the stream in place and returns `this`; none of them return a new copy. That matters because a `SwerveInputStream` is usually stored once and read by a long-lived drive command. Calling a mutator on that same reference later, from anywhere else in your code, changes what the already-running default command does on its very next loop; you never need to rebuild the stream or call `setDefaultCommand(...)` again.

For example, auto-aim can be retargeted on the fly from a completely separate button binding, without touching the drive command at all:

A Commands v2 fragment. `input` is the stream the default command drives with:

```java
// Two targets on the field. Use your game's real positions.
Pose2d speakerPose = new Pose2d(Meters.of(0.2), Meters.of(5.5), Rotation2d.ZERO);
Pose2d ampPose = new Pose2d(Meters.of(1.8), Meters.of(8.0), Rotation2d.ZERO);

// Aim at the speaker while the right bumper is held.
input.withAim(() -> speakerPose, driverController.rightBumper());

// Pressing the left bumper retargets the same stream at the amp.
driverController.leftBumper().onTrue(Commands.runOnce(() ->
    input.withAim(() -> ampPose, driverController.rightBumper())));
```

Pressing the left bumper swaps the aim target from the speaker to the amp. The default command keeps reading the exact same `input` instance; it just starts computing `ChassisVelocities` toward a different pose the moment the mutation happens.

{% hint style="info" %}
`clone()` is the one method here that doesn't mutate: it returns an independent copy, useful for deriving a second variant of a stream from a shared base configuration without one affecting the other. See [Copying a stream](./#copying-a-stream) below.
{% endhint %}

## From stick to velocity

Every loop, `get()` runs the stick values through the same steps, in this order:

<figure><img src="../../.gitbook/assets/swerve-input-stream-pipeline.png" alt="A raw stick value of 0.60 goes through a 0.05 deadband to become 0.579, is scaled by 0.8 to 0.463, is cubed to 0.099, and is multiplied by a maximum velocity of 4.5 meters per second to give 0.45 meters per second"><figcaption><p>A 60% push, step by step.</p></figcaption></figure>

| Step             | Method                                                                    | What it does                                                                    |
| ---------------- | ------------------------------------------------------------------------- | ------------------------------------------------------------------------------- |
| Deadband         | `withDeadband(...)`                                                       | Ignores small values near the center, then rescales the rest to the full range. |
| Scale            | `withScaleTranslation(...)`, `withScaleRotation(...)`                     | Multiplies by a constant.                                                       |
| Cube             | `withCubeTranslationControllerAxis()`, `withCubeRotationControllerAxis()` | Raises to the third power for finer control near the center.                    |
| Alliance flip    | `withAllianceRelativeControl()`                                           | Turns translation 180 degrees on the red alliance.                              |
| Maximum velocity | `withMaximumLinearVelocity(...)`, `withMaximumAngularVelocity(...)`       | Turns the result into meters per second and radians per second.                 |

{% hint style="warning" %}
The scale is applied **before** the cube. With `withScaleTranslation(0.8)` and the cube enabled, a full stick gives `0.8³ = 0.512` of the maximum velocity, not `0.8`.
{% endhint %}

<figure><img src="../../.gitbook/assets/swerve-input-stream-response.png" alt="A graph of output against stick position. No shaping is a straight line. A 0.05 deadband gives nothing until 5 percent and then rises to full. Deadband plus cube stays low near the center and rises steeply near the edge. Deadband plus a 0.5 scale tops out at half speed"><figcaption><p>What each option does to the stick's response.</p></figcaption></figure>

## Velocity limits

The maximum velocities are what a fully pushed stick produces.

### `withMaximumLinearVelocity`

`withMaximumLinearVelocity(LinearVelocity)` sets the top driving speed.

<figure><img src="../../.gitbook/assets/sis-velocity-limits.gif" alt="Animation of two robots with the stick fully pushed. With a maximum of 2.0 meters per second the robot moves slowly, with 4.5 meters per second it moves more than twice as fast"><figcaption><p>The same full stick with two maximum velocities.</p></figcaption></figure>

```java
input.withMaximumLinearVelocity(MetersPerSecond.of(4.5));
```

### `withMaximumAngularVelocity`

`withMaximumAngularVelocity(AngularVelocity)` sets the top turn rate.

<figure><img src="../../.gitbook/assets/sis-max-angular-velocity.gif" alt="Animation of two robots with the right stick fully pushed. With a maximum of 180 degrees per second the robot turns slowly, with 360 degrees per second it turns twice as fast"><figcaption><p>The same full stick with two maximum angular velocities.</p></figcaption></figure>

```java
input.withMaximumAngularVelocity(DegreesPerSecond.of(360));
```

{% hint style="info" %}
A stream copies `SwerveDriveConfig.withMaximumChassisSpeed(...)`'s value (if set) into its own maximum fields once, at construction time; if the drive has no maximum configured, the stream starts from its own defaults (4 m/s, 1 rotation/s) instead. Calling `withMaximumLinearVelocity(...)`/`withMaximumAngularVelocity(...)` afterward overrides that starting value for this stream only and takes effect immediately; the drive's config is never re-checked after construction, so the override sticks.
{% endhint %}

## Controller axes

Both methods set or replace a rotation source after construction. That is useful when a stream starts out with `SwerveInputStream.of(drive, x, y)` (no rotation source at all) and gains one later.

### `withControllerRotationAxis`

`withControllerRotationAxis(DoubleSupplier)` sets the axis that says how fast the robot turns.

<figure><img src="../../.gitbook/assets/sis-rotation-axis.gif" alt="Animation of the right stick pushed left and then right. Pushed left the robot turns counter-clockwise, pushed right it turns clockwise"><figcaption><p>One axis sets a turn rate.</p></figcaption></figure>

```java
input.withControllerRotationAxis(() -> -driverController.getRightX());
```

### `withControllerHeadingAxis`

`withControllerHeadingAxis(DoubleSupplier, DoubleSupplier)` sets the pair of axes whose direction becomes a target heading. It does nothing on its own: the heading is used by [`withHeadingControl`](swerve-input-stream-aim.md#withheadingcontrol) in `HEADING` mode.

<figure><img src="../../.gitbook/assets/sis-heading-axis.gif" alt="Animation of the right stick moving all the way around its edge while a robot on the field turns to face the same direction. Stick up is 0 degrees, stick left is 90 degrees, stick down is 180 degrees and stick right is minus 90 degrees"><figcaption><p>Two axes set a direction, and the robot turns to face it.</p></figcaption></figure>

The heading axes only replace rotation. The left stick keeps driving the robot, so it can move one way while facing another:

<figure><img src="../../.gitbook/assets/sis-heading-axis-driving.gif" alt="Animation of a robot driving across the field along a curved path set by the left stick. While it drives, the right stick swings from left to right and the robot turns to face where the right stick points, without changing the path it drives"><figcaption><p>Driving with the left stick while the right stick sets the heading.</p></figcaption></figure>

```java
input.withControllerHeadingAxis(() -> -driverController.getRightX(), () -> -driverController.getRightY());
```

## Deadband

`withDeadband(double)` (an alias for `deadband(double)`) applies the same deadband to every axis before any scaling or cubing happens. `0` clears it.

<figure><img src="../../.gitbook/assets/sis-deadband.gif" alt="Animation of a stick at rest reading small non-zero values. Without a deadband the robot creeps across the field. With a 0.1 deadband it stays still"><figcaption><p>Stick drift with and without a deadband.</p></figcaption></figure>

```java
input.withDeadband(0.05);
```

## Axis scaling

The scale methods multiply the stick output by a constant in `(0, 1]`, useful for a "slow mode" that doesn't require re-mapping the physical stick range.

### `withScaleTranslation`

`withScaleTranslation(double)` scales the driving speed.

<figure><img src="../../.gitbook/assets/sis-scaling.gif" alt="Animation of two robots with the left stick fully pushed. With no scale the robot moves at full speed, with a scale of 0.5 it moves at half speed"><figcaption><p>A translation scale of 0.5.</p></figcaption></figure>

```java
input.withScaleTranslation(0.5); // e.g. bound to a "slow mode" trigger
```

### `withScaleRotation`

`withScaleRotation(double)` scales the turn rate.

<figure><img src="../../.gitbook/assets/sis-scale-rotation.gif" alt="Animation of two robots with the right stick fully pushed. With no scale the robot turns at the full rate, with a scale of 0.5 it turns at half the rate"><figcaption><p>A rotation scale of 0.5.</p></figcaption></figure>

```java
input.withScaleRotation(0.5);
```

## Non-linear (cubic) response

Cubing gives finer control near the center of the stick without sacrificing full speed at the edges. Each method also has a `BooleanSupplier` overload to toggle the effect live.

### `withCubeTranslationControllerAxis`

`withCubeTranslationControllerAxis()` cubes the translation magnitude.

<figure><img src="../../.gitbook/assets/sis-cube.gif" alt="Animation of a stick sweeping from center to full and back, with two bars and a graph. The linear output follows the stick. The cubed output stays very small until the stick is pushed far, then rises to the same maximum"><figcaption><p>Linear and cubed response to the same stick.</p></figcaption></figure>

```java
input.withCubeTranslationControllerAxis();
```

### `withCubeRotationControllerAxis`

`withCubeRotationControllerAxis()` cubes the rotation axis.

<figure><img src="../../.gitbook/assets/sis-cube-rotation.gif" alt="Animation of the right stick sweeping from center to full and back, with two bars and a graph. The linear turn rate follows the stick. The cubed turn rate stays very small until the stick is pushed far, then rises to the same maximum"><figcaption><p>Linear and cubed turn rate from the same stick.</p></figcaption></figure>

```java
input.withCubeRotationControllerAxis(driverController.faceRight()); // only while the right face button is held
```

## Robot-relative control

By default the translation axes are field relative. `withRobotRelative()` treats them as robot relative instead, so pushing the stick forward drives toward the robot's front. The stream still outputs **field-relative** `ChassisVelocities` (it converts using the current heading), so the drive command does not change. The `withRobotRelative(BooleanSupplier)` overload toggles it live, e.g. while a button is held.

<figure><img src="../../.gitbook/assets/swerve-input-stream-frames.png" alt="Three drawings of a robot facing up and to the left with the left stick pushed forward. Field relative: it moves away from your driver station. Robot relative: it moves toward its own front. On the red alliance with alliance relative control: translation is flipped 180 degrees and it still moves away from you"><figcaption><p>Field relative, robot relative and alliance relative control.</p></figcaption></figure>

<figure><img src="../../.gitbook/assets/sis-robot-relative.gif" alt="Animation of two robots facing up and to the left with the stick pushed forward. Field relative, the robot moves straight away from the driver station. Robot relative, it moves toward its own front"><figcaption><p>Stick forward, field relative and robot relative.</p></figcaption></figure>

```java
input.withRobotRelative(driverController.faceLeft()); // robot-relative while the left face button is held
```

## Alliance-relative control

`withAllianceRelativeControl()` flips the field-relative translation 180 degrees on the Red alliance, so "forward" always means away from your own driver station regardless of which side you're on. `withAllianceRelativeControl(BooleanSupplier)` toggles it live.

<figure><img src="../../.gitbook/assets/sis-alliance-relative.gif" alt="Animation of a blue alliance robot and a red alliance robot, both with the stick pushed forward. Each one drives away from its own driver station, which are at opposite ends of the field"><figcaption><p>Stick forward on each alliance.</p></figcaption></figure>

```java
input.withAllianceRelativeControl();
```

{% hint style="warning" %}
Alliance-relative control assumes the robot's pose is a real field pose, where a heading of 0 degrees faces the red alliance wall. On red, a robot facing away from its own driver station has a heading of 180 degrees. If the robot starts at a pose of 0 degrees on red (the default starting pose, or after `zeroGyro()`) the flip sends it toward its own driver station. Set the real pose first, from your autonomous routine, vision, or `resetOdometry(...)`.
{% endhint %}

{% hint style="info" %}
`withAllianceRelativeControl()` and `withRobotRelative()` can both be set. While robot-relative control is active, alliance flipping is skipped.
{% endhint %}

## Translation heading offset

`withTranslationHeadingOffset(Rotation2d)` rotates the translation direction by a fixed angle before it's output, useful when a secondary driver's "forward" should map to something other than the robot's own forward. `withTranslationHeadingOffset(Rotation2d, BooleanSupplier)` only applies it while the supplier is true.

<figure><img src="../../.gitbook/assets/sis-heading-offset.gif" alt="Animation of two robots with the stick pushed forward. With no offset the robot moves forward. With a 90 degree offset it moves to the left"><figcaption><p>A 90° offset turns forward into left.</p></figcaption></figure>

```java
input.withTranslationHeadingOffset(Rotation2d.fromDegrees(90));
```

## Center of rotation

The stream outputs a rotation rate, and the drive decides which point the robot rotates around. That point is not part of the stream: it is `SwerveDriveConfig.withCenterOfRotation(...)`. The default is the center of the robot.

<figure><img src="../../.gitbook/assets/swerve-center-of-rotation.gif" alt="Animation of two robots spinning with the same right stick input. The first spins around its own center. The second spins around its front left corner, so the rest of the robot swings around that corner"><figcaption><p>The same turn around two different points.</p></figcaption></figure>

```java
// 0, 0 is the center of the robot. X is forward and Y is left.
SwerveDriveConfig config = new SwerveDriveConfig(this, fl, fr, bl, br)
    .withCenterOfRotation(Meters.of(0.3), Meters.of(0.3)); // the front left corner
```

Like the stream, the config is mutable, and the drive reads the center of rotation every loop. Changing it on the running drive's config takes effect on the next loop, for example to pivot around a corner while a button is held:

```java
driverController.leftBumper()
    .onTrue(Commands.runOnce(() -> drive.getConfig().withCenterOfRotation(Meters.of(0.3), Meters.of(0.3))))
    .onFalse(Commands.runOnce(() -> drive.getConfig().withCenterOfRotation(Meters.of(0), Meters.of(0))));
```

## Copying a stream

`clone()` is the only method on this page that doesn't mutate the original; it returns an independent copy that shares the base stream's configuration at the moment it was called. Changing the copy afterward, or changing the original, doesn't affect the other. A clone has no [telemetry](swerve-input-stream-telemetry.md); call `withTelemetry` on it if it needs its own.

```java
SwerveInputStream headingStream = input.clone();
```

## Turning to a heading, aiming, and telemetry

* [Heading and aim modes](swerve-input-stream-aim.md): `withHeadingControl`, `withHeading`, `withAim` and its options, and `withTranslationOnly`.
* [Input stream telemetry and live tuning](swerve-input-stream-telemetry.md): `withTelemetry`, the rotation controller's inputs, and the `Live Tuning` command.

## Code reference

{% @github-files/github-code-block url="https://github.com/Yet-Another-Software-Suite/YAMS/blob/master/examples/commands2/simple_robot-2027/src/main/java/first/robot/subsystems/SwerveSubsystem.java" %}
