---
icon: chart-line
---

# Input stream telemetry and live tuning

A [`SwerveInputStream`](./) can publish what it is doing, and let you change its deadband, scales and limits from a dashboard while you drive.

Telemetry is off until you call `withTelemetry(name, verbosity)` on a stream. It publishes every time the stream is read with `get()`, so there is nothing to call each loop. `getTelemetry()` returns the stream's `SwerveInputStreamTelemetry` as an `Optional`.

| Verbosity | Publishes                                                                                                                                       |
| --------- | ----------------------------------------------------------------------------------------------------------------------------------------------- |
| `LOW`     | The current drive mode                                                                                                                          |
| `MID`     | Also the stream's configuration, read-only, and the [rotation controller's inputs](swerve-input-stream-telemetry.md#rotation-controller-inputs) |
| `HIGH`    | Also editable copies of the configuration, and a `Live Tuning` command on the dashboard                                                         |

Call `withTelemetry` last, after the rest of the configuration, so the dashboard starts with the configured values. Calling it again replaces the stream's telemetry, and `getTelemetry().get().close()` stops publishing it, e.g. when replacing a stream with another of the same name.

## Rotation controller inputs

`HEADING`, `AIM` and `TRANSLATION_ONLY` turn the robot with the drive's rotation controller. At `MID` and above, the stream publishes what it gives that controller on every read, under `SwerveInputStream/<name>/rotationController`:

| Key           | Value                                                                                 |
| ------------- | ------------------------------------------------------------------------------------- |
| `measurement` | The robot's heading, in radians in the gyro's frame                                   |
| `setpoint`    | The heading being turned to, in radians in the gyro's frame: the aim heading in `AIM` |

Both are `NaN` on reads in `ANGULAR_VELOCITY`, which doesn't use the controller, so a graph shows a gap rather than a stale value.

Graph the two together while you hold the aim or heading button. The gap between them is the controller's error: a measurement that is slow to reach the setpoint needs more `kP`, and one that overshoots and wobbles needs less `kP` or some `kD`. Tune the gains in [`SwerveDriveConfig`](../mechanism-configs/swerve-drive-config.md#auto-align-controllers).

`getRotationControllerMeasurement()` and `getRotationControllerSetpoint()` return the same values as `Optional<Angle>`s, empty when the last `get()` didn't use the controller. C++ has `GetRotationControllerMeasurement()` and `GetRotationControllerSetpoint()`, returning `std::optional<wpi::units::radian_t>`.

{% tabs %}
{% tab title="Commands v2" %}
In `SwerveSubsystem.getChassisSpeedsSupplier(...)` from the [tutorial](../../tutorials/swerve-drive/driving.md), `withTelemetry` goes last:

```java
return SwerveInputStream.of(drive, translationX, translationY)
    .withControllerRotationAxis(rotation)
    .withDeadband(0.05)
    .withScaleTranslation(0.8)
    .withTelemetry("Driver", TelemetryVerbosity.HIGH);
```

The `Live Tuning` command is a Commands v2 `Command`.
{% endtab %}

{% tab title="Commands v3" %}
In `SwerveMechanism.getChassisSpeedsSupplier(...)` from the [tutorial](../../tutorials/swerve-drive/driving.md), `withTelemetry` goes last:

```java
return SwerveInputStream.of(drive, translationX, translationY)
    .withControllerRotationAxis(rotation)
    .withDeadband(0.05)
    .withScaleTranslation(0.8)
    .withTelemetry("Driver", TelemetryVerbosity.HIGH);
```

The `Live Tuning` command is a Commands v3 `Command`, published through `CommandTunable`. If an op mode builds a new stream with the same name, close the old stream's telemetry first, with `oldStream.getTelemetry().ifPresent(SwerveInputStreamTelemetry::close)`.
{% endtab %}

{% tab title="C++" %}
In `SwerveSubsystem::DriveWithSticks(...)` from the [tutorial](../../tutorials/swerve-drive/driving.md), `WithTelemetry` goes last:

```cpp
m_inputStream.emplace(
    utility::SwerveInputStream<4>::Of(m_drive.value(), translationX, translationY)
        .WithControllerRotationAxis(rotation)
        .WithDeadband(0.05)
        .WithScaleTranslation(0.8)
        .WithTelemetry("Driver", Cfg::TelemetryVerbosity::HIGH));
```

C++ has the same API: `WithTelemetry(name, verbosity)` and `GetTelemetry()`, which returns a `std::optional<std::reference_wrapper<SwerveInputStreamTelemetry<N>>>`. The verbosities are `SwerveDriveConfig::TelemetryVerbosity`: `LOW`, `MEDIUM` (Java's `MID`) and `HIGH`, and `NONE` turns telemetry off. The `Live Tuning` command is a Commands v2 `Command`, from `GetLiveTuningCommand()`.

The stream is a value type, so ownership works differently from Java:

* `WithTelemetry` stores the name and verbosity. The telemetry is created when the stream is first read with `Get()` or `GetTelemetry()` is called, so the copies made while building a stream do not publish.
* The stream owns its telemetry and stops publishing when it is destroyed.
* A copy of a stream keeps the telemetry settings and publishes under the same name once it is read. `Clone()` returns a copy without telemetry.
* Moving a stream moves the settings; the moved-from stream stops publishing.
{% endtab %}
{% endtabs %}

Neither framework's `Live Tuning` command requires a subsystem or mechanism, so running it does not interrupt the command driving with the stream.

## Live tuning

At `HIGH`, values edited on the dashboard are applied to the stream every loop while the `Live Tuning` command runs. Start and stop it from the dashboard, or bind `getTelemetry().get().getLiveTuningCommand()` to a button. Tunable values stay in sync both ways while it runs: a change made in code, such as a slow mode binding calling `withScaleTranslation`, or a `BooleanSupplier` passed to `withAllianceRelativeControl` changing, is published to the dashboard and is not overridden. Dashboard values outside the allowed range are replaced with the stream's current value.

## NetworkTables topics

`mode` (the current drive mode: `ANGULAR_VELOCITY`, `HEADING`, `AIM` or `TRANSLATION_ONLY`) is published to `/SwerveInputStream/<name>/` at every verbosity. At `MID` (`MEDIUM` in C++) and `HIGH`, the values below are also published there read-only. At `HIGH` they are published as tunable values under `/Tuning/SwerveInputStream/<name>/`, with the command at `/Tuning/SwerveInputStream/<name>/Live Tuning`.

| Topic                | Type    | Range   | Description                                                                                     |
| -------------------- | ------- | ------- | ----------------------------------------------------------------------------------------------- |
| `deadband`           | double  | \[0, 1) | Controller axis deadband                                                                        |
| `translationScale`   | double  | (0, 1]  | Translation axis scaling factor                                                                 |
| `rotationScale`      | double  | (0, 1]  | Rotation axis scaling factor                                                                    |
| `maxLinearVelocity`  | double  | > 0     | Maximum chassis linear velocity (m/s). Starts at the drive config's maximum and overrides it    |
| `maxAngularVelocity` | double  | > 0     | Maximum chassis angular velocity (rad/s). Starts at the drive config's maximum and overrides it |
| `translationCube`    | boolean | -       | Enable cubic translation response curve                                                         |
| `rotationCube`       | boolean | -       | Enable cubic rotation response curve                                                            |
| `allianceRelative`   | boolean | -       | Enable alliance-relative translation flip                                                       |
| `robotRelative`      | boolean | -       | Enable robot-relative control                                                                   |

The output velocities are not published: reading them would call `get()` a second time each loop, which runs the heading controller twice. Log the speeds you send to the drive instead.

{% hint style="success" %}
Once the values feel right, copy them from NetworkTables back into the stream's `with*()` calls in code. Values edited live are not persisted. For competition, use `TelemetryVerbosity.LOW`; in Java `getTelemetry().get().close()` stops publishing, and in C++ `WithTelemetry(name, TelemetryVerbosity::NONE)` does.
{% endhint %}

| Issue                        | Solution                                                                       |
| ---------------------------- | ------------------------------------------------------------------------------ |
| Telemetry not appearing      | Check the stream was given `withTelemetry` and is read with `get()` every loop |
| Changes don't apply          | Start the `Live Tuning` command; it only exists at `TelemetryVerbosity.HIGH`   |
| A dashboard value snaps back | The value is outside the allowed range in the table above                      |

## Code reference

{% @github-files/github-code-block url="https://github.com/Yet-Another-Software-Suite/YAMS/blob/master/examples/commands2/simple_robot-2027/src/main/java/first/robot/subsystems/SwerveSubsystem.java" %}
