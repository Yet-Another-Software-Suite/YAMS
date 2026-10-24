# SwerveInputStream telemetry and live tuning

This document describes the telemetry and live tuning features for `SwerveInputStream`.

## Overview

Telemetry is off until you call `withTelemetry(name, verbosity)` on a stream. It creates the stream's
`SwerveInputStreamTelemetry`, available from `getTelemetry()` as an `Optional`, and publishes every time the
stream is read with `get()`, so there is nothing to call each loop.

| Verbosity | Publishes |
|-----------|-----------|
| `LOW` | The current drive mode |
| `MID` | Also the stream's configuration, read-only |
| `HIGH` | Also editable copies of the configuration, and a `Live Tuning` command on the dashboard |

At `HIGH`, values edited on the dashboard are applied to the stream every loop while the `Live Tuning`
command runs; start and stop it from the dashboard, or bind `getTelemetry().get().getLiveTuningCommand()` to a
button. Tunable values stay in sync both ways while it runs. A change made in code, such as a slow mode binding
calling `withScaleTranslation`, or a `BooleanSupplier` passed to `withAllianceRelativeControl` changing, is
published to the dashboard and is not overridden. Dashboard values outside the allowed range are replaced with
the stream's current value.

Call `withTelemetry` last, after the rest of the configuration, so the dashboard starts with the configured values.
A `clone()` of a stream has no telemetry. Calling `withTelemetry` again replaces the stream's telemetry, and
`getTelemetry().get().close()` stops publishing it, e.g. when replacing a stream with another of the same name.

## Java usage

The API is the same in Commands v2 (`yams.commands2.swerve.SwerveInputStream`) and Commands v3
(`yams.commands3.swerve.SwerveInputStream`). The `Live Tuning` command is a Commands v2 `Command` in v2, and a
Commands v3 `Command` published through `CommandTunable` in v3. Neither requires a subsystem or mechanism, so it
does not interrupt the command driving with the stream.

### Commands v2

```java
SwerveInputStream driveStream = SwerveInputStream.of(drive, () -> -controller.getLeftY(), () -> -controller.getLeftX())
    .withControllerRotationAxis(() -> -controller.getRightX())
    .withDeadband(0.05)
    .withScaleTranslation(0.8)
    .withTelemetry("Driver", TelemetryVerbosity.HIGH);

// SwerveInputStream output is field relative.
swerve.setDefaultCommand(swerve.run(() -> drive.setFieldRelativeChassisSpeeds(driveStream.get())));
```

### Commands v3

```java
// In a teleop opmode.
robot.drive.setInputStream(SwerveInputStream.of(robot.drive.getSwerveDrive(),
                                                () -> -controller.getLeftY(),
                                                () -> -controller.getLeftX(),
                                                () -> -controller.getRightX())
    .withDeadband(0.05)
    .withScaleTranslation(0.8)
    .withTelemetry("Driver", TelemetryVerbosity.HIGH));
robot.drive.setDefaultCommand(robot.drive.driveInputStream());
```

## C++ usage

C++ has the same API: `WithTelemetry(name, verbosity)` and `GetTelemetry()`, which returns a
`std::optional<std::reference_wrapper<SwerveInputStreamTelemetry<N>>>`. The verbosities are
`SwerveDriveConfig::TelemetryVerbosity`: `LOW`, `MEDIUM` (Java's `MID`) and `HIGH`, and `NONE` turns telemetry off.
The `Live Tuning` command is a Commands v2 `Command`, from `GetLiveTuningCommand()`.

The stream is a value type, so ownership works differently from Java:

- `WithTelemetry` stores the name and verbosity. The telemetry is created when the stream is first read with
  `Get()` or `GetTelemetry()` is called, so the copies made while building a stream do not publish.
- The stream owns its telemetry and stops publishing when it is destroyed.
- A copy of a stream keeps the telemetry settings and publishes under the same name once it is read. `Clone()`
  returns a copy without telemetry.
- Moving a stream moves the settings; the moved-from stream stops publishing.

```cpp
#include "yams/mechanisms/swerve/utility/SwerveInputStream.hpp"

using namespace yams::mechanisms::swerve::utility;

// Member of your subsystem.
SwerveInputStream<4> m_driveStream = SwerveInputStream<4>::Of(m_drive,
    [this]{ return -m_driverController.GetLeftY(); },
    [this]{ return -m_driverController.GetLeftX(); })
  .WithControllerRotationAxis([this]{ return -m_driverController.GetRightX(); })
  .WithDeadband(0.05)
  .WithScaleTranslation(0.8)
  .WithTelemetry("Driver", SwerveDriveConfig::TelemetryVerbosity::HIGH);

// In Periodic(). SwerveInputStream output is field relative.
void Periodic() override {
    m_drive.SetFieldRelativeChassisSpeeds(m_driveStream.Get());
}
```

## NetworkTables topics

### State (read-only)

Published to `/SwerveInputStream/<name>/`. `mode` at every verbosity; the configuration values below at `MID`
(`MEDIUM` in C++) and `HIGH`.

| Topic | Type | Description |
|-------|------|-------------|
| `mode` | string | Current drive mode: `ANGULAR_VELOCITY`, `HEADING`, `AIM`, `TRANSLATION_ONLY` |

The output velocities are not published: reading them would call `get()` a second time each loop, which
runs the heading controller twice. Log the speeds you send to the drive instead.

### Live tuning

At `HIGH`, published to `/Tuning/SwerveInputStream/<name>/`, with the `Live Tuning` command in
`/Tuning/SwerveInputStream/<name>/Live Tuning`. The same topics are published read-only under
`/SwerveInputStream/<name>/` at `MID` and `HIGH`.

| Topic | Type | Range | Description |
|-------|------|-------|-------------|
| `deadband` | double | [0, 1) | Controller axis deadband |
| `translationScale` | double | (0, 1] | Translation axis scaling factor |
| `rotationScale` | double | (0, 1] | Rotation axis scaling factor |
| `maxLinearVelocity` | double | > 0 | Maximum chassis linear velocity (m/s). Starts at the drive config's maximum and overrides it |
| `maxAngularVelocity` | double | > 0 | Maximum chassis angular velocity (rad/s). Starts at the drive config's maximum and overrides it |
| `translationCube` | boolean | - | Enable cubic translation response curve |
| `rotationCube` | boolean | - | Enable cubic rotation response curve |
| `allianceRelative` | boolean | - | Enable alliance-relative translation flip |
| `robotRelative` | boolean | - | Enable robot-relative output |

## Dashboard integration

### Elastic example

For a stream with `withTelemetry("Driver", TelemetryVerbosity.HIGH)`, open a tab in
[Elastic](https://frc-elastic.gitbook.io/docs) and drag these keys from the NetworkTables tree onto it:

1. **The drive mode**, `SwerveInputStream/Driver/mode`, as a text display.

2. **The `Live Tuning` command**, `Tuning/SwerveInputStream/Driver/Live Tuning`. Press its button to start it.
   Edits below only apply while it runs.

3. **The numbers to tune**, from `Tuning/SwerveInputStream/Driver`, as number sliders. Set each slider's
   range in its properties:
   - `deadband`: 0.0 to 0.1
   - `translationScale`: 0.1 to 1.0
   - `rotationScale`: 0.1 to 1.0
   - `maxLinearVelocity`: 0.1 to 10 m/s
   - `maxAngularVelocity`: 0.1 to 2π (6.283) rad/s

4. **The features**, from `Tuning/SwerveInputStream/Driver`, as toggle switches:
   - `translationCube`, `rotationCube`, `allianceRelative` and `robotRelative`

Elastic can always write to NetworkTables, so there is nothing to turn on. Save the layout from Elastic's
File menu to keep the tab for next time.

## Tuning workflow

### 1. Measure a baseline
- Deploy with telemetry at `TelemetryVerbosity.HIGH`
- Note the values the stream starts with
- Drive the robot and note how it responds

### 2. Adjust values live
- Start the `Live Tuning` command, then change values on the dashboard. They apply on the next loop while it runs
- No need to rebuild or redeploy

### 3. Fine-tune
- **Deadband**: Increase if the joystick is jittery at rest
- **Translation scale**: Reduce for slower, more precise movements
- **Rotation scale**: Adjust for comfortable rotation feel
- **Cube modes**: Enable for progressive control (sensitive at small inputs, full power at full input)

### 4. Save the values
- Once tuned, copy the final values from NetworkTables into your robot code
- Commit to version control

## Best practices

1. **Publish during development only**: use `TelemetryVerbosity.LOW` in competition code. In Java,
   `getTelemetry().get().close()` stops publishing; in C++, `WithTelemetry(name, TelemetryVerbosity::NONE)` does
2. **Use descriptive names**: Give each SwerveInputStream a clear name (e.g., "drive", "intake_aiming")
3. **Watch the mode and features**: check the mode and which features are on during testing, and log the speeds you send to the drive, since the stream doesn't publish them
4. **Document final values**: Keep notes on what worked best for your chassis and driver

## Troubleshooting

| Issue | Solution |
|-------|----------|
| Telemetry not appearing | Check the stream was given `withTelemetry` and is read with `get()` every loop |
| Changes don't apply | Start the `Live Tuning` command; it only exists at `TelemetryVerbosity.HIGH` |
| A dashboard value snaps back | Enter a value inside its range in the [live tuning table](#live-tuning) |
| Live tuning causes instability | Use slider ranges that respect your drivetrain limits |
| High NetworkTables latency | Lower the stream's verbosity to `LOW`, or stop its telemetry as in [best practices](#best-practices) |

## See also

- [Swerve input stream](https://yams.yassrobotics.com/documentation/details/swerve-input-stream) in the YAMS documentation
- [`SwerveInputStream` Java API reference](https://yams.yassrobotics.com/api-reference/java-reference/swerve/swerve-input-stream)
- [`SwerveInputStream` C++ API reference](https://yams.yassrobotics.com/api-reference/c++-reference/swerve/swerve-input-stream)
