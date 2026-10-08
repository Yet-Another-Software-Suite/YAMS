# SwerveInputStream Telemetry and Live Tuning

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

## Java Usage

The API is the same in Commands v2 (`yams.commands2.swerve.SwerveInputStream`) and Commands v3
(`yams.commands3.swerve.SwerveInputStream`). The `Live Tuning` command is a commands v2 `Command` in v2, and a
commands v3 `Command` published through `CommandTunable` in v3. Neither requires a subsystem or mechanism, so it
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

## C++ Usage

C++ has the same API: `WithTelemetry(name, verbosity)` and `GetTelemetry()`, which returns a
`std::optional<std::reference_wrapper<SwerveInputStreamTelemetry<N>>>`. The verbosities are
`SwerveDriveConfig::TelemetryVerbosity`: `LOW`, `MEDIUM` (Java's `MID`) and `HIGH`, and `NONE` turns telemetry off.
The `Live Tuning` command is a commands v2 `Command`, from `GetLiveTuningCommand()`.

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

## NetworkTables Topics

### State (Read-Only)

Published to `/SwerveInputStream/<name>/`. `mode` at every verbosity; the configuration values below at `MID`
(`MEDIUM` in C++) and `HIGH`.

| Topic | Type | Description |
|-------|------|-------------|
| `mode` | string | Current drive mode: `ANGULAR_VELOCITY`, `HEADING`, `AIM`, `TRANSLATION_ONLY` |

The output velocities are not published: reading them would call `get()` a second time each loop, which
runs the heading controller twice. Log the speeds you send to the drive instead.

### Live Tuning

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

## Dashboard Integration

### Shuffleboard Example

For a stream with `withTelemetry("Driver", TelemetryVerbosity.HIGH)`, create a tab and add widgets:

1. **Display widgets** for state monitoring, from `SwerveInputStream/Driver`:
   - Mode indicator (String)

2. **Live Tuning toggle**, from `Tuning/SwerveInputStream/Driver/Live Tuning`. Edits below only apply while it
   runs.

3. **Slider widgets** for tuning, from `Tuning/SwerveInputStream/Driver`:
   - Deadband: slider 0.0 to 0.1
   - Translation Scale: slider 0.1 to 1.0
   - Rotation Scale: slider 0.1 to 1.0
   - Max Linear Velocity: slider 0 to 10 m/s
   - Max Angular Velocity: slider 0 to 2π rad/s

4. **Toggle widgets** for features, from `Tuning/SwerveInputStream/Driver`:
   - Translation Cube, Rotation Cube, Alliance Relative, Robot Relative

Example JSON for Shuffleboard:
```json
{
  "SwerveInputStream/Driver": {
    "mode": {"class": "String", "position": [0, 0]}
  },
  "Tuning/SwerveInputStream/Driver": {
    "Live Tuning": {"class": "Command", "position": [1, 0]},
    "deadband": {"class": "Slider", "position": [0, 1], "min": 0.0, "max": 0.1},
    "translationScale": {"class": "Slider", "position": [1, 1], "min": 0.1, "max": 1.0},
    "rotationScale": {"class": "Slider", "position": [2, 1], "min": 0.1, "max": 1.0},
    "maxLinearVelocity": {"class": "Slider", "position": [0, 2], "min": 0.0, "max": 10.0},
    "maxAngularVelocity": {"class": "Slider", "position": [1, 2], "min": 0.0, "max": 6.283},
    "translationCube": {"class": "Toggle Button", "position": [0, 3]},
    "rotationCube": {"class": "Toggle Button", "position": [1, 3]},
    "allianceRelative": {"class": "Toggle Button", "position": [2, 3]},
    "robotRelative": {"class": "Toggle Button", "position": [3, 3]}
  }
}
```

## Tuning Workflow

### 1. Measure Baseline Performance
- Deploy with telemetry enabled
- Record current control parameters
- Test vehicle responsiveness

### 2. Adjust Parameters Live
- Dashboard updates take effect immediately in the next robot loop
- No need to rebuild or redeploy

### 3. Fine-Tune
- **Deadband**: Increase if the joystick is jittery at rest
- **Translation Scale**: Reduce for slower, more precise movements
- **Rotation Scale**: Adjust for comfortable rotation feel
- **Cube modes**: Enable for progressive control (sensitive at small inputs, full power at full input)

### 4. Save Parameters
- Once tuned, copy the final values from NetworkTables into your robot code
- Commit to version control

## Best Practices

1. **Publish during development only**: use `TelemetryVerbosity.LOW` in competition code. In Java,
   `getTelemetry().get().close()` stops publishing; in C++, `WithTelemetry(name, TelemetryVerbosity::NONE)` does
2. **Use descriptive names**: Give each SwerveInputStream a clear name (e.g., "drive", "intake_aiming")
3. **Monitor all parameters**: Check mode, velocities, and active features during testing
4. **Document final values**: Keep notes on what worked best for your chassis and driver

## Troubleshooting

| Issue | Solution |
|-------|----------|
| Telemetry not appearing | Check the stream was given `withTelemetry` and is read with `get()` every loop |
| Changes don't apply | Start the `Live Tuning` command; it only exists at `TelemetryVerbosity.HIGH` |
| A dashboard value snaps back | The value is outside the allowed range in the Live Tuning table |
| Live tuning causes instability | Use slider ranges that respect your drivetrain limits |
| High NetworkTables latency | Reduce update frequency or disable verbose logging on the dashboard |

## See Also

- `SwerveInputStream` documentation
- NetworkTables integration guides
- Dashboard software (Shuffleboard, AdvantageScope)
