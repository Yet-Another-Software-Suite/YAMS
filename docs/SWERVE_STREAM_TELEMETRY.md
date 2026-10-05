# SwerveInputStream Telemetry and Live Tuning

This document describes the telemetry and live tuning features for `SwerveInputStream`.

## Overview

`SwerveInputStreamTelemetry` publishes real-time telemetry from a `SwerveInputStream` to NetworkTables, enabling:

- **Monitoring**: View current drive state (velocities, mode, etc.) on the dashboard
- **Live tuning**: Adjust control parameters (deadband, scaling, max velocities) without redeploying code
- **Debugging**: Quickly identify input issues or calibration problems

## Java Usage

### Basic Setup (Commands v2)

```java
import yams.commands2.swerve.SwerveInputStream;
import yams.commands2.swerve.SwerveInputStreamTelemetry;

// In your subsystem or command initialization:
SwerveInputStream driveStream = SwerveInputStream.of(drive, leftY, leftX)
    .withControllerRotationAxis(rightX)
    .withDeadband(0.05)
    .withScaleTranslation(0.8);

SwerveInputStreamTelemetry telemetry = new SwerveInputStreamTelemetry(driveStream, "drive");

// In your periodic or command execute:
void periodic() {
    telemetry.update();
    drive.setRobotRelativeChassisSpeeds(driveStream.get());
}
```

### Commands v3 Example

```java
import yams.commands3.swerve.SwerveInputStream;
import yams.commands3.swerve.SwerveInputStreamTelemetry;

var telemetry = new SwerveInputStreamTelemetry(input, "drive");

public Command teleop(SwerveInputStream input) {
    return drive.run(coroutine -> {
        while (true) {
            telemetry.update();
            drive.setRobotRelativeChassisSpeeds(input.get());
            coroutine.yield();
        }
    });
}
```

## C++ Usage

```cpp
#include "yams/mechanisms/swerve/utility/SwerveInputStreamTelemetry.hpp"

using namespace yams::mechanisms::swerve::utility;

// In your subsystem:
auto driveStream = SwerveInputStream<4>::Of(m_drive,
    [this]{ return -m_driverController.GetLeftY(); },
    [this]{ return -m_driverController.GetLeftX(); })
  .WithControllerRotationAxis([this]{ return -m_driverController.GetRightX(); })
  .WithDeadband(0.05)
  .WithScaleTranslation(0.8);

auto telemetry = std::make_unique<SwerveInputStreamTelemetry<4>>(driveStream, "drive");

// In Periodic():
void Periodic() override {
    telemetry->Update();
    m_drive.Drive([this]{ return m_driveStream.Get(); });
}
```

## NetworkTables Topics

Published to `/SwerveInputStream/<name>/`:

### State (Read-Only)

| Topic | Type | Description |
|-------|------|-------------|
| `mode` | string | Current drive mode: `ANGULAR_VELOCITY`, `HEADING`, `AIM`, `TRANSLATION_ONLY` |
| `vx` | double | Forward/back velocity (m/s) |
| `vy` | double | Left/right velocity (m/s) |
| `omega` | double | Rotation velocity (rad/s) |

### Live Tuning

| Topic | Type | Range | Description |
|-------|------|-------|-------------|
| `deadband` | double | [0, 1) | Controller axis deadband |
| `translationScale` | double | (0, 1] | Translation axis scaling factor |
| `rotationScale` | double | (0, 1] | Rotation axis scaling factor |
| `maxLinearVelocity` | double | > 0 | Maximum chassis linear velocity (m/s) |
| `maxAngularVelocity` | double | > 0 | Maximum chassis angular velocity (rad/s) |
| `translationCube` | boolean | - | Enable cubic translation response curve |
| `rotationCube` | boolean | - | Enable cubic rotation response curve |
| `allianceRelative` | boolean | - | Enable alliance-relative translation flip |
| `robotRelative` | boolean | - | Enable robot-relative output |

## Dashboard Integration

### Shuffleboard Example

Create a tab and add widgets for the SwerveInputStream topics:

1. **Display widgets** for state monitoring:
   - Mode indicator (String)
   - Velocity gauges (Vx, Vy, Omega)

2. **Slider widgets** for tuning:
   - Deadband: slider 0.0 to 0.1
   - Translation Scale: slider 0.1 to 1.0
   - Rotation Scale: slider 0.1 to 1.0
   - Max Linear Velocity: slider 0 to 10 m/s
   - Max Angular Velocity: slider 0 to 2π rad/s

3. **Toggle widgets** for features:
   - Translation Cube, Rotation Cube, Alliance Relative, Robot Relative

Example JSON for Shuffleboard:
```json
{
  "SwerveInputStream/drive": {
    "mode": {"class": "String", "position": [0, 0]},
    "vx": {"class": "Gauge", "position": [1, 0]},
    "vy": {"class": "Gauge", "position": [2, 0]},
    "deadband": {"class": "Slider", "position": [0, 1], "min": 0.0, "max": 0.1},
    "translationScale": {"class": "Slider", "position": [1, 1], "min": 0.1, "max": 1.0},
    "rotationScale": {"class": "Slider", "position": [2, 1], "min": 0.1, "max": 1.0}
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

1. **Publish during development only**: Disable telemetry in competition code to save bandwidth
2. **Use descriptive names**: Give each SwerveInputStream a clear name (e.g., "drive", "intake_aiming")
3. **Monitor all parameters**: Check mode, velocities, and active features during testing
4. **Document final values**: Keep notes on what worked best for your chassis and driver

## Troubleshooting

| Issue | Solution |
|-------|----------|
| Telemetry not appearing | Check NetworkTables connection; ensure `update()` is called every loop |
| Changes don't apply immediately | Verify the robot code is reading the published values |
| Live tuning causes instability | Use slider ranges that respect your drivetrain limits |
| High NetworkTables latency | Reduce update frequency or disable verbose logging on the dashboard |

## See Also

- `SwerveInputStream` documentation
- NetworkTables integration guides
- Dashboard software (Shuffleboard, AdvantageScope)
