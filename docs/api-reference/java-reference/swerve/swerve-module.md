# SwerveModule

**Package:** `yams.core.mechanisms.swerve`\
**Javadoc:** [`SwerveModule`](https://javadocs.yassrobotics.com/yams/yams/core/mechanisms/swerve/SwerveModule.html)

Represents one swerve module: a drive motor and an azimuth (steering) motor. You construct one `SwerveModule` per corner and pass them to the [`SwerveDriveConfig`](swerve-drive-config.md); `SwerveDrive` then manages them. The same class is used with Commands v2 and Commands v3.

## Constructor

```java
SwerveModule(SwerveModuleConfig config)
```

| Parameter | Type                 | Description                                                                |
| --------- | -------------------- | -------------------------------------------------------------------------- |
| `config`  | `SwerveModuleConfig` | Module-level configuration including motors, encoder offset, and location. |

***

## State

| Method                         | Returns                           | Description                                                                                                                                  |
| ------------------------------ | --------------------------------- | -------------------------------------------------------------------------------------------------------------------------------------------- |
| `getState()`                   | `SwerveModuleVelocityWithAzimuth` | Current drive velocity and azimuth angle, plus how fast the azimuth is turning (`azimuthVelocity`). Extends WPILib's `SwerveModuleVelocity`. |
| `getPosition()`                | `SwerveModulePosition`            | Integrated drive distance and azimuth angle.                                                                                                 |
| `getName()`                    | `String`                          | Module name as set in config.                                                                                                                |
| `getConfig()`                  | `SwerveModuleConfig`              | The configuration this module was constructed with.                                                                                          |
| `getRawAbsoluteEncoderAngle()` | `Angle`                           | Absolute encoder angle with no offset applied.                                                                                               |
| `getDriveMotorController()`    | `SmartMotorController`            | The drive motor.                                                                                                                             |
| `getAzimuthMotorController()`  | `SmartMotorController`            | The azimuth motor.                                                                                                                           |

***

## Control

| Method                                                                     | Returns                | Description                                                                                                                                                                                                                                                                                                                                                                      |
| -------------------------------------------------------------------------- | ---------------------- | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `setSwerveModuleState(SwerveModuleVelocity state)`                         | `SwerveModuleVelocity` | Commands a drive speed and azimuth angle. Returns the optimized state actually sent.                                                                                                                                                                                                                                                                                             |
| `setSwerveModuleState(SwerveModuleVelocity state, Force feedforwardForce)` | `SwerveModuleVelocity` | Same, plus a drive-wheel feedforward `Force` applied on top (e.g. from a PathPlanner setpoint generator). The force is reversed when optimization turns the wheel around and scaled by cosine compensation when enabled.                                                                                                                                                         |
| `setAzimuthAngle(Rotation2d angle)`                                        | `SwerveModuleVelocity` | Turns the wheel to `angle` without driving it, as for locking the drive in place or tuning the azimuth motor. Unlike `setSwerveModuleState`, the velocity deadband does not hold the wheel where it is (see [`SwerveModuleConfig.getLockedState(...)`](swerve-module-config.md#state-helpers)). Used by `SwerveDrive.lockPose()` and `SwerveDrive.setAzimuthAngles(Rotation2d)`. |
| `getCouplingVelocity()`                                                    | `LinearVelocity`       | The wheel velocity the drive encoder reads because the azimuth is turning, which a drive held still must be commanded to.                                                                                                                                                                                                                                                        |
| `seedAzimuthEncoder()`                                                     | `void`                 | Reads the absolute encoder and seeds the relative encoder to match. Call on robot enable.                                                                                                                                                                                                                                                                                        |

***

## Lifecycle

| Method              | Returns | Description                                                                                       |
| ------------------- | ------- | ------------------------------------------------------------------------------------------------- |
| `updateTelemetry()` | `void`  | Publishes module state and encoder readings to NetworkTables, and updates both motors' telemetry. |
| `simIterate()`      | `void`  | Advances module simulation by one loop iteration.                                                 |

***

## Telemetry and DataLog

`setupTelemetry(String mechName)` wires up this module's telemetry under `Mechanisms/<mechName>/modules/<moduleName>`; `SwerveDrive` calls it automatically for every module during its own construction, so you don't normally call it yourself. `updateTelemetry()` then publishes this module's `SwerveModuleVelocity` and absolute encoder angle to NetworkTables at the verbosity from `SwerveModuleConfig.getTelemetryVerbosity()`, or from an explicit `SwerveModuleTelemetryConfig` passed via `SwerveModuleConfig.withTelemetry(name, SwerveModuleTelemetryConfig)`; see [SwerveModuleConfig](swerve-module-config.md#datalog-telemetry) for how to additionally record fields to a WPILib DataLog. For field-level control (or a DataLog name) on the drive/azimuth motors themselves, configure each motor's own `SmartMotorControllerConfig.withTelemetry(name, SmartMotorControllerTelemetryConfig)`.

{% hint style="warning" %}
`SwerveModuleConfig` has no `withDataLogName(String)` method: `setupTelemetry(...)` only consults the `SwerveModuleTelemetryConfig`'s own, separate DataLog name. Use `SwerveModuleConfig.withTelemetry(name, new SwerveModuleTelemetryConfig().withDataLogName(...))` instead; see [SwerveModuleConfig](swerve-module-config.md#datalog-telemetry).
{% endhint %}

***

## See also

* [SwerveModuleConfig](swerve-module-config.md)
* [C++ SwerveModule](../../c++-reference/swerve/swerve-module.md)
