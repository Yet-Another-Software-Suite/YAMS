# SwerveModule

**Namespace:** `yams::mechanisms::swerve`\
**Header:** `#include <yams/mechanisms/swerve/SwerveModule.hpp>`\
**Java equivalent:** [Java SwerveModule](../../java-reference/swerve/swerve-module.md)\
**Doxygen:** [`SwerveModule`](https://doxygen.yassrobotics.com/yams/classyams_1_1mechanisms_1_1swerve_1_1SwerveModule.html)

`SwerveModule` represents one swerve module: a drive motor and an azimuth motor. You construct one per corner from a `SwerveModuleConfig` (which must outlive the module) and pass pointers to them into `SwerveDriveConfig::WithModules(...)`; `SwerveDrive<N>` then drives, simulates and publishes them.

***

## Constructor

```cpp
explicit SwerveModule(config::SwerveModuleConfig* config)
```

***

## State

| Method                       | Signature                                                 | Description                                                                              |
| ---------------------------- | --------------------------------------------------------- | ---------------------------------------------------------------------------------------- |
| `GetState`                   | `wpi::math::SwerveModuleVelocity GetState() const`        | Current drive velocity and azimuth angle.                                                |
| `GetPosition`                | `wpi::math::SwerveModulePosition GetPosition() const`     | Integrated drive distance and azimuth angle.                                             |
| `GetName`                    | `std::string GetName() const`                             | Module name from config.                                                                 |
| `GetConfig`                  | `const config::SwerveModuleConfig& GetConfig() const`     | Module configuration.                                                                    |
| `GetRawAbsoluteEncoderAngle` | `wpi::units::degree_t GetRawAbsoluteEncoderAngle() const` | Absolute encoder angle with no offset applied: the raw reading captured at construction. |

***

## Control

| Method                 | Signature                                                                      | Description                                                                                                                                                                                      |
| ---------------------- | ------------------------------------------------------------------------------ | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| `SetSwerveModuleState` | `void SetSwerveModuleState(wpi::math::SwerveModuleVelocity state)`             | Commands drive speed and azimuth angle.                                                                                                                                                          |
| `SetAzimuthAngle`      | `wpi::math::SwerveModuleVelocity SetAzimuthAngle(wpi::math::Rotation2d angle)` | Turns the wheel to `angle` with zero drive speed, skipping the velocity deadband, and returns the commanded state. Used by `LockPose()` and `SetAzimuthAngles(...)`.                             |
| `GetCouplingVelocity`  | `wpi::units::meters_per_second_t GetCouplingVelocity() const`                  | The wheel velocity the drive encoder reads because the azimuth is turning, which a drive held still must be commanded to.                                                                        |
| `SetupTelemetry`       | `void SetupTelemetry(const std::string& mechName)`                             | Wires up telemetry under `Mechanisms/<mechName>/modules/<moduleName>`. Called by `SwerveDrive<N>`; see below.                                                                                    |
| `SeedAzimuthEncoder`   | `void SeedAzimuthEncoder()`                                                    | Reads the absolute encoder and seeds the azimuth motor's relative encoder (skipped in simulation and when the azimuth uses an external feedback encoder). The constructor already calls it once. |

***

## Motor access

| Method                      | Signature                                                                   | Description                   |
| --------------------------- | --------------------------------------------------------------------------- | ----------------------------- |
| `GetDriveMotorController`   | `motorcontrollers::SmartMotorController* GetDriveMotorController() const`   | Pointer to the drive motor.   |
| `GetAzimuthMotorController` | `motorcontrollers::SmartMotorController* GetAzimuthMotorController() const` | Pointer to the azimuth motor. |

***

## Lifecycle

| Method            | Signature                | Description                              |
| ----------------- | ------------------------ | ---------------------------------------- |
| `UpdateTelemetry` | `void UpdateTelemetry()` | Publishes module state to NetworkTables. |
| `SimIterate`      | `void SimIterate()`      | Advances module simulation.              |

***

## Telemetry & DataLog

`SetupTelemetry(const std::string& mechName)` wires up this module's telemetry under `Mechanisms/<mechName>/modules/<moduleName>`; `SwerveDrive<N>` calls it automatically for every module during its own construction, so you don't normally call it yourself. `UpdateTelemetry()` then publishes this module's `SwerveModuleVelocity` and absolute encoder angle to NetworkTables at the verbosity from `SwerveModuleConfig::GetTelemetryVerbosity()`, or from an explicit `telemetry::SwerveModuleTelemetryConfig` passed via `SwerveModuleConfig::WithTelemetry(name, SwerveModuleTelemetryConfig)`; see [SwerveModuleConfig](swerve-module-config.md#datalog-telemetry) for how to additionally record fields to a WPILib DataLog. For field-level control (or a DataLog name) on the drive/azimuth motors themselves, configure each motor's own `SmartMotorControllerConfig::WithTelemetry(name, SmartMotorControllerTelemetryConfig)`.

{% hint style="warning" %}
`SwerveModuleConfig` has no `WithDataLogName(const std::string&)` method: `SetupTelemetry(...)` only consults the `SwerveModuleTelemetryConfig`'s own, separate DataLog name. Use `SwerveModuleConfig::WithTelemetry(name, telemetry::SwerveModuleTelemetryConfig{}.WithDataLogName(...))` instead; see [SwerveModuleConfig](swerve-module-config.md#datalog-telemetry).
{% endhint %}

***

## See also

* [Java SwerveModule](../../java-reference/swerve/swerve-module.md)
* [SwerveModuleConfig](swerve-module-config.md)
* [SwerveDrive\<N>](swerve-drive.md)
