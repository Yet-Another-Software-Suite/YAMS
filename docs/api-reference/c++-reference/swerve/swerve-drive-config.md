# SwerveDriveConfig

**Namespace:** `yams::mechanisms::swerve`\
**Header:** `#include <yams/mechanisms/swerve/SwerveDriveConfig.hpp>`\
**Java equivalent:** [Java SwerveDriveConfig](../../java-reference/swerve/swerve-drive-config.md)\
**Doxygen:** [`SwerveDriveConfig`](https://doxygen.yassrobotics.com/yams/classyams_1_1mechanisms_1_1swerve_1_1SwerveDriveConfig.html)

Configures a complete `SwerveDrive<N>`. Uses a fluent builder pattern: every `With*` method returns `*this` for chaining.

***

## Constructor

```cpp
SwerveDriveConfig() = default;
```

Build the config with `WithSubsystem()` and `WithModules()`, then pass a pointer to it into `SwerveDrive<N>`'s constructor.

***

## Type alias

```cpp
using TelemetryVerbosity = motorcontrollers::SmartMotorControllerConfig::TelemetryVerbosity;
```

***

## Builder methods

All methods return `SwerveDriveConfig&` for chaining.

| Method                                  | Parameters                                                                                | Description                                                                                                                                                                                        |
| --------------------------------------- | ----------------------------------------------------------------------------------------- | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `WithSubsystem`                         | `wpi::cmd::SubsystemBase* subsystem`                                                      | Associates a command subsystem with this drive (used for command requirements).                                                                                                                    |
| `WithModules`                           | `std::vector<SwerveModule*> modules`                                                      | Sets the modules that make up this drive. Vector size must equal the `NumModules` template argument of `SwerveDrive`.                                                                              |
| `WithRotation`                          | `std::function<wpi::math::Rotation3d()> rotationSupplier`                                 | **Required** (or `WithGyro`). Supplies the robot's rotation from the gyro, as its full attitude, for field-relative control and odometry.                                                          |
| `WithGyro`                              | `std::function<wpi::math::Rotation3d()> gyroSupplier`                                     | Same as `WithRotation`.                                                                                                                                                                            |
| `WithGyro`                              | `std::function<wpi::units::degree_t()> gyroSupplier`                                      | Supplies the gyro heading for a gyro mounted flat, as `WithRotation` with no roll or pitch.                                                                                                        |
| `WithGyroVelocity`                      | `std::function<wpi::units::degrees_per_second_t()> angularVelocitySupplier`               | Supplies gyro angular velocity. Required for angular-velocity skew correction.                                                                                                                     |
| `WithGyroInverted`                      | `bool inverted`                                                                           | Negates the raw gyro angle when `true`.                                                                                                                                                            |
| `WithStartingPose`                      | `wpi::math::Pose2d pose`                                                                  | Initial field-relative odometry pose. Defaults to the origin.                                                                                                                                      |
| `WithMaximumChassisSpeed`               | `wpi::units::meters_per_second_t speed, wpi::units::degrees_per_second_t angularVelocity` | Maximum chassis linear/angular speeds used for wheel-speed desaturation.                                                                                                                           |
| `WithMaximumModuleSpeed`                | `wpi::units::meters_per_second_t speed`                                                   | Maximum module linear speed used for wheel-speed desaturation.                                                                                                                                     |
| `WithCenterOfRotation`                  | `wpi::math::Translation2d center`                                                         | Overrides the chassis center of rotation (default is the robot center).                                                                                                                            |
| `WithCenterOfRotation`                  | `wpi::units::meter_t forward, wpi::units::meter_t left`                                   | Overrides the center of rotation via forward/left distances.                                                                                                                                       |
| `WithDiscretizationTime`                | `wpi::units::second_t dt`                                                                 | Discretization timestep used to compensate for `ChassisVelocities` skew.                                                                                                                           |
| `WithSimDiscretizationTime`             | `wpi::units::second_t dt`                                                                 | Discretization timestep used only in simulation. Falls back to `WithDiscretizationTime` if unset.                                                                                                  |
| `WithGyroAngularVelocityScaleFactor`    | `double scaleFactor`                                                                      | Scale factor `[0, 1]` applied to gyro angular velocity for skew correction on the real robot.                                                                                                      |
| `WithSimGyroAngularVelocityScaleFactor` | `double scaleFactor`                                                                      | Scale factor used only in simulation. Falls back to `WithGyroAngularVelocityScaleFactor` if unset.                                                                                                 |
| `WithTranslationController`             | `wpi::math::PIDController controller`                                                     | PID controller (meters) used by `DriveToPose`.                                                                                                                                                     |
| `WithRotationController`                | `wpi::math::PIDController controller`                                                     | PID controller (radians) used by `DriveToPose`. Continuous input over `[-pi, pi]` is enabled automatically.                                                                                        |
| `WithSimTranslationController`          | `wpi::math::PIDController controller`                                                     | Translation controller used only in simulation. Falls back to `WithTranslationController` if unset.                                                                                                |
| `WithSimRotationController`             | `wpi::math::PIDController controller`                                                     | Rotation controller used only in simulation. Falls back to `WithRotationController` if unset.                                                                                                      |
| `WithTelemetry`                         | `const std::string& name, TelemetryVerbosity verbosity`                                   | Sets the telemetry name (NetworkTables/DataLog prefix, defaults to `"swerve"`) and verbosity for drive-level fields (pose, gyro, chassis speeds, module states, auto-align PID gains) in one call. |
| `WithTelemetry`                         | `const std::string& name, telemetry::SwerveDriveTelemetryConfig telemetryConfig`          | Sets the telemetry name and configures telemetry with an explicit `SwerveDriveTelemetryConfig`, taking precedence over the verbosity-based overload.                                               |

{% hint style="info" %}
`WithTranslationController` and `WithRotationController` are optional, but `SwerveDrive::DriveToPose(...)` and `DriveToPoseSetpoint(...)` throw a `SwerveDriveConfigurationException` if either is missing, and the `tuning` live-tuning command is only published when both are configured.
{% endhint %}

{% hint style="info" %}
`WithRotation` (or `WithGyro`, which calls it) is required. Omitting it causes `GetGyroAngle()` to throw a `std::runtime_error` and prevents field-relative control and odometry from functioning correctly.

Both `WithTelemetry` overloads require a name; there is no name-less `WithTelemetry(...)` or standalone `WithTelemetryName(...)` on `SwerveDriveConfig` (unlike, say, `SmartMotorControllerConfig`). `SwerveDriveConfig` also has no `WithDataLogName(...)`/`GetDataLogName()` of its own: DataLog output is configured on a `SwerveDriveTelemetryConfig` instead. See below.
{% endhint %}

***

## DataLog telemetry

`SwerveDriveConfig` has no `WithDataLogName(...)` method: DataLog output is configured on a [`SwerveDriveTelemetryConfig`](swerve-drive.md#telemetry--datalog) and wired in via `WithTelemetry(name, SwerveDriveTelemetryConfig)`:

```cpp
SwerveDriveConfig driveConfig;
driveConfig.WithSubsystem(this)
    .WithModules({&frontLeft, &frontRight, &backLeft, &backRight})
    .WithGyro([gyroPtr = &gyro]() -> wpi::units::degree_t {
      return wpi::units::degree_t{wpi::units::turn_t{gyroPtr->GetYaw().GetValue()}};
    })
    .WithTelemetry("swerve",
        yams::telemetry::SwerveDriveTelemetryConfig{SwerveDriveConfig::TelemetryVerbosity::HIGH}
            .WithDataLogName("swerve"));  // logs pose, gyro, chassis speeds, module states
```

`SwerveDriveTelemetryConfig{TelemetryVerbosity}` is shorthand for `SwerveDriveTelemetryConfig{}.WithTelemetryVerbosity(verbosity)`.

{% hint style="warning" %}
`WithTelemetry(name, SwerveDriveTelemetryConfig)` replaces the default field selection entirely; call `WithTelemetryVerbosity(...)` (or the individual `With*()` field methods) on it yourself, since a default-constructed `SwerveDriveTelemetryConfig` has every field disabled. This DataLog name does **not** cascade to the modules passed to `WithModules()`, or to those modules' drive/azimuth motors. Each `SwerveModuleConfig` needs its own telemetry setup (see [SwerveModuleConfig](swerve-module-config.md#datalog-telemetry)), and each motor's `SmartMotorControllerConfig` needs its own `WithTelemetry(name, SmartMotorControllerTelemetryConfig)` for field-level control and its own DataLog name.
{% endhint %}

***

## Getters

| Method                               | Signature                                                                                                    | Description                                                                                                                                                                                                                         |
| ------------------------------------ | ------------------------------------------------------------------------------------------------------------ | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `GetModules`                         | `const std::vector<SwerveModule*>& GetModules() const`                                                       | The configured module pointers.                                                                                                                                                                                                     |
| `GetSubsystem`                       | `wpi::cmd::SubsystemBase* GetSubsystem() const`                                                              | The associated subsystem.                                                                                                                                                                                                           |
| `GetInitialPose`                     | `wpi::math::Pose2d GetInitialPose() const`                                                                   | The starting pose.                                                                                                                                                                                                                  |
| `GetMaximumChassisLinearVelocity`    | `std::optional<wpi::units::meters_per_second_t> GetMaximumChassisLinearVelocity() const`                     | Optional max chassis linear speed.                                                                                                                                                                                                  |
| `GetMaximumChassisAngularVelocity`   | `std::optional<wpi::units::degrees_per_second_t> GetMaximumChassisAngularVelocity() const`                   | Optional max chassis angular speed.                                                                                                                                                                                                 |
| `GetMaximumModuleLinearVelocity`     | `std::optional<wpi::units::meters_per_second_t> GetMaximumModuleLinearVelocity() const`                      | Optional max module speed.                                                                                                                                                                                                          |
| `GetCenterOfRotation`                | `std::optional<wpi::math::Translation2d> GetCenterOfRotation() const`                                        | Optional center-of-rotation override.                                                                                                                                                                                               |
| `GetTelemetryVerbosity`              | `std::optional<TelemetryVerbosity> GetTelemetryVerbosity() const`                                            | Configured telemetry verbosity, if any.                                                                                                                                                                                             |
| `GetTelemetryName`                   | `const std::string& GetTelemetryName() const`                                                                | Telemetry name prefix for the drive (defaults to `"swerve"`).                                                                                                                                                                       |
| `GetSwerveDriveTelemetryConfig`      | `std::optional<telemetry::SwerveDriveTelemetryConfig> GetSwerveDriveTelemetryConfig()`                       | The explicit config passed to `WithTelemetry(name, SwerveDriveTelemetryConfig)`, if any. Moves it out; intended to be called exactly once, by `SwerveDrive` itself.                                                                 |
| `GetGyroAngle`                       | `wpi::units::degree_t GetGyroAngle() const`                                                                  | Gyro angle with inversion applied. Throws a `std::runtime_error` if no gyro supplier was configured.                                                                                                                                |
| `GetTranslationPID`                  | `std::optional<std::reference_wrapper<wpi::math::PIDController>> GetTranslationPID()`                        | Active translation controller (sim variant if simulating and configured), or `std::nullopt` if none was configured.                                                                                                                 |
| `GetRotationPID`                     | `std::optional<std::reference_wrapper<wpi::math::PIDController>> GetRotationPID()`                           | Active rotation controller (sim variant if simulating and configured), or `std::nullopt` if none was configured.                                                                                                                    |
| `OptimizeRobotRelativeChassisSpeeds` | `wpi::math::ChassisVelocities OptimizeRobotRelativeChassisSpeeds(wpi::math::ChassisVelocities speeds) const` | Applies angular-velocity skew correction (when a scale factor and `WithGyroVelocity` are both set) and discretization (when `WithDiscretizationTime` is set). `SwerveDrive` calls this before converting speeds into module states. |

## Static helpers

| Method             | Signature                                                                                               | Description                                                                                                    |
| ------------------ | ------------------------------------------------------------------------------------------------------- | -------------------------------------------------------------------------------------------------------------- |
| `CubeTranslation`  | `static wpi::math::Translation2d CubeTranslation(wpi::math::Translation2d translation)`                 | Cubes the translation's magnitude and keeps its direction. Near-zero input is returned unchanged.              |
| `ScaleTranslation` | `static wpi::math::Translation2d ScaleTranslation(wpi::math::Translation2d translation, double scalar)` | Scales the translation's magnitude by `scalar` and keeps its direction. Near-zero input is returned unchanged. |

***

## Example

```cpp
SwerveDriveConfig config;
config.WithSubsystem(this)
    .WithModules({&m_fl.value(), &m_fr.value(), &m_bl.value(), &m_br.value()})
    .WithGyro([gyroPtr = &m_gyro]() -> wpi::units::degree_t {
      return wpi::units::degree_t{wpi::units::turn_t{gyroPtr->GetYaw().GetValue()}};
    })
    .WithMaximumChassisSpeed(wpi::units::meters_per_second_t{4.5},
                             wpi::units::degrees_per_second_t{360})
    .WithStartingPose(wpi::math::Pose2d{wpi::units::meter_t{0}, wpi::units::meter_t{0}, wpi::math::Rotation2d{}})
    .WithTranslationController(wpi::math::PIDController{1, 0, 0})
    .WithRotationController(wpi::math::PIDController{1, 0, 0})
    .WithTelemetry("swerve",
        yams::telemetry::SwerveDriveTelemetryConfig{SwerveDriveConfig::TelemetryVerbosity::HIGH}
            .WithDataLogName("swerve"));

yams::mechanisms::swerve::SwerveDrive<4> m_drive{&config};
```

***

## See also

* [Java SwerveDriveConfig](../../java-reference/swerve/swerve-drive-config.md)
* [SwerveDrive](swerve-drive.md)
* [SwerveModuleConfig](swerve-module-config.md)
