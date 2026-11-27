# SwerveModuleConfig

**Package:** `yams.core.mechanisms.config`\
**C++ equivalent:** [C++ SwerveModuleConfig](../../c++-reference/swerve/swerve-module-config.md)\
**Javadoc:** [`SwerveModuleConfig`](https://javadocs.yassrobotics.com/yams/yams/core/mechanisms/config/SwerveModuleConfig.html)

Configures one swerve module. Takes drive and azimuth motor controllers in the constructor. It does not need a `Subsystem` or `Mechanism`, so the same class is used with Commands v2 and Commands v3.

## Constructor

```java
SwerveModuleConfig(SmartMotorController driveMotor, SmartMotorController azimuthMotor)
```

| Parameter      | Type                   | Description                                  |
| -------------- | ---------------------- | -------------------------------------------- |
| `driveMotor`   | `SmartMotorController` | Motor controller for the drive wheel.        |
| `azimuthMotor` | `SmartMotorController` | Motor controller for the steering mechanism. |

Use the no-arg constructor plus `withSmartMotorController(SmartMotorController, SmartMotorController)` when the motors aren't available yet at construction time.

All builder methods return `SwerveModuleConfig` for chaining.

***

## Builder methods

| Method                                                                                         | Parameters                              | Description                                                                                                                                                                                                                                                                                                        |
| ---------------------------------------------------------------------------------------------- | --------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| `withSmartMotorController(SmartMotorController driveMotor, SmartMotorController azimuthMotor)` | `driveMotor`, `azimuthMotor`            | Sets both motor controllers (use with the no-arg constructor).                                                                                                                                                                                                                                                     |
| `withCosineCompensation(boolean compensate)`                                                   | `compensate`                            | Scales drive speed by `cos(angle_error)` to reduce skew while steering.                                                                                                                                                                                                                                            |
| `withCouplingRatio(GearBox couplingRatio)`                                                     | `couplingRatio`                         | Steer-to-drive coupling, compensated as in the CTRE swerve API: drive motor (input) to azimuth (output), e.g. `GearBox.fromReductionStages(kCoupleRatio)` with `kCoupleRatio` from CTRE Tuner X. The motion steering causes is added to the drive setpoint and removed from the measured wheel speed and distance. |
| `withCouplingRatio(double driveMotorRotationsPerAzimuthRotation)`                              | `driveMotorRotationsPerAzimuthRotation` | Same, as a single reduction. `0` disables coupling compensation.                                                                                                                                                                                                                                                   |
| `withAbsoluteEncoder(Object absoluteEncoder)`                                                  | `absoluteEncoder`                       | Same-vendor absolute encoder object for the azimuth `SmartMotorController`.                                                                                                                                                                                                                                        |
| `withAbsoluteEncoder(Supplier<Angle> locationProvider)`                                        | `locationProvider`                      | Cross-vendor absolute encoder angle supplier.                                                                                                                                                                                                                                                                      |
| `withAbsoluteEncoder(DoubleSupplier rotationSupplier)`                                         | `rotationSupplier`                      | Cross-vendor absolute encoder angle supplier, in rotations.                                                                                                                                                                                                                                                        |
| `withAbsoluteEncoderGearing(GearBox gearing)`                                                  | `gearing`                               | Gearing between the absolute encoder shaft and the azimuth mechanism, if not 1:1.                                                                                                                                                                                                                                  |
| `withAbsoluteEncoderOffset(Angle offset)`                                                      | `offset`                                | Absolute encoder reading when the module is at its forward (zero) position, bevel left.                                                                                                                                                                                                                            |
| `withLocation(Translation2d location)`                                                         | `location`: module position on chassis  | Module position relative to robot center, in meters. Used in kinematics calculations.                                                                                                                                                                                                                              |
| `withLocation(Distance front, Distance left)`                                                  | `front`, `left`                         | Module position via separate forward/left distances.                                                                                                                                                                                                                                                               |
| `withLocation(Distance distance, Angle angle)`                                                 | `distance`, `angle`                     | Module position via polar coordinates from the center of rotation.                                                                                                                                                                                                                                                 |
| `withDistanceFromCenterOfRotation(Distance front, Distance left)`                              | `front`, `left`                         | Equivalent to `withLocation(Distance, Distance)`.                                                                                                                                                                                                                                                                  |
| `withWheelRadius(Distance radius)`                                                             | `radius`: wheel radius                  | Drive wheel radius used for velocity and position conversion.                                                                                                                                                                                                                                                      |
| `withWheelDiameter(Distance diameter)`                                                         | `diameter`                              | Drive wheel diameter used for velocity and position conversion.                                                                                                                                                                                                                                                    |
| `withVelocityDeadband(LinearVelocity speed)`                                                   | `speed`                                 | Below this speed, the module holds its current azimuth instead of tracking.                                                                                                                                                                                                                                        |
| `withOptimization(boolean swerveModuleStateOptimization)`                                      | `swerveModuleStateOptimization`         | Enables/disables state optimization (turn the wheel around and reverse the drive when that is the shorter way to the requested angle). `SwerveDriveConfig.withModuleStateOptimization(boolean)` overrides it for every module.                                                                                     |
| `withTelemetry(String telemetryName, TelemetryVerbosity telemetryVerbosity)`                   | `telemetryName`, `telemetryVerbosity`   | NetworkTables key prefix and verbosity for this module's own fields (absolute encoder angle, `SwerveModuleVelocity`).                                                                                                                                                                                              |
| `withTelemetry(String telemetryName, SwerveModuleTelemetryConfig telemetryConfig)`             | `telemetryName`, `telemetryConfig`      | NetworkTables key prefix plus an explicit [`SwerveModuleTelemetryConfig`](swerve-module.md), taking precedence over `withTelemetry(String, TelemetryVerbosity)`.                                                                                                                                                   |

{% hint style="info" %}
`withAbsoluteEncoderOffset` must be calibrated per module. An incorrect offset causes the module to drive at an angle on startup until `seedAzimuthEncoder()` corrects it.
{% endhint %}

***

## State helpers

`SwerveModule` uses these to turn a requested `SwerveModuleVelocity` into what it sends to the motors. You rarely call them yourself.

| Method                                                                                    | Returns                | Description                                                                                                                                                                                                                                                                                                                                                                                                               |
| ----------------------------------------------------------------------------------------- | ---------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `getOptimizedState(SwerveModuleVelocity state)`                                           | `SwerveModuleVelocity` | Applies the velocity deadband, optimization and cosine compensation, measuring the wheel with the azimuth motor.                                                                                                                                                                                                                                                                                                          |
| `getOptimizedState(SwerveModuleVelocity state, SwerveModuleVelocityWithAzimuth measured)` | `SwerveModuleVelocity` | Same, against a given measured state.                                                                                                                                                                                                                                                                                                                                                                                     |
| `getLockedState(Rotation2d angle, SwerveModuleVelocityWithAzimuth measured)`              | `SwerveModuleVelocity` | Zero-speed state that turns the wheel to `angle` without driving it, as for locking the drive in place. Unlike `getOptimizedState`, the velocity deadband does not hold the wheel where it is. With optimization enabled the wheel turns to whichever of `angle` and its opposite is nearer. Used by `SwerveModule.setAzimuthAngle(Rotation2d)`, `SwerveDrive.lockPose()` and `SwerveDrive.setAzimuthAngles(Rotation2d)`. |
| `getCosineCompensationScalar(Rotation2d desiredAngle)`                                    | `double`               | Cosine compensation factor from 0 to 1 against the current azimuth angle, or 1 if cosine compensation is disabled.                                                                                                                                                                                                                                                                                                        |

***

## DataLog telemetry

`SwerveModuleConfig` has no `withDataLogName(String)` method: DataLog output is configured on a [`SwerveModuleTelemetryConfig`](swerve-module.md) and wired in via `withTelemetry(String, SwerveModuleTelemetryConfig)`:

```java
SwerveModuleConfig frontLeftConfig = new SwerveModuleConfig(driveMotor, azimuthMotor)
    .withWheelRadius(Inches.of(2))
    .withLocation(new Translation2d(0.298, 0.298))
    .withAbsoluteEncoderOffset(Degrees.of(45.2))
    .withTelemetry("FrontLeft", new SwerveModuleTelemetryConfig(TelemetryVerbosity.HIGH)
        .withDataLogName("swerve/frontLeft"));  // logs this module's absolute encoder angle and state
```

{% hint style="warning" %}
This DataLog name is independent of the drive-level one configured via `SwerveDriveConfig.withTelemetry(String name, SwerveDriveTelemetryConfig)` (neither cascades to the other) and does **not** cascade to this module's drive/azimuth motors. To get field-level DataLog control (or a DataLog name) on the drive or azimuth motor itself, configure that motor's own `SmartMotorControllerConfig.withTelemetry(name, SmartMotorControllerTelemetryConfig)` before constructing the `SwerveModule`; see [SmartMotorControllerTelemetryConfig](../motor-controllers/smart-motor-controller-telemetry-config.md).
{% endhint %}

***

## Example

```java
SwerveModuleConfig frontLeft = new SwerveModuleConfig(driveMotor, azimuthMotor)
    .withWheelRadius(Inches.of(2))
    .withLocation(new Translation2d(0.298, 0.298))
    .withAbsoluteEncoderOffset(Degrees.of(45.2))
    .withTelemetry("FrontLeft", new SwerveModuleTelemetryConfig(TelemetryVerbosity.HIGH)
        .withDataLogName("swerve/frontLeft"));
```

***

## See also

* [C++ SwerveModuleConfig](../../c++-reference/swerve/swerve-module-config.md)
* [SwerveModule](swerve-module.md)
* [SwerveDriveConfig](swerve-drive-config.md)
