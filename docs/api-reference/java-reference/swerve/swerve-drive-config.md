# SwerveDriveConfig

**Package (robot code):** `yams.commands2.config` (Commands v2) or `yams.commands3.config` (Commands v3)\
**Extends:** `yams.core.mechanisms.config.SwerveDriveConfig<SwerveDriveConfig>`\
**C++ equivalent:** [C++ SwerveDriveConfig](../../c++-reference/swerve/swerve-drive-config.md)\
**Javadoc:** [`SwerveDriveConfig`](https://javadocs.yassrobotics.com/yams/yams/commands2/config/SwerveDriveConfig.html) (Commands v2), [`SwerveDriveConfig`](https://javadocs.yassrobotics.com/yams/yams/commands3/config/SwerveDriveConfig.html) (Commands v3)

Configures a complete swerve drivetrain. Takes the owning `Subsystem` (Commands v2) or `Mechanism` (Commands v3) and a varargs array of `SwerveModule` objects, one per module. All builder methods live on the shared `yams.core` base class, so they are the same in both frameworks; only the constructor and the subsystem/mechanism binding differ.

## Constructor

{% tabs %}
{% tab title="Commands v2" %}
```java
import yams.commands2.config.SwerveDriveConfig;

SwerveDriveConfig(Subsystem swerveSubsystem, SwerveModule... modules)
```

| Parameter         | Type              | Description                                                                                          |
| ----------------- | ----------------- | ---------------------------------------------------------------------------------------------------- |
| `swerveSubsystem` | `Subsystem`       | The subsystem that owns this drivetrain. Used for command requirements.                              |
| `modules`         | `SwerveModule...` | One module per corner, in the order: front-left, front-right, back-left, back-right (by convention). |

Use the no-arg constructor plus `withSubsystem(Subsystem)` and `withModules(SwerveModule...)` when the modules aren't available yet at construction time. `getSubsystem()` returns the subsystem.
{% endtab %}

{% tab title="Commands v3" %}
```java
import yams.commands3.config.SwerveDriveConfig;

SwerveDriveConfig(Mechanism swerveMechanism, SwerveModule... modules)
```

| Parameter         | Type              | Description                                                                                          |
| ----------------- | ----------------- | ---------------------------------------------------------------------------------------------------- |
| `swerveMechanism` | `Mechanism`       | The mechanism that owns this drivetrain. Used for command requirements.                              |
| `modules`         | `SwerveModule...` | One module per corner, in the order: front-left, front-right, back-left, back-right (by convention). |

Use the no-arg constructor plus `withMechanism(Mechanism)` and `withModules(SwerveModule...)` when the modules aren't available yet at construction time. `getMechanism()` returns the mechanism.
{% endtab %}
{% endtabs %}

All builder methods return `SwerveDriveConfig` for chaining.

***

## Builder methods

| Method                                                                                          | Parameters                                     | Description                                                                                                                                                                                                                                                                                            |
| ----------------------------------------------------------------------------------------------- | ---------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| `withSubsystem(Subsystem subsystem)` (v2) / `withMechanism(Mechanism mechanism)` (v3)           | `subsystem` / `mechanism`                      | Sets the owner (use with the no-arg constructor).                                                                                                                                                                                                                                                      |
| `withModules(SwerveModule... modules)`                                                          | `modules`                                      | Sets the swerve modules for the drive.                                                                                                                                                                                                                                                                 |
| `withRotation(Supplier<Rotation3d> rotation)`                                                   | `rotation`: attitude supplier                  | **Required** (or `withGyro`). The robot's rotation from the gyro, as its attitude, e.g. `pigeon::getRotation3d`. Its yaw is the heading for field-relative control and odometry; its roll and pitch are used by anti-tipping.                                                                          |
| `withGyro(Supplier<Rotation3d> gyro)`                                                           | `gyro`: attitude supplier                      | Same as `withRotation`.                                                                                                                                                                                                                                                                                |
| `withGyroHeadingAxis(GyroAxis axis)`                                                            | `axis`                                         | Which gyro axis points up through the robot. Defaults to `GyroAxis.YAW`. See [Gyro](swerve-drive-config.md#gyro).                                                                                                                                                                                      |
| `withGyroInverted(boolean inverted)`                                                            | `inverted`                                     | Negates the heading when `true`.                                                                                                                                                                                                                                                                       |
| `withGyroVelocity(Supplier<AngularVelocity> supplier)`                                          | `supplier`                                     | Supplies gyro angular velocity. Used for angular-velocity skew correction.                                                                                                                                                                                                                             |
| `withAntiTipping(LinearVelocity kP, Angle tippingThreshold, LinearVelocity maxCorrectionSpeed)` | `kP`, `tippingThreshold`, `maxCorrectionSpeed` | Drives the robot back under itself once its pitch or roll (from the gyro) passes `tippingThreshold`, using WPILib's `AntiTipping`. The correction is added to every robot-relative chassis velocity. Throws `IllegalArgumentException` for non-positive speeds or a threshold outside 0 to 90 degrees. |
| `withAntiTipping(AntiTipping antiTipping)`                                                      | `antiTipping`                                  | Same, with a prebuilt `org.wpilib.math.controller.AntiTipping`.                                                                                                                                                                                                                                        |
| `withStartingPose(Pose2d pose)`                                                                 | `pose`                                         | Initial odometry pose. Defaults to the origin.                                                                                                                                                                                                                                                         |
| `withMaximumChassisSpeed(LinearVelocity speed, AngularVelocity rotSpeed)`                       | `speed`, `rotSpeed`                            | Maximum translational and rotational speeds used for wheel-speed desaturation. Also the starting maximum of every [`SwerveInputStream`](swerve-input-stream.md).                                                                                                                                       |
| `withMaximumModuleSpeed(LinearVelocity speed)`                                                  | `speed`                                        | Maximum module speed used for wheel-speed desaturation.                                                                                                                                                                                                                                                |
| `withModuleStateOptimization(boolean enabled)`                                                  | `enabled`                                      | Overrides every module's `SwerveModuleConfig.withOptimization(...)`. Takes effect immediately on modules already in the config, so it can be toggled on a running drive via `drive.getConfig()`.                                                                                                       |
| `withCenterOfRotation(Translation2d centerOfRotation)`                                          | `centerOfRotation`                             | Overrides the chassis center of rotation (default is the robot center).                                                                                                                                                                                                                                |
| `withCenterOfRotation(Distance forward, Distance left)`                                         | `forward`, `left`                              | Overrides the center of rotation via forward/left distances.                                                                                                                                                                                                                                           |
| `withDiscretizationTime(Time dt)`                                                               | `dt`                                           | Discretization timestep used to compensate for `ChassisVelocities` skew.                                                                                                                                                                                                                               |
| `withSimDiscretizationTime(Time dt)`                                                            | `dt`                                           | Discretization timestep used only in simulation. Falls back to `withDiscretizationTime` if unset.                                                                                                                                                                                                      |
| `withGyroAngularVelocityScaleFactor(double scaleFactor)`                                        | `scaleFactor`                                  | Scale factor `[0, 1]` applied to gyro angular velocity for skew correction on the real robot.                                                                                                                                                                                                          |
| `withSimGyroAngularVelocityScaleFactor(double scaleFactor)`                                     | `scaleFactor`                                  | Scale factor used only in simulation. Falls back to `withGyroAngularVelocityScaleFactor` if unset.                                                                                                                                                                                                     |
| `withTranslationController(PIDController controller)`                                           | `controller`                                   | PID controller for autonomous translation corrections (`driveToPose`).                                                                                                                                                                                                                                 |
| `withRotationController(PIDController controller)`                                              | `controller`                                   | PID controller for autonomous rotation corrections (`driveToPose`, and the heading, aim and translation-only modes of `SwerveInputStream`).                                                                                                                                                            |
| `withSimTranslationController(PIDController controller)`                                        | `controller`                                   | Translation controller used only in simulation. Falls back to `withTranslationController` if unset.                                                                                                                                                                                                    |
| `withSimRotationController(PIDController controller)`                                           | `controller`                                   | Rotation controller used only in simulation. Falls back to `withRotationController` if unset.                                                                                                                                                                                                          |
| `withTelemetry(String name, TelemetryVerbosity verbosity)`                                      | `name`, `verbosity`                            | Sets the telemetry name (NetworkTables/DataLog prefix, defaults to `"swerve"`) and verbosity for drive-level fields (pose, gyro, chassis speeds, module states, auto-align PID gains) in one call.                                                                                                     |
| `withTelemetry(String name, SwerveDriveTelemetryConfig telemetryConfig)`                        | `name`, `telemetryConfig`                      | Sets the telemetry name and configures telemetry with an explicit [`SwerveDriveTelemetryConfig`](swerve-drive.md#telemetry--datalog), taking precedence over the verbosity-based overload.                                                                                                             |

{% hint style="info" %}
Both overloads require a name; there is no name-less `withTelemetry(...)` or standalone `withTelemetryName(...)` on `SwerveDriveConfig` (unlike, say, `SmartMotorControllerConfig`).
{% endhint %}

{% hint style="info" %}
`withRotation` (or `withGyro`, which calls it) is required. On a real robot, constructing a `SwerveDrive` without it throws `IllegalStateException`.
{% endhint %}

***

## Gyro

`withRotation` (or `withGyro`) takes the gyro's full attitude as a `Rotation3d` supplier (most gyros expose one, e.g. `Pigeon2.getRotation3d()`).

`GyroAxis` (`yams.core.mechanisms.config.enums.GyroAxis`) says which axis of the gyro points up through the robot, so a gyro mounted on its side still reads the robot's heading:

| Value   | Meaning                                                                                                                                |
| ------- | -------------------------------------------------------------------------------------------------------------------------------------- |
| `YAW`   | The gyro's Z axis points up (mounted flat). The default.                                                                               |
| `ROLL`  | The gyro's X axis points up: the robot turning shows up as the gyro's roll. The gyro's Y axis is taken to point to the robot's left.   |
| `PITCH` | The gyro's Y axis points up: the robot turning shows up as the gyro's pitch. The gyro's X axis is taken to point to the robot's front. |

`GyroAxis.toRobotAttitude(Rotation3d gyroAttitude)` rotates the gyro's attitude into the robot frame; `SwerveDriveConfig` calls it for you before reading the heading and the tilt. `withGyroInverted` applies to the resulting heading. There is no gyro offset: the pose estimator keeps the offset between the gyro and the pose, so set the robot's heading with the starting pose, `SwerveDrive.resetOdometry(Pose2d)` or `zeroOdometryHeading()`.

| Getter                | Returns      | Description                                                                                                                                                                                                                                                                           |
| --------------------- | ------------ | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `getGyroRotation3d()` | `Rotation3d` | The robot's attitude: the gyro attitude rotated into the robot frame for the configured `GyroAxis`, with the inversion applied to its yaw. The yaw is the heading, wrapped to +/-180 degrees; roll and pitch are the robot's tilt. Throws `IllegalStateException` if no gyro was set. |

{% hint style="warning" %}
`getGyroAngle()` was removed. Use `getGyroRotation3d()` (on the config or on [`SwerveDrive`](swerve-drive.md#gyro), which also handles simulation), e.g. `getGyroRotation3d().toRotation2d()` for a `Rotation2d` heading. `withGyro` now takes a `Supplier<Rotation3d>` instead of a `Supplier<Angle>`.
{% endhint %}

```java
// Pigeon 2 mounted on its side, so the robot turning is the gyro's pitch.
SwerveDriveConfig config = new SwerveDriveConfig(this, fl, fr, bl, br)
    .withGyro(pigeon::getRotation3d)
    .withGyroHeadingAxis(GyroAxis.PITCH);
```

***

## DataLog telemetry

`SwerveDriveConfig` itself has no `withDataLogName(...)` method: DataLog output is configured on a `SwerveDriveTelemetryConfig` (`yams.core.telemetry`) and wired in via `withTelemetry(String name, SwerveDriveTelemetryConfig)`:

```java
SwerveDriveConfig config = new SwerveDriveConfig(this, frontLeft, frontRight, backLeft, backRight)
    .withGyro(m_gyro::getRotation3d)
    .withTelemetry("swerve", new SwerveDriveTelemetryConfig(TelemetryVerbosity.HIGH)
        .withDataLogName("swerve"));  // logs pose, gyro, chassis speeds, module states
```

`SwerveDriveTelemetryConfig(TelemetryVerbosity)` is shorthand for `new SwerveDriveTelemetryConfig().withTelemetryVerbosity(verbosity)`.

{% hint style="warning" %}
`withTelemetry(name, SwerveDriveTelemetryConfig)` replaces the default field selection entirely; if you pass a config, call `withTelemetryVerbosity(...)` (or the individual `with*()` field methods) on it yourself, since a bare `new SwerveDriveTelemetryConfig()` has every field disabled. This DataLog name does **not** cascade to the modules passed to the constructor, or to those modules' drive/azimuth motors. Each `SwerveModuleConfig` needs its own telemetry setup (see [SwerveModuleConfig](swerve-module-config.md#datalog-telemetry)), and each motor's `SmartMotorControllerConfig` needs its own `withTelemetry(name, SmartMotorControllerTelemetryConfig)` for field-level control and its own DataLog name.
{% endhint %}

***

## Example

{% tabs %}
{% tab title="Commands v2" %}
```java
import yams.commands2.config.SwerveDriveConfig;
import yams.commands2.swerve.SwerveDrive;

// Inside a SubsystemBase constructor.
SwerveDriveConfig config = new SwerveDriveConfig(this, frontLeft, frontRight, backLeft, backRight)
    .withGyro(m_gyro::getRotation3d)
    .withMaximumChassisSpeed(MetersPerSecond.of(4.5), RadiansPerSecond.of(Math.PI * 2))
    .withTranslationController(new PIDController(1, 0, 0))
    .withRotationController(new PIDController(1, 0, 0))
    .withTelemetry("swerve", new SwerveDriveTelemetryConfig(TelemetryVerbosity.HIGH)
        .withDataLogName("swerve"));
SwerveDrive drive = new SwerveDrive(config);
```
{% endtab %}

{% tab title="Commands v3" %}
```java
import yams.commands3.config.SwerveDriveConfig;
import yams.commands3.swerve.SwerveDrive;

// Inside a Mechanism constructor.
SwerveDriveConfig config = new SwerveDriveConfig(this, frontLeft, frontRight, backLeft, backRight)
    .withGyro(m_gyro::getRotation3d)
    .withMaximumChassisSpeed(MetersPerSecond.of(4.5), RadiansPerSecond.of(Math.PI * 2))
    .withTranslationController(new PIDController(1, 0, 0))
    .withRotationController(new PIDController(1, 0, 0))
    .withTelemetry("swerve", new SwerveDriveTelemetryConfig(TelemetryVerbosity.HIGH)
        .withDataLogName("swerve"));
SwerveDrive drive = new SwerveDrive(config);
```
{% endtab %}
{% endtabs %}

***

## See also

* [C++ SwerveDriveConfig](../../c++-reference/swerve/swerve-drive-config.md)
* [SwerveDrive](swerve-drive.md)
* [SwerveModuleConfig](swerve-module-config.md)
