---
icon: database
---

# DataLog best practices

{% hint style="info" %}
Using DataLog will increase your storage usage on the SystemCore and could have negative consequences if left unmanaged. Review [WPILib's DataLog documentation](https://docs.wpilib.org/en/stable/docs/software/telemetry/datalog.html#custom-data-logging-using-datalog) for more information.
{% endhint %}

Every YAMS telemetry field is published to NetworkTables by default. Separately, you can opt any field into a WPILib `DataLog`, the same file format read by AdvantageScope offline, and the one WPILib's `DataLogManager` automatically saves to a USB drive or the SystemCore's local storage during a match. This page covers how to control both destinations independently, and calls out the one place, `SwerveDrive`, where the rules are different.

## The two knobs: NetworkTables and DataLog

For any single-motor mechanism (`Arm`, `Elevator`, `Pivot`, `FlyWheel`, `DifferentialMechanism`, `DoubleJointedArm`) telemetry is controlled per motor controller with `SmartMotorControllerTelemetryConfig`:

```java
SmartMotorControllerTelemetryConfig telemetryConfig = new SmartMotorControllerTelemetryConfig()
    .withTelemetryVerbosity(TelemetryVerbosity.LOW)
    .withStatorCurrent()
    .withTemperature()
    .withDataLogName("motors/shooter")   // log the fields above to DataLog...
    .withoutNetworkTables();             // ...and don't publish them to NT4

SmartMotorControllerConfig motorConfig = new SmartMotorControllerConfig(this)
    .withClosedLoopController(4, 0, 0)
    .withGearing(new MechanismGearing(GearBox.fromReductionStages(3, 4)))
    .withTelemetry("ShooterMotor", telemetryConfig);
```

* `withDataLogName(name)` turns on DataLog for every field enabled on that config, under the given prefix.
* `withoutNetworkTables()` (or `withNetworkTables(false)`) stops those fields from being published to NT4 at all. You can freely enable DataLog, NetworkTables, both, or neither, they're independent toggles.
* Which fields get logged is controlled the same way for both destinations, there's only one set of enabled fields, per `withTelemetryVerbosity()` and the individual `with*()` methods described in [Telemetry](telemetry.md).

{% hint style="success" %}
Set a DataLogName for competition matches. It costs almost nothing to add and gives you a full record to review between matches, often the difference between guessing why a mechanism stalled and knowing exactly why.
{% endhint %}

## Choosing what to log

Every field is disabled by default. `withTelemetryVerbosity()` gives you a cumulative preset (`LOW` → `MID` → `HIGH`, each including everything below it) so you don't have to enable dozens of fields by hand, then layer on or remove individual fields as needed.

A few guidelines:

* **Don't just reach for `HIGH` everywhere.** `HIGH` includes PID gains, feedforward gains, current limits, and every boolean status flag, useful while tuning, but mostly redundant once a mechanism is dialed in. Prefer `LOW` or `MID` plus the handful of fields you actually watch (current, temperature, setpoint and position) for a competition build.
* **Prune before you log, not after.** DataLog files only grow, there's no way to selectively remove a field after the fact without re-recording. Decide what you need before the match, not while reviewing the log afterward.
* **DataLog is not a substitute for NetworkTables during development.** Live tools (Elastic, Shuffleboard, AdvantageScope in live mode) read NT4, not DataLog. Keep NetworkTables on while you're actively tuning, and consider disabling it only for fields you truly don't need live, once you're locking in a competition build.

Need to flip a field that doesn't have its own `with*()` method, or toggle several fields at once? `withCustom(field, value)` takes any `BooleanTelemetryField`/`DoubleTelemetryField` (or an array of either) plus a boolean, and enables or disables them accordingly:

```java
SmartMotorControllerTelemetryConfig telemetryConfig = new SmartMotorControllerTelemetryConfig()
    .withTelemetryVerbosity(TelemetryVerbosity.LOW)
    .withCustom(new SmartMotorControllerTelemetry.DoubleTelemetryField[]{
        SmartMotorControllerTelemetry.DoubleTelemetryField.SupplyCurrent,
        SmartMotorControllerTelemetry.DoubleTelemetryField.StatorCurrentLimit
    }, true);
```

## Enabling/disabling NetworkTables conditionally

`withNetworkTables(boolean)` takes a plain boolean, read once when the config is built, so drive it from a constant you set before you deploy:

```java
// Constants.java: set to true before you deploy for matches.
public static final boolean COMPETITION = false;

SmartMotorControllerTelemetryConfig telemetryConfig = new SmartMotorControllerTelemetryConfig()
    .withTelemetryVerbosity(TelemetryVerbosity.LOW)
    .withDataLogName("motors/shooter")
    .withNetworkTables(!Constants.COMPETITION);
```

This keeps full NT4 telemetry for pit testing and practice, and drops NT4 traffic (while still writing to DataLog) in the code you deploy for matches.

{% hint style="warning" %}
`RobotState.isFMSAttached()` does not work here. The config is built when the robot code starts, before the field connects, so it reads `false` and NetworkTables stays on in every match.
{% endhint %}

## Swerve drive is different

{% hint style="warning" %}
`SwerveDriveConfig`/`SwerveModuleConfig` do **not** use `SmartMotorControllerTelemetryConfig`, they have their own `SwerveDriveTelemetryConfig`/`SwerveModuleTelemetryConfig`. The shape is the same (verbosity preset plus individual `with*()` fields, `withoutNetworkTables()`, `withDataLogName(...)`), but the DataLog name for both the drive-level fields (pose, gyro, chassis speeds, module states) and a module's absolute encoder angle has to go through the telemetry config, neither `SwerveDriveConfig` nor `SwerveModuleConfig` has a `withDataLogName(...)` of its own.
{% endhint %}

`SwerveDriveConfig.withTelemetry(name, TelemetryVerbosity)` and `SwerveModuleConfig.withTelemetry(name, TelemetryVerbosity)` pick a verbosity preset the same way a single-motor mechanism does, both always take the telemetry name as the first argument, there's no name-less overload. For anything more, hand `withTelemetry(name, ...)` a `SwerveDriveTelemetryConfig`/`SwerveModuleTelemetryConfig` instead of a bare `TelemetryVerbosity`, that's what lets you enable individual fields, disable NetworkTables, or set a DataLog name for the drive-level and per-module fields (pose, gyro, chassis speeds, module states, absolute encoder angle).

You can still send this to DataLog:

```java
SwerveDriveConfig driveConfig = new SwerveDriveConfig(this, frontLeft, frontRight, backLeft, backRight)
    .withGyro(gyro::getRotation3d)
    .withTelemetry("swerve", new SwerveDriveTelemetryConfig(TelemetryVerbosity.HIGH)
        .withDataLogName("swerve"));                 // logs pose, gyro, chassis speeds, module states

SwerveModuleConfig frontLeftConfig = new SwerveModuleConfig(driveMotorFL, azimuthMotorFL)
    .withWheelRadius(Meters.of(0.0508))
    .withLocation(new Translation2d(0.381, 0.381))
    .withTelemetry("FrontLeft", new SwerveModuleTelemetryConfig(TelemetryVerbosity.HIGH)
        .withDataLogName("swerve/frontLeft"));       // logs this module's absolute encoder angle (+ state at HIGH)
```

`new SwerveDriveTelemetryConfig(TelemetryVerbosity.HIGH)`/`new SwerveModuleTelemetryConfig(TelemetryVerbosity.HIGH)` are shorthand constructors for `new SwerveDriveTelemetryConfig().withTelemetryVerbosity(TelemetryVerbosity.HIGH)` and the module equivalent. Each module's DataLog name is independent of the drive's `SwerveDriveTelemetryConfig.withDataLogName(...)` above, set it separately per module if you want per-module DataLog prefixes.

{% hint style="info" %}
Neither the drive's nor a module's DataLog name cascades to the drive/azimuth motors that make up that module. Setting one does not log the other, and neither logs the motors.
{% endhint %}

### Getting granular fields out of a swerve module's motors

The drive and azimuth motors of a `SwerveModule` are still ordinary `SmartMotorController`s under the hood. If you want field-level control (or a DataLog name) on just the drive motor or just the azimuth motor, set it on that motor's own `SmartMotorControllerConfig`, before you build the `SwerveModule`, exactly as you would for a standalone motor:

```java
SmartMotorControllerTelemetryConfig driveTelemetry = new SmartMotorControllerTelemetryConfig()
    .withStatorCurrent()
    .withTemperature()
    .withDataLogName("swerve/frontLeft/drive");

SmartMotorControllerConfig driveMotorConfig = new SmartMotorControllerConfig(this)
    .withClosedLoopController(0.1, 0.0, 0.0)
    .withGearing(new MechanismGearing(GearBox.fromReductionStages(6.75)))
    .withTelemetry("driveMotor", driveTelemetry);

SmartMotorController driveMotorFL = new TalonFXWrapper(new TalonFX(10, new CANBus(CANPort.CAN_S0)), DCMotor.getKrakenX60(1), driveMotorConfig);
```

This gives you the same fine-grained control (and independent DataLog name) on that one motor that you'd get on any other mechanism's motor, it's only the swerve-level pose/gyro/chassis-speed/encoder fields that are all-or-nothing per verbosity level.

## Related pages

* [Telemetry](telemetry.md)
* [Swerve Drive](../tutorials/swerve-drive/)
* [How do I view my DataLog in AdvantageScope?](../how-to-simulation-and-logging/how-do-i-view-my-datalog-in-advantagescope.md)
