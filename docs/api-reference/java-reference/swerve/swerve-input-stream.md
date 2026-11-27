# SwerveInputStream

**Package:** `yams.commands2.swerve` (Commands v2) or `yams.commands3.swerve` (Commands v3)\
**Javadoc:** [`SwerveInputStream`](https://javadocs.yassrobotics.com/yams/yams/commands2/swerve/SwerveInputStream.html) (Commands v2), [`SwerveInputStream`](https://javadocs.yassrobotics.com/yams/yams/commands3/swerve/SwerveInputStream.html) (Commands v3)

Converts raw joystick axis values into field-relative `ChassisVelocities` for swerve drive control. Handles deadbands, cubic input curves, axis scaling, alliance-relative flipping, and multiple drive modes (angular-velocity, heading-snap, translation-only, aim-at-target). Implements `Supplier<ChassisVelocities>` so it can be passed directly wherever a `ChassisVelocities` supplier is expected.

The API is the same in both packages; they differ only in the type of the `Live Tuning` command published by [telemetry](swerve-input-stream.md#telemetry-and-live-tuning). Import the one matching your command framework.

***

## Creating a stream

```java
// Translation only; add rotation later with withControllerRotationAxis() or withControllerHeadingAxis()
SwerveInputStream.of(SwerveDrive drive, DoubleSupplier x, DoubleSupplier y)

// With a rotation axis (angular-velocity mode)
SwerveInputStream.of(SwerveDrive drive, DoubleSupplier x, DoubleSupplier y, DoubleSupplier rot)
new SwerveInputStream(SwerveDrive drive, DoubleSupplier x, DoubleSupplier y, DoubleSupplier rot)

// With heading axes (heading-angle mode, used while withHeadingControl() is true)
new SwerveInputStream(SwerveDrive drive, DoubleSupplier x, DoubleSupplier y,
                      DoubleSupplier headingX, DoubleSupplier headingY)
```

| Parameter  | Type             | Description                                                                                                                                                   |
| ---------- | ---------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `drive`    | `SwerveDrive`    | The drivetrain instance (`yams.core.mechanisms.swerve.SwerveDrive`, so a v2 or v3 `SwerveDrive` works). Used for heading reference, rotation PID, and config. |
| `x`        | `DoubleSupplier` | Forward/back axis in range `[-1, 1]`. Positive is forward.                                                                                                    |
| `y`        | `DoubleSupplier` | Left/right axis in range `[-1, 1]`. Positive is left.                                                                                                         |
| `rot`      | `DoubleSupplier` | Rotation axis in range `[-1, 1]`, counter-clockwise positive. Used for angular-velocity control.                                                              |
| `headingX` | `DoubleSupplier` | Heading X component in range `[-1, 1]`. Used for heading-angle control.                                                                                       |
| `headingY` | `DoubleSupplier` | Heading Y component in range `[-1, 1]`. Used for heading-angle control.                                                                                       |

***

## Builder methods

All builder methods return `SwerveInputStream` for chaining.

### Velocity limits

| Method                                                 | Parameters | Description                                                                                                                    |
| ------------------------------------------------------ | ---------- | ------------------------------------------------------------------------------------------------------------------------------ |
| `withMaximumLinearVelocity(LinearVelocity velocity)`   | `velocity` | Chassis speed a full translation axis maps to. Defaults to the drive's maximum chassis linear velocity, or 4 m/s.              |
| `withMaximumAngularVelocity(AngularVelocity velocity)` | `velocity` | Chassis rotation rate a full rotation axis maps to. Defaults to the drive's maximum chassis angular velocity, or 1 rotation/s. |

{% hint style="info" %}
A stream copies `SwerveDriveConfig.withMaximumChassisSpeed(...)`'s value (if set) into its own maximum fields once, at construction time; otherwise it starts from its own defaults (4 m/s, 1 rotation/s). Calling `withMaximumLinearVelocity(...)`/`withMaximumAngularVelocity(...)` afterward overrides that value for this stream and takes effect immediately; the drive's config is not re-checked after construction.
{% endhint %}

### Axis configuration

| Method                                                                        | Parameters                              | Description                                                                                                                                                                  |
| ----------------------------------------------------------------------------- | --------------------------------------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `withControllerRotationAxis(DoubleSupplier rot)`                              | `rot`: range `[-1, 1]`                  | Set or replace the rotation axis supplier after construction. Enables angular-velocity mode.                                                                                 |
| `withControllerHeadingAxis(DoubleSupplier headingX, DoubleSupplier headingY)` | `headingX`, `headingY`: range `[-1, 1]` | Face the direction the stick points while `withHeadingControl()` is active. The robot stops turning while the stick is inside the deadband. Replaces any `withHeading(...)`. |
| `withHeading(Supplier<Angle> heading)`                                        | `heading`                               | Field-relative heading to face while `withHeadingControl()` is active. Replaces any controller heading axis.                                                                 |
| `withDeadband(double deadband)` / `deadband(double deadband)`                 | `deadband`: 0.0 to 1.0                  | Deadband applied to all axes before scaling. `0` disables it.                                                                                                                |
| `withScaleTranslation(double scale)`                                          | `scale`: range `(0, 1]`                 | Multiplies the translation output magnitude by `scale`. `0` disables scaling.                                                                                                |
| `withScaleRotation(double scale)`                                             | `scale`: range `(0, 1]`                 | Multiplies the rotation axis value by `scale`. `0` disables scaling.                                                                                                         |

### Input curves

| Method                                                       | Parameters | Description                                                                                 |
| ------------------------------------------------------------ | ---------- | ------------------------------------------------------------------------------------------- |
| `withCubeTranslationControllerAxis()`                        | N/A        | Applies a cubic curve to translation magnitude for finer low-speed control. Always enabled. |
| `withCubeTranslationControllerAxis(BooleanSupplier enabled)` | `enabled`  | Applies a cubic curve to translation magnitude when supplier returns `true`.                |
| `withCubeRotationControllerAxis()`                           | N/A        | Applies a cubic curve to the rotation axis. Always enabled.                                 |
| `withCubeRotationControllerAxis(BooleanSupplier enabled)`    | `enabled`  | Applies a cubic curve to the rotation axis when supplier returns `true`.                    |

### Drive modes

Heading, aim and translation-only control need a rotation controller on the drive (`SwerveDriveConfig.withRotationController(...)`); without one, `get()` throws `SwerveDriveConfigurationException` in those modes.

| Method                                                             | Parameters          | Description                                                                                                                 |
| ------------------------------------------------------------------ | ------------------- | --------------------------------------------------------------------------------------------------------------------------- |
| `withHeadingControl(BooleanSupplier trigger)`                      | `trigger`           | Enables heading-angle control mode while trigger returns `true`. Requires `withControllerHeadingAxis()` or `withHeading()`. |
| `withAim(Supplier<Pose2d> target, BooleanSupplier trigger)`        | `target`, `trigger` | Rotates the robot to face the target pose while translating. Enables AIM mode while trigger is `true`.                      |
| `withAimLookahead(Time lookahead)`                                 | `lookahead`         | Aims from where the robot will be after the lookahead time, moving the pose along its field relative velocity.              |
| `withAimFeedforward(SimpleMotorFeedforward feedforward)`           | `feedforward`       | Adds a feedforward to the aim rotation so the robot keeps up with a target whose direction is changing.                     |
| `withAimHeadingOffset(Rotation2d offset)`                          | `offset`            | Rotates the aim heading, for a shooter that does not point out of the front of the robot.                                   |
| `withAimHeadingOffset(Rotation2d offset, BooleanSupplier enabled)` | `offset`, `enabled` | Rotates the aim heading while `enabled` is `true`.                                                                          |
| `aimLock(Angle tolerance)`                                         | `tolerance`         | Returns a `Trigger` that is true while aiming and within the tolerance of the aim heading.                                  |
| `isAimLocked(Angle tolerance)`                                     | `tolerance`         | Whether the robot is aiming and within the tolerance of the aim heading.                                                    |
| `getTargetVector(Pose2d target)`                                   | `target`            | Field relative vector from the robot to the target, from the looked ahead position if a lookahead is set.                   |
| `calculateAngularVelocity(Angle target)`                           | `target`            | Angular velocity that turns the robot to a heading, from the rotation controller and the aim feedforward.                   |
| `withTranslationOnly(BooleanSupplier trigger)`                     | `trigger`           | Suppresses rotation and holds the heading the robot had when enabled while trigger returns `true`.                          |

### Field orientation

| Method                                                                    | Parameters         | Description                                                                                                                          |
| ------------------------------------------------------------------------- | ------------------ | ------------------------------------------------------------------------------------------------------------------------------------ |
| `withAllianceRelativeControl()`                                           | N/A                | Flips translation 180 degrees on the Red alliance so "forward" always points away from your driver station. Always enabled.          |
| `withAllianceRelativeControl(BooleanSupplier enabled)`                    | `enabled`          | Alliance-relative control when supplier returns `true`.                                                                              |
| `withRobotRelative()`                                                     | N/A                | Treats the translation axes as robot relative. The output is still field relative, and alliance flipping is skipped. Always enabled. |
| `withRobotRelative(BooleanSupplier enabled)`                              | `enabled`          | Robot-relative translation axes when supplier returns `true`.                                                                        |
| `withTranslationHeadingOffset(Rotation2d angle)`                          | `angle`            | Rotates the translation direction by `angle`. Always enabled.                                                                        |
| `withTranslationHeadingOffset(Rotation2d angle, BooleanSupplier enabled)` | `angle`, `enabled` | Rotates the translation direction by `angle` when supplier returns `true`.                                                           |

### Utility

| Method    | Parameters | Description                                                                                                          |
| --------- | ---------- | -------------------------------------------------------------------------------------------------------------------- |
| `clone()` | N/A        | Returns a copy of this stream. Subsequent changes to the copy do not affect the original. The copy has no telemetry. |

***

## Output methods

| Method                               | Returns             | Description                                                                                                                                                                                                                                                                                                |
| ------------------------------------ | ------------------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `get()`                              | `ChassisVelocities` | Computes and returns the current **field-relative** `ChassisVelocities` based on all configured inputs and the active drive mode. Call once per robot loop, and pass the result to `SwerveDrive.setFieldRelativeChassisSpeeds(...)` (or convert with `toRobotRelative(...)` for `SwerveDrive.drive(...)`). |
| `getCurrentModeName()`               | `String`            | Name of the drive mode used by the last `get()` call.                                                                                                                                                                                                                                                      |
| `getRotationControllerMeasurement()` | `Optional<Angle>`   | The measurement the last `get()` gave the rotation controller in `HEADING`, `AIM` or `TRANSLATION_ONLY`: the robot's heading, on the field, from the drive's odometry rotation. Empty if the last `get()` did not use the controller.                                                                      |
| `getRotationControllerSetpoint()`    | `Optional<Angle>`   | The setpoint the last `get()` gave the rotation controller: the heading being turned to, on the field, from the drive's odometry rotation. Empty if the last `get()` did not use the controller.                                                                                                           |

The current configuration can also be read and changed at runtime (this is what live tuning uses): `getAxisDeadband()`/`setAxisDeadband(double)`, `getTranslationAxisScale()`/`setTranslationAxisScale(double)`, `getOmegaAxisScale()`/`setOmegaAxisScale(double)`, `getMaximumChassisLinearVelocity()`/`setMaximumChassisLinearVelocity(LinearVelocity)`, `getMaximumChassisAngularVelocity()`/`setMaximumChassisAngularVelocity(AngularVelocity)`, `isTranslationCubeEnabled()`/`setTranslationCubeEnabled(boolean)`, `isOmegaCubeEnabled()`/`setOmegaCubeEnabled(boolean)`, `isAllianceRelativeEnabled()`/`setAllianceRelativeEnabled(boolean)`, and `isRobotRelativeEnabled()`/`setRobotRelativeEnabled(boolean)`.

***

## Drive modes

`SwerveInputStream` automatically selects the active drive mode each loop based on which suppliers are configured and active.

| Mode               | Description                                                                                                                                         |
| ------------------ | --------------------------------------------------------------------------------------------------------------------------------------------------- |
| `ANGULAR_VELOCITY` | Default. Rotates using the angular-velocity axis (`withControllerRotationAxis`).                                                                    |
| `HEADING`          | Turns to a heading from `withControllerHeadingAxis()` or `withHeading()`. Active when the `withHeadingControl()` trigger is `true`.                 |
| `AIM`              | Rotates to face a target `Pose2d`. Active when `withAim()` trigger is `true` and a target is set.                                                   |
| `TRANSLATION_ONLY` | Suppresses rotation and holds the current heading. Active when `withTranslationOnly()` trigger is `true`, or when no rotation source is configured. |

Mode priority (highest to lowest): `TRANSLATION_ONLY` > `AIM` > `HEADING` > `ANGULAR_VELOCITY`.

{% hint style="info" %}
An earlier `DRIVE_TO_POSE` mode (with a matching `atTargetPose(...)` method) has been removed from `SwerveInputStream`. Use [`SwerveDrive.driveToPose(...)`](swerve-drive.md#commands) for auto-align instead.
{% endhint %}

***

## Telemetry and live tuning

Telemetry is off until you call `withTelemetry(name, verbosity)`. It publishes every time the stream is read with `get()`, so there is nothing to call each loop.

| Method                                                     | Returns                                | Description                                                                                                                                                                                                               |
| ---------------------------------------------------------- | -------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `withTelemetry(String name, TelemetryVerbosity verbosity)` | `SwerveInputStream`                    | Publishes this stream's telemetry under `SwerveInputStream/<name>`. Replaces any telemetry the stream already had. Call it last, after the rest of the configuration, so the dashboard starts with the configured values. |
| `getTelemetry()`                                           | `Optional<SwerveInputStreamTelemetry>` | The stream's telemetry, or empty if `withTelemetry` was not called.                                                                                                                                                       |

| Verbosity | Publishes                                                                                                                                                                                                                                                                                                                                                                                                                                          |
| --------- | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `LOW`     | The current drive mode (`mode`).                                                                                                                                                                                                                                                                                                                                                                                                                   |
| `MID`     | Also the stream's configuration, read-only: `deadband`, `translationScale`, `rotationScale`, `maxLinearVelocity`, `maxAngularVelocity`, `translationCube`, `rotationCube`, `allianceRelative`, `robotRelative`. And the rotation controller's inputs, `rotationController/measurement` and `rotationController/setpoint`, in radians on the field, from the drive's odometry rotation. They are `NaN` while the stream isn't using the controller. |
| `HIGH`    | Also editable copies of the configuration under `Tuning/SwerveInputStream/<name>`, and a `Live Tuning` command at `Tuning/SwerveInputStream/<name>/Live Tuning`.                                                                                                                                                                                                                                                                                   |

While the `Live Tuning` command runs, dashboard edits are applied to the stream every loop and changes made in code (for example a slow-mode binding calling `withScaleTranslation`) are published back to the dashboard. Dashboard values outside the allowed range are replaced with the stream's current value. The command requires no subsystem or mechanism, so it does not interrupt the command driving with the stream.

### SwerveInputStreamTelemetry

**Package:** `yams.commands2.telemetry` (Commands v2) or `yams.commands3.telemetry` (Commands v3). Implements `AutoCloseable`.

| Method                   | Returns             | Description                                                                                                                                                                                       |
| ------------------------ | ------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `getLiveTuningCommand()` | `Optional<Command>` | The `Live Tuning` command at `HIGH` verbosity, otherwise empty. A `org.wpilib.command2.Command` in v2, a `org.wpilib.command3.Command` in v3. Bind it to a button or start it from the dashboard. |
| `applyTuningValues()`    | `void`              | Applies dashboard edits to the stream and publishes code changes. The `Live Tuning` command calls this every loop. Does nothing below `HIGH`.                                                     |
| `updateTelemetry()`      | `void`              | Publishes the mode, the configuration and the rotation controller's inputs. Called by `SwerveInputStream.get()`.                                                                                  |
| `close()`                | `void`              | Stops publishing, canceling and removing the `Live Tuning` command. Use it when replacing a stream with another of the same name.                                                                 |

{% tabs %}
{% tab title="Commands v2" %}
In Commands v2 the `Live Tuning` command is a regular `Command`, published directly to the dashboard.

```java
import yams.commands2.swerve.SwerveInputStream;

SwerveInputStream driveStream = SwerveInputStream.of(drive, () -> -controller.getLeftY(), () -> -controller.getLeftX())
    .withControllerRotationAxis(() -> -controller.getRightX())
    .withDeadband(0.05)
    .withScaleTranslation(0.8)
    .withTelemetry("Driver", TelemetryVerbosity.HIGH);

// SwerveInputStream output is field relative.
setDefaultCommand(run(() -> drive.setFieldRelativeChassisSpeeds(driveStream.get())));
```
{% endtab %}

{% tab title="Commands v3" %}
Commands v3 `Command`s do not implement WPILib's `ComplexTunable`, so YAMS wraps the `Live Tuning` command in a `CommandTunable` (`yams.commands3.telemetry.CommandTunable`) before publishing it. `CommandTunable(Command command)` exposes a `running` boolean on the dashboard: setting it `true` schedules the command and `false` cancels it. YAMS uses the same wrapper for the v3 mechanism and swerve tuning commands; you only need it to publish your own v3 commands with `Tunables.publish(...)`.

```java
import yams.commands3.swerve.SwerveInputStream;

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

Where the mechanism drives with the active stream in a coroutine:

```java
public Command driveInputStream() {
    return run(coroutine -> {
        while (true) {
            drive.setFieldRelativeChassisSpeeds(inputStream.get());
            coroutine.yield();
        }
    }).named("Swerve Drive Input Stream");
}

public void setInputStream(SwerveInputStream inputStream) {
    if (this.inputStream != null) {
        // Stop publishing the old stream's telemetry before replacing it.
        this.inputStream.getTelemetry().ifPresent(SwerveInputStreamTelemetry::close);
    }
    this.inputStream = inputStream;
}
```
{% endtab %}
{% endtabs %}

***

## Example

```java
NiDsXboxController driver = new NiDsXboxController(0);

// Angular-velocity stream: left stick translates, right stick X rotates
SwerveInputStream driveInput = SwerveInputStream.of(swerveDrive,
        () -> -driver.getLeftY(),
        () -> -driver.getLeftX())
    .withControllerRotationAxis(() -> -driver.getRightX())
    .withDeadband(0.05)
    .withScaleTranslation(0.8)
    .withScaleRotation(0.6)
    .withAllianceRelativeControl()
    .withCubeTranslationControllerAxis();

// Heading-snap variant: clone so changes don't affect the original
SwerveInputStream headingInput = driveInput.clone()
    .withControllerHeadingAxis(driver::getRightX, driver::getRightY)
    .withHeadingControl(() -> driver.getRightStickButton());

// get() returns field-relative ChassisVelocities. Call it once per loop, e.g. in a command:
swerveDrive.setFieldRelativeChassisSpeeds(driveInput.get());
```

***

## See also

* [SwerveDrive](swerve-drive.md)
* [C++ SwerveInputStream](../../c++-reference/swerve/swerve-input-stream.md)
