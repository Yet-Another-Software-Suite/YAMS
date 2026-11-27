# SwerveInputStream

**Namespace:** `yams::mechanisms::swerve::utility`\
**Header:** `#include <yams/mechanisms/swerve/utility/SwerveInputStream.hpp>`\
**Template:** `template <size_t NumModules = 4>`\
**Java equivalent:** [Java SwerveInputStream](../../java-reference/swerve/swerve-input-stream.md)\
**Doxygen:** [`SwerveInputStream`](https://doxygen.yassrobotics.com/yams/classyams_1_1mechanisms_1_1swerve_1_1utility_1_1SwerveInputStream.html)

`SwerveInputStream<N>` converts raw joystick inputs into **field-relative** `wpi::math::ChassisVelocities` for a `SwerveDrive<N>`. Handles deadbands, cubic input curves, axis scaling, alliance flipping, and multiple drive modes (angular-velocity, heading-angle, aim-at-target, translation-only). It can also publish its own telemetry and be live tuned from the dashboard (see [Telemetry & Live Tuning](swerve-input-stream.md#telemetry--live-tuning)).

{% hint style="info" %}
The stream keeps a pointer to the `SwerveDrive<N>` it was built from, so the drive must outlive it. Store the stream as a member of your subsystem rather than a local in a command factory.
{% endhint %}

***

## Factory method

```cpp
static SwerveInputStream Of(SwerveDrive<NumModules>& drive,
                            std::function<double()> x,
                            std::function<double()> y)
```

Constructs a `SwerveInputStream` with translation axes only. Chain `WithControllerRotationAxis()` or `WithControllerHeadingAxis()` to add a rotation source before calling `Get()`.

***

## Constructors

```cpp
// Variant 1: with an angular-velocity rotation axis
SwerveInputStream(SwerveDrive<NumModules>& drive,
                  std::function<double()> x,
                  std::function<double()> y,
                  std::function<double()> rot)

// Variant 2: with heading X/Y axes (heading-angle mode)
SwerveInputStream(SwerveDrive<NumModules>& drive,
                  std::function<double()> x,
                  std::function<double()> y,
                  std::function<double()> headingX,
                  std::function<double()> headingY)
```

The second constructor pre-configures heading axes; pair it with `WithHeadingControl()` to activate heading-angle mode.

***

## Builder methods

All builder methods return `SwerveInputStream&` for chaining.

### Velocity limits

| Method                       | Signature                                                               | Description                                                             |
| ---------------------------- | ----------------------------------------------------------------------- | ----------------------------------------------------------------------- |
| `WithMaximumLinearVelocity`  | `WithMaximumLinearVelocity(wpi::units::meters_per_second_t velocity)`   | Maximum chassis translational speed. Default is 4 m/s.                  |
| `WithMaximumAngularVelocity` | `WithMaximumAngularVelocity(wpi::units::radians_per_second_t velocity)` | Maximum chassis rotational speed. Default is 2 pi rad/s (1 rotation/s). |

{% hint style="info" %}
A stream copies `SwerveDriveConfig::WithMaximumChassisSpeed(...)`'s value (if set) into its own maximum fields once, at construction time; otherwise it starts from its own defaults (4 m/s, 2 pi rad/s). Calling `WithMaximumLinearVelocity(...)`/`WithMaximumAngularVelocity(...)` afterward overrides that value for this stream and takes effect immediately; the drive's config is not re-checked after construction.
{% endhint %}

### Axis configuration

| Method                       | Signature                                                                                       | Description                                                                                                                                                                  |
| ---------------------------- | ----------------------------------------------------------------------------------------------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `WithControllerRotationAxis` | `WithControllerRotationAxis(std::function<double()> rot)`                                       | Set or replace the rotation axis supplier. Range `[-1, 1]`. Enables `ANGULAR_VELOCITY` mode.                                                                                 |
| `WithControllerHeadingAxis`  | `WithControllerHeadingAxis(std::function<double()> headingX, std::function<double()> headingY)` | Set or replace the heading axis suppliers. Range `[-1, 1]`. The target heading is `atan2(headingX, headingY)`. Enables `HEADING` mode when `WithHeadingControl()` is active. |
| `WithHeading`                | `WithHeading(std::function<wpi::units::radian_t()> heading)`                                    | Supplies a target heading directly instead of from controller axes (replaces any heading axes). Used in `HEADING` mode.                                                      |
| `WithDeadband`               | `WithDeadband(double deadband)`                                                                 | Deadband applied to all axes (0.0 to 1.0). Pass 0 to disable.                                                                                                                |
| `WithScaleTranslation`       | `WithScaleTranslation(double scale)`                                                            | Multiplies translation output magnitude by `scale`. Range `(0, 1]`.                                                                                                          |
| `WithScaleRotation`          | `WithScaleRotation(double scale)`                                                               | Multiplies rotation axis value by `scale`. Range `(0, 1]`.                                                                                                                   |

### Input curves

| Method                              | Signature                                                                                           | Description                                                                       |
| ----------------------------------- | --------------------------------------------------------------------------------------------------- | --------------------------------------------------------------------------------- |
| `WithCubeTranslationControllerAxis` | `WithCubeTranslationControllerAxis()` or `WithCubeTranslationControllerAxis(std::function<bool()>)` | Applies a cubic input curve to translation magnitude for finer low-speed control. |
| `WithCubeRotationControllerAxis`    | `WithCubeRotationControllerAxis()` or `WithCubeRotationControllerAxis(std::function<bool()>)`       | Applies a cubic curve to the rotation axis.                                       |

### Drive modes

| Method                     | Signature                                                                                | Description                                                                                                                |
| -------------------------- | ---------------------------------------------------------------------------------------- | -------------------------------------------------------------------------------------------------------------------------- |
| `WithHeadingControl`       | `WithHeadingControl(std::function<bool()> trigger)`                                      | Enables heading-angle control mode when trigger returns `true`. Requires `WithControllerHeadingAxis()` or `WithHeading()`. |
| `WithAim`                  | `WithAim(std::function<wpi::math::Pose2d()> aimTarget, std::function<bool()> trigger)`   | Rotates the robot to face `aimTarget` while translating. Enables `AIM` mode when trigger is `true`.                        |
| `WithAimLookahead`         | `WithAimLookahead(wpi::units::second_t lookahead)`                                       | Aims from where the robot will be after the lookahead time, moving the pose along its field relative velocity.             |
| `WithAimFeedforward`       | `WithAimFeedforward(wpi::math::SimpleMotorFeedforward<wpi::units::radians> feedforward)` | Adds a feedforward to the aim rotation so the robot keeps up with a target whose direction is changing.                    |
| `WithAimHeadingOffset`     | `WithAimHeadingOffset(wpi::math::Rotation2d offset)`                                     | Rotates the aim heading, for a shooter that does not point out of the front of the robot.                                  |
| `WithAimHeadingOffset`     | `WithAimHeadingOffset(wpi::math::Rotation2d offset, std::function<bool()> enabled)`      | Rotates the aim heading while `enabled` returns `true`.                                                                    |
| `IsAimLocked`              | `IsAimLocked(wpi::units::radian_t tolerance)`                                            | Whether the robot is aiming and within the tolerance of the aim heading.                                                   |
| `GetTargetVector`          | `GetTargetVector(const wpi::math::Pose2d& target)`                                       | Field relative vector from the robot to the target, from the looked ahead position if a lookahead is set.                  |
| `CalculateAngularVelocity` | `CalculateAngularVelocity(wpi::units::radian_t target)`                                  | Angular velocity that turns the robot to a heading, from the rotation controller and the aim feedforward.                  |
| `WithTranslationOnly`      | `WithTranslationOnly(std::function<bool()> trigger)`                                     | Suppresses rotation and locks the current heading while trigger returns `true`.                                            |

{% hint style="warning" %}
`HEADING`, `AIM` and `TRANSLATION_ONLY` modes use the drive's rotation PID controller. If `SwerveDriveConfig::WithRotationController(...)` was not called, `Get()` throws a `SwerveDriveConfigurationException` when one of those modes becomes active.
{% endhint %}

### Field orientation

| Method                         | Signature                                                                                                                                                 | Description                                                                                                                                                                |
| ------------------------------ | --------------------------------------------------------------------------------------------------------------------------------------------------------- | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `WithAllianceRelativeControl`  | `WithAllianceRelativeControl()` or `WithAllianceRelativeControl(std::function<bool()>)`                                                                   | Rotates the translation input by 180 degrees on the Red alliance so "forward" always points away from your driver station. Ignored while robot-relative control is active. |
| `WithRobotRelative`            | `WithRobotRelative()` or `WithRobotRelative(std::function<bool()>)`                                                                                       | Treats the translation input as robot-relative and converts it to field-relative using the drive's odometry rotation. The output of `Get()` is still field-relative.       |
| `WithTranslationHeadingOffset` | `WithTranslationHeadingOffset(wpi::math::Rotation2d angle)` or `WithTranslationHeadingOffset(wpi::math::Rotation2d angle, std::function<bool()> enabled)` | Rotates the output translation vector by a fixed `angle`.                                                                                                                  |

### Utility

| Method  | Signature                         | Description                                                                                                       |
| ------- | --------------------------------- | ----------------------------------------------------------------------------------------------------------------- |
| `Clone` | `SwerveInputStream Clone() const` | Returns a copy of this stream without telemetry. Subsequent `With*` calls on the copy do not affect the original. |

***

## Output

| Method       | Signature                                   | Description                                                                                                                                       |
| ------------ | ------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------- |
| `Get`        | `wpi::math::ChassisVelocities Get()`        | Computes and returns the current **field-relative** `ChassisVelocities`, and updates the stream's telemetry if enabled. Call once per robot loop. |
| `operator()` | `wpi::math::ChassisVelocities operator()()` | Same as `Get()`, allows using the stream as a `std::function<wpi::math::ChassisVelocities()>` directly.                                           |

{% hint style="warning" %}
`Get()` returns field-relative speeds. Send them to `SwerveDrive::SetFieldRelativeChassisSpeeds(...)`. `SwerveDrive::Drive(...)` expects robot-relative speeds, so do not pass the stream to it directly.
{% endhint %}

***

## Drive modes

`SwerveInputStream` automatically selects the active drive mode each loop based on which suppliers are configured and active.

| Mode               | Description                                                                                                                                                   |
| ------------------ | ------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `ANGULAR_VELOCITY` | Default. Rotates using the angular-velocity axis (`WithControllerRotationAxis`).                                                                              |
| `HEADING`          | Controls robot heading via right-stick X/Y angle (or `WithHeading`). Active when `WithHeadingControl()` trigger is `true` and a heading source is configured. |
| `AIM`              | Rotates to face a target `wpi::math::Pose2d`. Active when `WithAim()` trigger is `true` and a target is set.                                                  |
| `TRANSLATION_ONLY` | Suppresses rotation and holds the current heading. Active when `WithTranslationOnly()` trigger is `true`, or when no rotation source is configured.           |

Mode priority (highest to lowest): `TRANSLATION_ONLY` > `AIM` > `HEADING` > `ANGULAR_VELOCITY`. `GetCurrentModeName()` returns the active mode as a string.

***

## Telemetry & live tuning

Telemetry is off until you call `WithTelemetry(name, verbosity)` on a stream. It then publishes every time the stream is read with `Get()`, so there is nothing to call each loop.

| Method                             | Signature                                                                                      | Description                                                                                                                                                                                                                           |
| ---------------------------------- | ---------------------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `WithTelemetry`                    | `SwerveInputStream& WithTelemetry(std::string name, TelemetryVerbosity verbosity)`             | Publishes this stream under `SwerveInputStream/<name>`. Replaces any telemetry the stream already had; `TelemetryVerbosity::NONE` turns it off.                                                                                       |
| `GetTelemetry`                     | `std::optional<std::reference_wrapper<SwerveInputStreamTelemetry<NumModules>>> GetTelemetry()` | The stream's telemetry, or empty if telemetry is not enabled. Creates it on first call.                                                                                                                                               |
| `GetCurrentModeName`               | `std::string GetCurrentModeName() const`                                                       | Active drive mode (`ANGULAR_VELOCITY`, `HEADING`, `AIM`, `TRANSLATION_ONLY`).                                                                                                                                                         |
| `GetRotationControllerMeasurement` | `std::optional<wpi::units::radian_t> GetRotationControllerMeasurement() const`                 | The measurement the last `Get()` gave the rotation controller in `HEADING`, `AIM` or `TRANSLATION_ONLY`: the robot's heading, on the field, from the drive's odometry rotation. Empty if the last `Get()` did not use the controller. |
| `GetRotationControllerSetpoint`    | `std::optional<wpi::units::radian_t> GetRotationControllerSetpoint() const`                    | The setpoint the last `Get()` gave the rotation controller: the heading being turned to, on the field, from the drive's odometry rotation. Empty if the last `Get()` did not use the controller.                                      |

`TelemetryVerbosity` is `SwerveDriveConfig::TelemetryVerbosity` (`NONE`, `LOW`, `MEDIUM`, `HIGH`; `MEDIUM` is Java's `MID`):

| Verbosity | Publishes                                                                                                                                                                                                                                                                                                         |
| --------- | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `LOW`     | The current drive mode (`SwerveInputStream/<name>/mode`).                                                                                                                                                                                                                                                         |
| `MEDIUM`  | Also the stream's configuration, read-only, under `SwerveInputStream/<name>/`. And the rotation controller's inputs, `rotationController/measurement` and `rotationController/setpoint`, in radians on the field, from the drive's odometry rotation. They are `NaN` while the stream isn't using the controller. |
| `HIGH`    | Also editable copies of the configuration under `Tuning/SwerveInputStream/<name>/`, and a `Live Tuning` command at `Tuning/SwerveInputStream/<name>/Live Tuning`.                                                                                                                                                 |

The tunable values are `deadband` `[0, 1)`, `translationScale` `(0, 1]`, `rotationScale` `(0, 1]`, `maxLinearVelocity` (m/s, > 0), `maxAngularVelocity` (rad/s, > 0), and the booleans `translationCube`, `rotationCube`, `allianceRelative` and `robotRelative`. The output velocities are not published (reading them would run the heading controller a second time each loop); log the speeds you send to the drive instead.

At `HIGH`, dashboard edits are applied to the stream every loop while the `Live Tuning` command runs. Start and stop it from the dashboard, or bind it to a button through `SwerveInputStreamTelemetry::GetLiveTuningCommand()`. Values stay in sync both ways: a change made in code (for example a slow-mode binding calling `WithScaleTranslation`) is published to the dashboard and is not overridden, and a dashboard value outside its allowed range is replaced with the stream's current value. The command has no requirements, so it does not interrupt the command driving with the stream.

**`SwerveInputStreamTelemetry<N>`** (`#include <yams/mechanisms/swerve/utility/SwerveInputStreamTelemetry.hpp>`, already included by `SwerveInputStream.hpp`):

| Method                 | Signature                                                                         | Description                                                                                                                    |
| ---------------------- | --------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------ |
| `GetLiveTuningCommand` | `std::optional<std::reference_wrapper<wpi::cmd::Command>> GetLiveTuningCommand()` | The `Live Tuning` command at `HIGH`, otherwise empty.                                                                          |
| `ApplyTuningValues`    | `void ApplyTuningValues()`                                                        | Applies dashboard edits and publishes code changes. Called every loop by the `Live Tuning` command; does nothing below `HIGH`. |
| `UpdateTelemetry`      | `void UpdateTelemetry()`                                                          | Publishes the mode and, at `MEDIUM` and above, the configuration and the rotation controller's inputs. Called by `Get()`.      |

The live-tuning values map to plain getters and setters on the stream, which you can also call yourself: `GetAxisDeadband`/`SetAxisDeadband`, `GetTranslationAxisScale`/`SetTranslationAxisScale`, `GetOmegaAxisScale`/`SetOmegaAxisScale`, `GetMaximumChassisLinearVelocity`/`SetMaximumChassisLinearVelocity`, `GetMaximumChassisAngularVelocity`/`SetMaximumChassisAngularVelocity`, `IsTranslationCubeEnabled`/`SetTranslationCubeEnabled`, `IsOmegaCubeEnabled`/`SetOmegaCubeEnabled`, `IsAllianceRelativeEnabled`/`SetAllianceRelativeEnabled`, and `IsRobotRelativeEnabled`/`SetRobotRelativeEnabled`.

{% hint style="info" %}
The stream is a value type, so telemetry ownership differs from Java:

* `WithTelemetry` only stores the name and verbosity. The telemetry is created when the stream is first read with `Get()` (or `GetTelemetry()` is called), so the temporary copies made while building a stream do not publish. Call `WithTelemetry` last so the dashboard starts with the configured values.
* The stream owns its telemetry and stops publishing (and removes the `Live Tuning` command) when it is destroyed.
* A copy of a stream keeps the telemetry settings and publishes under the same name once it is read. `Clone()` returns a copy without telemetry.
* Moving a stream moves the settings; the moved-from stream stops publishing.
* To stop publishing, call `WithTelemetry(name, TelemetryVerbosity::NONE)`. Use `LOW` in competition code.
{% endhint %}

{% hint style="info" %}
The C++ API is WPILib Commands v2 only: the `Live Tuning` command is a `wpi::cmd::Command`. WPILib has no C++ Commands v3.
{% endhint %}

***

## Example

```cpp
#include <yams/mechanisms/swerve/utility/SwerveInputStream.hpp>

using namespace yams::mechanisms::swerve;
using namespace yams::mechanisms::swerve::utility;

// Members of your swerve subsystem (declared after m_drive and m_controller).
SwerveInputStream<4> m_driveStream =
    SwerveInputStream<4>::Of(m_drive,
                             [this] { return -m_controller.GetLeftY(); },
                             [this] { return -m_controller.GetLeftX(); })
        .WithControllerRotationAxis([this] { return -m_controller.GetRightX(); })
        .WithDeadband(0.1)
        .WithScaleTranslation(0.8)
        .WithCubeTranslationControllerAxis()
        .WithAllianceRelativeControl()
        .WithTelemetry("Driver", SwerveDriveConfig::TelemetryVerbosity::HIGH);

// Heading-snap variant using Clone() (no telemetry of its own):
SwerveInputStream<4> m_headingStream =
    m_driveStream.Clone()
        .WithControllerHeadingAxis([this] { return m_controller.GetRightX(); },
                                   [this] { return m_controller.GetRightY(); })
        .WithHeadingControl([this] { return m_controller.GetRightStickButton(); });

// The stream output is field relative.
wpi::cmd::CommandPtr DriveCommand() {
  return Run([this] { m_drive.SetFieldRelativeChassisSpeeds(m_driveStream.Get()); });
}
```

***

## See also

* [Java SwerveInputStream](../../java-reference/swerve/swerve-input-stream.md)
* [C++ SwerveDrive\<N>](swerve-drive.md)
* [SwerveDriveConfig](swerve-drive-config.md)
