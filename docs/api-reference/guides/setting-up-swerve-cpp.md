# Setting up swerve drive (C++)

**Goal:** Create a fully functional `SwerveDrive` subsystem in C++ with field-relative driving, odometry, and simulation.

The gains and encoder offsets in this guide are placeholders, as [the overview](guides.md#the-gains-in-these-guides) explains. The code blocks are pieces of one `DriveSubsystem` class: the hardware, configs, wrappers and modules are all members of it. The complete class is the `SwerveSubsystem.cpp` example at the end of the page.

***

## Prerequisites

Install the Commands v2 YAMS vendordep (`https://cdn.yassrobotics.com/yams_commands2.json`). `YAMS.hpp` does not pull in the vendor wrappers, so include the wrapper for your motor controller explicitly.

```cpp
#include <yams/YAMS.hpp>

// Or individually:
#include <yams/mechanisms/swerve/SwerveDrive.hpp>
#include <yams/mechanisms/swerve/SwerveDriveConfig.hpp>
#include <yams/mechanisms/swerve/SwerveModule.hpp>
#include <yams/mechanisms/swerve/utility/SwerveInputStream.hpp>
#include <yams/mechanisms/config/SwerveModuleConfig.hpp>
#include <yams/gearing/MechanismGearing.hpp>
#include <yams/motorcontrollers/SmartMotorControllerConfig.hpp>

// Vendor wrapper (always included individually):
#include <yams/motorcontrollers/remote/TalonFXWrapper.hpp>

// WPILib 2027 headers used below:
#include <wpi/commands2/CommandPtr.hpp>
#include <wpi/commands2/SubsystemBase.hpp>
#include <wpi/commands2/button/CommandXboxController.hpp>
#include <wpi/math/controller/PIDController.hpp>
#include <wpi/math/controller/SimpleMotorFeedforward.hpp>
#include <wpi/math/geometry/Pose2d.hpp>
#include <wpi/math/geometry/Translation2d.hpp>
#include <wpi/math/system/DCMotor.hpp>
```

WPILib 2027 renamed `ChassisSpeeds` to `wpi::math::ChassisVelocities` and `SwerveModuleState` to `wpi::math::SwerveModuleVelocity`. The YAMS method names (`SetFieldRelativeChassisSpeeds`, ...) are unchanged.

***

{% stepper %}
{% step %}
#### Configure drive and azimuth motor controllers

Each swerve module needs two `SmartMotorController` instances: one for the drive wheel (velocity control) and one for the azimuth (steering angle, position control).

```cpp
// Drive motor config: velocity PID + feedforward
yams::motorcontrollers::SmartMotorControllerConfig driveConfig;
driveConfig
    .WithSubsystem(this)
    .WithZeroPower(yams::motorcontrollers::SmartMotorControllerConfig::MotorMode::BRAKE)
    .WithMotorGearing(yams::gearing::MechanismGearing(6.75))
    .WithFeedback(0.1, 0.0, 0.0)                 // Placeholder PID gains.
    .WithFeedforward(wpi::math::SimpleMotorFeedforward<wpi::units::turns>{   // Placeholder kS, kV, kA.
        0.12_V, wpi::units::unit_t<wpi::math::SimpleMotorFeedforward<wpi::units::turns>::kv_unit>{2.2},
        wpi::units::unit_t<wpi::math::SimpleMotorFeedforward<wpi::units::turns>::ka_unit>{0.3}})   // kS, kV, kA
    .WithStatorCurrentLimit(60_A)
    .WithTelemetry("DriveFL");

// Azimuth motor config: position PID, continuous wrapping
yams::motorcontrollers::SmartMotorControllerConfig azimuthConfig;
azimuthConfig
    .WithSubsystem(this)
    .WithZeroPower(yams::motorcontrollers::SmartMotorControllerConfig::MotorMode::BRAKE)
    .WithMotorGearing(yams::gearing::MechanismGearing(12.8))
    .WithFeedback(7.0, 0.0, 0.3)                 // Placeholder PID gains.
    .WithContinuousWrapping(-0.5_tr, 0.5_tr)
    .WithStatorCurrentLimit(40_A)
    .WithTelemetry("AzimuthFL");
```

Repeat for each module (FR, BL, BR), adjusting the telemetry name and CAN IDs.

**Check it:** the project builds.
{% endstep %}

{% step %}
#### Construct motor controllers

The wrapper takes pointers to the vendor hardware object and to the config. Both must outlive the wrapper, so keep them as subsystem members.

```cpp
ctre::phoenix6::hardware::TalonFX driveTalonFL{10, ctre::phoenix6::CANBus{}};
ctre::phoenix6::hardware::TalonFX azimuthTalonFL{11, ctre::phoenix6::CANBus{}};
ctre::phoenix6::hardware::CANcoder encoderFL_{12, ctre::phoenix6::CANBus{}};  // the module's absolute encoder
ctre::phoenix6::hardware::Pigeon2 imu_{14, ctre::phoenix6::CANBus{}};         // the gyro, one per robot

yams::motorcontrollers::remote::TalonFXWrapper driveMotorFL{
    &driveTalonFL,
    wpi::math::DCMotor::KrakenX60(1),
    &driveConfig
};
yams::motorcontrollers::remote::TalonFXWrapper azimuthMotorFL{
    &azimuthTalonFL,
    wpi::math::DCMotor::KrakenX60(1),
    &azimuthConfig
};
```

**Check it:** the project builds.
{% endstep %}

{% step %}
#### Create a `SwerveModuleConfig` for each module

Pass drive and azimuth motor controllers, then set the wheel radius, the module location (relative to robot center), the absolute encoder supplier, and its offset.

```cpp
yams::mechanisms::config::SwerveModuleConfig frontLeftModuleConfig{
    &driveMotorFL, &azimuthMotorFL
};
frontLeftModuleConfig
    .WithWheelRadius(0.0508_m)                                  // 2-inch wheel radius
    .WithLocation(wpi::math::Translation2d{0.381_m, 0.381_m})  // front-left position
    .WithAbsoluteEncoder([this]() -> wpi::units::degree_t {
        return wpi::units::degree_t{wpi::units::turn_t{encoderFL_.GetAbsolutePosition().GetValue()}};
    })
    .WithAbsoluteEncoderOffset(130.0_deg)                       // Placeholder: measure it on the robot
    .WithTelemetry("FrontLeft",
        yams::motorcontrollers::SmartMotorControllerConfig::TelemetryVerbosity::HIGH);
```

Repeat for `FrontRight`, `BackLeft`, `BackRight` with their positions and offsets:

| Module      | Location                 |
| ----------- | ------------------------ |
| Front Left  | `{ +0.381_m, +0.381_m }` |
| Front Right | `{ +0.381_m, -0.381_m }` |
| Back Left   | `{ -0.381_m, +0.381_m }` |
| Back Right  | `{ -0.381_m, -0.381_m }` |

+x is toward the front of the robot and +y toward the left.

**Check it:** the project builds.
{% endstep %}

{% step %}
#### Construct `SwerveModule` instances

```cpp
yams::mechanisms::swerve::SwerveModule frontLeft{&frontLeftModuleConfig};
yams::mechanisms::swerve::SwerveModule frontRight{&frontRightModuleConfig};
yams::mechanisms::swerve::SwerveModule backLeft{&backLeftModuleConfig};
yams::mechanisms::swerve::SwerveModule backRight{&backRightModuleConfig};
```

**Check it:** the project builds.
{% endstep %}

{% step %}
#### Create a `SwerveDriveConfig`

Collect the four modules, set kinematic limits, and provide a gyro supplier. `WithGyro` takes either the yaw as a `wpi::units::degree_t`, as here, or the gyro's full attitude as a `wpi::math::Rotation3d`.

```cpp
yams::mechanisms::swerve::SwerveDriveConfig driveConfig;
driveConfig
    .WithSubsystem(this)
    .WithModules({&frontLeft, &frontRight, &backLeft, &backRight})
    .WithGyro([this]() -> wpi::units::degree_t {
        return wpi::units::degree_t{wpi::units::turn_t{imu_.GetYaw().GetValue()}};
    })
    .WithMaximumChassisSpeed(4.5_mps, wpi::units::degrees_per_second_t{360.0})
    .WithTranslationController(wpi::math::PIDController{1.0, 0, 0})
    .WithRotationController(wpi::math::PIDController{1.0, 0, 0})
    .WithTelemetry(
        "swerve",
        yams::motorcontrollers::SmartMotorControllerConfig::TelemetryVerbosity::HIGH);
```

**Check it:** the project builds.
{% endstep %}

{% step %}
#### Construct `SwerveDrive`

The template parameter must match the number of modules passed to `WithModules`:

```cpp
yams::mechanisms::swerve::SwerveDrive<4> swerveDrive{&driveConfig};
```

**Check it:** run the simulation. The code starts without an exception.
{% endstep %}

{% step %}
#### Integrate into a subsystem

The subsystem owns the `SwerveInputStream` so it lives as long as the drive command that reads it. The constructor and the members are left out here: they are the pieces above.

```cpp
class DriveSubsystem : public wpi::cmd::SubsystemBase {
public:
    using InputStream = yams::mechanisms::swerve::utility::SwerveInputStream<4>;

    DriveSubsystem() { /* configure as shown above, then swerveDrive_.emplace(&driveConfig_) */ }

    yams::mechanisms::swerve::SwerveDrive<4>& GetSwerveDrive() { return swerveDrive_.value(); }

    /** Field-relative drive command driven by a SwerveInputStream. */
    wpi::cmd::CommandPtr DriveFieldRelativeCommand(InputStream inputStream) {
        inputStream_.emplace(std::move(inputStream));
        return Run([this] {
            swerveDrive_->SetFieldRelativeChassisSpeeds(inputStream_->Get());
        }).WithName("Field Oriented Drive");
    }

    void AddVisionMeasurement(wpi::math::Pose2d pose, wpi::units::second_t timestamp) {
        swerveDrive_->AddVisionMeasurement(pose, timestamp);
    }

    wpi::math::Pose2d GetPose() { return swerveDrive_->GetPose(); }
    void ZeroGyro() { swerveDrive_->ZeroGyro(); }

    void Periodic() override { swerveDrive_->UpdateTelemetry(); }
    void SimulationPeriodic() override { swerveDrive_->SimIterate(); }

private:
    // ... hardware, motor configs, wrappers, module configs, modules, drive config ...
    std::optional<yams::mechanisms::swerve::SwerveDrive<4>> swerveDrive_;
    std::optional<InputStream> inputStream_;
};
```

**Check it:** run the simulation. The drive shows in NetworkTables under **Mechanisms -> swerve**, with a field showing the robot at the origin.
{% endstep %}

{% step %}
#### Set up driver input with `SwerveInputStream`

`SwerveInputStream<4>` translates raw joystick axes into field-relative `wpi::math::ChassisVelocities`. Pass the `SwerveDrive` reference as the first argument, then set a rotation axis with `WithControllerRotationAxis`.

```cpp
// In RobotContainer or a drive command factory:
using InputStream = yams::mechanisms::swerve::utility::SwerveInputStream<4>;

InputStream driverInput =
    InputStream::Of(
        driveSubsystem_.GetSwerveDrive(),
        [this] { return -driverController_.GetLeftY(); },   // forward/back
        [this] { return -driverController_.GetLeftX(); }    // strafe
    )
    .WithControllerRotationAxis([this] { return -driverController_.GetRightX(); })
    .WithDeadband(0.1)
    .WithMaximumLinearVelocity(4.5_mps)
    .WithMaximumAngularVelocity(wpi::units::radians_per_second_t{2 * std::numbers::pi})
    .WithAllianceRelativeControl()
    .WithCubeTranslationControllerAxis()
    .WithTelemetry("Driver",
        yams::motorcontrollers::SmartMotorControllerConfig::TelemetryVerbosity::HIGH);

driveSubsystem_.SetDefaultCommand(
    driveSubsystem_.DriveFieldRelativeCommand(driverInput)
);
```

`WithTelemetry(name, verbosity)` publishes the stream under `SwerveInputStream/<name>` every time it is read. At `HIGH` the stream is also live tunable from the dashboard. Call it last, after the rest of the configuration.

**Check it:** in simulation, enable teleop with a controller connected (or a keyboard joystick in the Sim GUI). The left stick drives the robot on the field and the right stick turns it.
{% endstep %}

{% step %}
#### Heading-snap mode (optional)

Clone the input stream to add heading control without affecting the base stream. A `Clone()` has no telemetry, so give it its own name if you want it published.

```cpp
InputStream headingInput =
    driverInput.Clone()
    .WithControllerHeadingAxis(
        [this] { return driverController_.GetRightX(); },
        [this] { return driverController_.GetRightY(); }
    )
    .WithHeadingControl([this] { return driverController_.RightStick().Get(); });
```

**Check it:** drive with `headingInput` and hold the right stick in. The robot turns to face where the right stick points, and holds that heading.
{% endstep %}

{% step %}
#### Azimuth encoder seeding

Azimuth encoders are seeded automatically during `SwerveModule` construction. If you need to re-seed after the CAN bus has fully settled, call `SeedAzimuthEncoder()` on each module:

```cpp
frontLeft.SeedAzimuthEncoder();
frontRight.SeedAzimuthEncoder();
backLeft.SeedAzimuthEncoder();
backRight.SeedAzimuthEncoder();
```

**Check it:** on the robot, with the robot disabled, turn each wheel by hand. Its azimuth angle in telemetry follows the wheel. If a wheel points the wrong way when enabled, its encoder offset is wrong.
{% endstep %}

{% step %}
#### Auto-align (`DriveToPose`) and live PID tuning

`DriveToPose(wpi::math::Pose2d)` returns a `wpi::cmd::CommandPtr` that drives the robot to a field-relative pose using the `WithTranslationController`/`WithRotationController` PID gains set on `SwerveDriveConfig`:

```cpp
wpi::cmd::CommandPtr DriveSubsystem::DriveToPoseCommand(wpi::math::Pose2d target) {
    return swerveDrive_->DriveToPose(target);
}
```

At `TelemetryVerbosity::HIGH`, `SwerveDrive<N>` also publishes a live tuning command for these gains and for the module gains, so you can tune them without redeploying. [SwerveDrive (C++): Auto-Align](../c++-reference/swerve/swerve-drive.md#auto-align-drive-to-pose-and-pid-control) lists its fields.

**Check it:** in simulation, schedule `DriveToPoseCommand` with a pose 2 m forward and 1 m left of the origin. The robot drives there and stops.
{% endstep %}
{% endstepper %}

***

## Key SwerveDrive methods

| Method                                                               | Description                                                                                 |
| -------------------------------------------------------------------- | ------------------------------------------------------------------------------------------- |
| `SetFieldRelativeChassisSpeeds(ChassisVelocities)`                   | Field-relative drive                                                                        |
| `SetRobotRelativeChassisSpeeds(ChassisVelocities)`                   | Robot-relative drive                                                                        |
| `Drive(std::function<ChassisVelocities()>)`                          | Returns a run `CommandPtr` for continuous robot-relative driving                            |
| `DriveToPose(Pose2d)`                                                | Returns a `CommandPtr` that PID-drives to a field-relative pose                             |
| `DriveToPoseSetpoint(Pose2d)`                                        | Computes one loop's `ChassisVelocities` toward a pose (building block behind `DriveToPose`) |
| `SetTranslationPID(PIDController)` / `SetRotationPID(PIDController)` | Replace the auto-align PID controllers (integrator preserved unless gains changed)          |
| `ResetTranslationPID()` / `ResetRotationPID()`                       | Reset the auto-align PID controllers' internal state                                        |
| `LockPose()`                                                         | X-pattern to resist pushing                                                                 |
| `GetPose()`                                                          | Current field-relative pose from odometry                                                   |
| `GetGyroAngle()`                                                     | Current gyro heading as a `degree_t`                                                        |
| `GetGyroRotation3d()`                                                | The gyro's full attitude as a `Rotation3d`                                                  |
| `ResetOdometry(Pose2d)`                                              | Reset odometry to a known pose                                                              |
| `ZeroOdometryHeading()`, or `ZeroGyro()`                             | Make the way the robot faces the zero heading for field-relative driving                    |
| `AddVisionMeasurement(Pose2d, second_t)`                             | Fuse vision pose into odometry                                                              |
| `GetSimPose()`                                                       | Ground-truth simulated pose, useful for feeding a simulated vision system                   |
| `UpdateTelemetry()`                                                  | Call in `Periodic()`                                                                        |
| `SimIterate()`                                                       | Call in `SimulationPeriodic()`                                                              |

***

## Notes

{% hint style="warning" %}
`WithSubsystem(this)` must be set on every `SmartMotorControllerConfig` **and** on `SwerveDriveConfig` before construction. Without it, command scheduling will not work correctly.
{% endhint %}

{% hint style="info" %}
`SwerveInputStream::Of()` takes translation axes only. Add rotation later with `WithControllerRotationAxis()` or switch to heading control with `WithControllerHeadingAxis()` + `WithHeadingControl()`.
{% endhint %}

{% hint style="info" %}
**Gyro mounted on its side?** Supply the full attitude with `WithGyro` (a `Rotation3d`), then call `.WithGyroHeadingAxis(yams::mechanisms::config::GyroAxis::ROLL)` or `GyroAxis::PITCH` on `SwerveDriveConfig` to say which gyro axis points up through the robot. The default is `GyroAxis::YAW`, for a gyro mounted flat.
{% endhint %}

## If it did not work

* **It does not build.** Check the vendordep, the wrapper `#include`, and that every hardware object, config, wrapper and module is a member that outlives what points at it.
* **The robot drives the wrong way, or spins when it should drive straight.** Check each module's `WithLocation` signs, then that the drive motors are not inverted.
* **A wheel points the wrong way, or the wheels fight each other.** The absolute encoder offsets are wrong. Measure them as in [On the real robot](https://yams.yassrobotics.com/documentation/tutorials/swerve-drive/real-robot).
* **Field-relative driving is rotated.** The gyro supplier is wrong, or the gyro was not zeroed facing away from the driver.

[Troubleshooting](https://yams.yassrobotics.com/documentation/tutorials/troubleshooting) covers more.

***

## Examples

Complete C++ implementations from the `cpptest` reference project:

{% @github-files/github-code-block url="https://github.com/Yet-Another-Software-Suite/YAMS/blob/master/examples/cpptest/src/main/cpp/subsystems/SwerveSubsystem.cpp" %}

***

## Related pages

* [SwerveDrive (C++)](../c++-reference/swerve/swerve-drive.md)
* [SwerveModule (C++)](../c++-reference/swerve/swerve-module.md)
* [SwerveInputStream (C++)](../c++-reference/swerve/swerve-input-stream.md)
* [SmartMotorControllerConfig (C++)](../c++-reference/motor-controllers/smart-motor-controller-config.md)
* [Java equivalent: Setting Up Swerve Drive](setting-up-swerve.md)
