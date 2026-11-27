# Overview

The YAMS C++ API mirrors the Java API under the `yams` namespace. Mechanism classes live in `yams::mechanisms` (with `positional`, `velocity`, `config` and `swerve` sub-namespaces), motor controller types in `yams::motorcontrollers` (with `local`, `remote` and `simulation` sub-namespaces), telemetry in `yams::telemetry`, gearing in `yams::gearing`, and math utilities in `yams::math`. Configuration classes use the same fluent `With*` builder methods as their Java counterparts.

YAMS C++ targets WPILib 2027, where the WPILib C++ types live in the `wpi::` namespace with `.hpp` headers. All physical quantities use the WPILib units library under `wpi::units` (for example `wpi::units::degree_t` instead of Java's `Angle`, and `wpi::units::meters_per_second_t` for velocity, from headers such as `<wpi/units/angle.hpp>`). Other WPILib types you will see in signatures include `wpi::math::DCMotor` (`<wpi/math/system/DCMotor.hpp>`), `wpi::math::Rotation2d` / `wpi::math::Pose2d`, `wpi::math::ChassisVelocities` and `wpi::math::SwerveModuleVelocity`, and the command types `wpi::cmd::CommandPtr`, `wpi::cmd::Trigger` and `wpi::cmd::SubsystemBase` (`<wpi/commands2/...>`). The `SwerveDrive` and `SwerveInputStream` classes are templates parameterized on the number of modules (default 4), matching the WPILib `SwerveDriveKinematics` convention.

{% hint style="info" %}
**C++ is Commands v2 only.** WPILib has not published a C++ Commands v3 API, so the YAMS C++ library ships only in the Commands v2 vendordep (`https://cdn.yassrobotics.com/yams_commands2.json`). The Commands v3 vendordep (`yams_commands3.json`) contains no C++ artifacts. All C++ command factories return `wpi::cmd::CommandPtr` and all triggers are `wpi::cmd::Trigger`.
{% endhint %}

**Headers**

Every class has its own header under `yams/` that matches its namespace, for example `#include <yams/mechanisms/positional/Arm.hpp>` or `#include <yams/motorcontrollers/local/SparkWrapper.hpp>`. The convenience header `#include <yams/YAMS.hpp>` pulls in the gearing, math, units, mechanism, swerve, telemetry and exception headers, but not the vendor-specific motor controller wrappers (`SparkWrapper`, `NovaWrapper`, `A301Wrapper`, `TalonFXWrapper`, `TalonFXSWrapper`), `EasyCRT`, or the simulation `Sensor` / `BatterySim` headers; include those directly.

**Top-level namespaces**

* `yams::mechanisms`: SmartMechanism base class
* `yams::mechanisms::positional`: Arm, Elevator, Pivot
* `yams::mechanisms::velocity`: FlyWheel
* `yams::mechanisms::config`: ArmConfig, ElevatorConfig, PivotConfig, FlyWheelConfig, MechanismPositionConfig, SimSensorConfig, SwerveModuleConfig
* `yams::mechanisms::swerve`: SwerveDrive\<N>, SwerveDriveConfig, SwerveModule
* `yams::mechanisms::swerve::utility`: SwerveInputStream\<N>, SwerveInputStreamTelemetry\<N>
* `yams::motorcontrollers`: SmartMotorController, SmartMotorControllerConfig, YMotors
* `yams::motorcontrollers::local`: SparkWrapper, NovaWrapper, A301Wrapper
* `yams::motorcontrollers::remote`: TalonFXWrapper, TalonFXSWrapper
* `yams::motorcontrollers::simulation`: Sensor, SensorData, BatterySim, sim suppliers
* `yams::telemetry`: SmartMotorControllerTelemetryConfig, SwerveDriveTelemetryConfig, SwerveModuleTelemetryConfig, MechanismTelemetry
* `yams::gearing`: GearBox, Sprocket, MechanismGearing
* `yams::math`: LQRConfig, LQRController, DerivativeTimeFilter, SmartMath
* `yams::units`: EasyCRT, extra unit types (`YUnits.hpp`)
* `yams::exceptions`: YamsException and subclasses
