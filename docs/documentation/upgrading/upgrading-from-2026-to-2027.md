---
icon: arrow-up-right-dots
---

# Upgrading from 2026 to 2027

YAMS 2027 is built on WPILib 2027 (`2027_alpha7`) and runs on the SystemCore. Upgrading a 2026 robot project touches four areas:

1. [WPILib 2027 itself](upgrading-from-2026-to-2027.md#wpilib-2027-and-the-systemcore): new `org.wpilib.*` packages, Java 25 and the 2027 vendor libraries.
2. [The YAMS vendordep](upgrading-from-2026-to-2027.md#install-the-2027-yams-vendordep): there are now two of them (Commands v2 and Commands v3), served from `cdn.yassrobotics.com`.
3. [The YAMS Java packages](upgrading-from-2026-to-2027.md#yams-java-package-split): YAMS is split into `yams.core`, `yams.commands2` and `yams.commands3`.
4. [YAMS API changes](upgrading-from-2026-to-2027.md#renamed-and-changed-apis): mechanism constructors, mechanism configs, Trigger factories, live tuning, encoder getters and swerve gyro handling.

{% hint style="warning" %}
These are breaking changes. Code written against 2026 YAMS will fail to compile until it's updated using the patterns below. The [Quick checklist](upgrading-from-2026-to-2027.md#quick-checklist) at the end of this page lists every step.
{% endhint %}

## WPILib 2027 and the SystemCore

Install the WPILib 2027 (`2027_alpha7`) release and create or import your project with it. The parts that affect YAMS code are:

*   **Java packages are now `org.wpilib.*`.** `edu.wpi.first.*` imports no longer exist. For example:

    | 2026 import                                                | 2027 import                                       |
    | ---------------------------------------------------------- | ------------------------------------------------- |
    | `edu.wpi.first.units.Units`                                | `org.wpilib.units.Units`                          |
    | `edu.wpi.first.units.measure.Angle`                        | `org.wpilib.units.measure.Angle`                  |
    | `edu.wpi.first.math.system.plant.DCMotor`                  | `org.wpilib.math.system.DCMotor`                  |
    | `edu.wpi.first.math.controller.ArmFeedforward`             | `org.wpilib.math.controller.ArmFeedforward`       |
    | `edu.wpi.first.math.geometry.Pose2d`                       | `org.wpilib.math.geometry.Pose2d`                 |
    | `edu.wpi.first.math.kinematics.ChassisSpeeds`              | `org.wpilib.math.kinematics.ChassisVelocities`    |
    | `edu.wpi.first.math.kinematics.SwerveModuleState`          | `org.wpilib.math.kinematics.SwerveModuleVelocity` |
    | `edu.wpi.first.wpilibj2.command.Command` / `SubsystemBase` | `org.wpilib.command2.Command` / `SubsystemBase`   |
    | `edu.wpi.first.wpilibj2.command.button.Trigger`            | `org.wpilib.command2.button.Trigger`              |
    | (new) Commands v3                                          | `org.wpilib.command3.*`                           |
* **New projects use the `first.robot` package**, with `first/Main.java` starting `first.robot.Robot`. The examples in these docs use `first.robot` too.
* **Java 25** is required (it ships with the WPILib 2027 installer).
* **Vendor libraries:** use the 2027 builds of the vendor libraries YAMS is built against: Phoenix 6 `26.70` alpha, REVLib 2027 and ThriftyLib 2027. Some vendor constructors now take a CAN bus, for example `new TalonFX(1, new CANBus(CANPort.CAN_S0))` and `new SparkMax(CANPorts.fromBusId(1), 2, MotorType.kBrushless)`.
* **C++:** WPILib 2027 uses `wpi::` namespaces (for example `wpi::units::turn_t`, `wpi::math::Rotation2d`, `wpi::cmd::Trigger`) instead of `frc::`, `frc2::` and `units::`. YAMS C++ include paths (`yams/mechanisms/positional/Arm.hpp`, `yams/motorcontrollers/local/SparkWrapper.hpp`, ...) and the `yams::` namespaces did not change.

## Install the 2027 YAMS vendordep

The 2026 vendordep (`https://yet-another-software-suite.github.io/YAMS/yams.json`) does not work with WPILib 2027. Remove it from your project's `vendordeps` folder, then install **exactly one** of the 2027 vendordeps, matching the command framework your robot uses:

| Command framework                   | Vendordep name                             | Vendordep URL                                      |
| ----------------------------------- | ------------------------------------------ | -------------------------------------------------- |
| Commands v2 (`org.wpilib.command2`) | Yet Another Mechanism System (Commands v2) | `https://cdn.yassrobotics.com/yams_commands2.json` |
| Commands v3 (`org.wpilib.command3`) | Yet Another Mechanism System (Commands v3) | `https://cdn.yassrobotics.com/yams_commands3.json` |

The two vendordeps conflict with each other, so a project can only have one of them. The Commands v3 vendordep is Java only, because WPILib has no C++ Commands v3; C++ projects use the Commands v2 vendordep. See [Installation](../#installation) for the online and offline install steps.

YAMS maven artifacts are served from `https://cdn.yassrobotics.com/`. If your `build.gradle` or a vendordep JSON still points at the old `yet-another-software-suite.github.io` maven URL, replace it.

## YAMS Java package split

YAMS's Java API is now split into three package trees:

* **`yams.core`**: mechanism and motor controller state, physics simulation, telemetry, configs, wrappers, gearing, math, units and exceptions. It has no dependency on a command framework.
* **`yams.commands2`**: the classes that bind to WPILib Commands v2 (`Subsystem`, `Command`, `Trigger`). Shipped in the Commands v2 vendordep.
* **`yams.commands3`**: the same classes bound to WPILib Commands v3 (`Mechanism`, `Command`, `Trigger` from `org.wpilib.command3`). Shipped in the Commands v3 vendordep.

Robot code imports the mechanisms, `SmartMotorControllerConfig`, `SwerveDriveConfig`, `SwerveDrive` and `SwerveInputStream` from `yams.commands2` (or `yams.commands3`). Everything else comes from `yams.core`. Do not construct the `yams.core` versions of the command-bound classes directly.

| Old package (2026)                                                                                                | New package for robot code                                                                   |
| ----------------------------------------------------------------------------------------------------------------- | -------------------------------------------------------------------------------------------- |
| `yams.mechanisms.positional.Arm`                                                                                  | `yams.commands2.mechanisms.Arm`                                                              |
| `yams.mechanisms.positional.Elevator`                                                                             | `yams.commands2.mechanisms.Elevator`                                                         |
| `yams.mechanisms.positional.Pivot`                                                                                | `yams.commands2.mechanisms.Pivot`                                                            |
| `yams.mechanisms.positional.DifferentialMechanism`                                                                | `yams.commands2.mechanisms.DifferentialMechanism`                                            |
| `yams.mechanisms.positional.DoubleJointedArm`                                                                     | `yams.commands2.mechanisms.DoubleJointedArm`                                                 |
| `yams.mechanisms.velocity.FlyWheel`                                                                               | `yams.commands2.mechanisms.FlyWheel`                                                         |
| `yams.mechanisms.swerve.SwerveDrive`                                                                              | `yams.commands2.swerve.SwerveDrive`                                                          |
| `yams.mechanisms.swerve.utility.SwerveInputStream`                                                                | `yams.commands2.swerve.SwerveInputStream`                                                    |
| `yams.motorcontrollers.SmartMotorControllerConfig`                                                                | `yams.commands2.config.SmartMotorControllerConfig`                                           |
| `yams.mechanisms.config.SwerveDriveConfig`                                                                        | `yams.commands2.config.SwerveDriveConfig`                                                    |
| `yams.mechanisms.config.ArmConfig`, `ElevatorConfig`, `PivotConfig`, `FlyWheelConfig`, `SwerveModuleConfig`, etc. | unchanged, now under `yams.core.mechanisms.config`                                           |
| `yams.mechanisms.swerve.SwerveModule`, `yams.mechanisms.SmartMechanism`                                           | `yams.core.mechanisms.swerve.SwerveModule`, `yams.core.mechanisms.SmartMechanism`            |
| `yams.motorcontrollers.SmartMotorController`                                                                      | `yams.core.motorcontrollers.SmartMotorController`                                            |
| `yams.motorcontrollers.local.SparkWrapper`, `yams.motorcontrollers.remote.TalonFX*Wrapper`                        | unchanged, now under `yams.core.motorcontrollers.*`                                          |
| `SmartMotorControllerConfig.ControlMode`, `SmartMotorControllerConfig.MotorMode`                                  | `yams.core.motorcontrollers.enums.ControlMode`, `yams.core.motorcontrollers.enums.MotorMode` |
| `SmartMotorControllerConfig.TelemetryVerbosity`                                                                   | `yams.core.telemetry.enums.TelemetryVerbosity`                                               |
| `yams.exceptions.*`, `yams.gearing.*`, `yams.math.*`, `yams.units.*`, `yams.telemetry.*`                          | unchanged, now under `yams.core.*`                                                           |

For Commands v3, use the same class names under `yams.commands3` (for example `yams.commands3.mechanisms.Arm` and `yams.commands3.config.SmartMotorControllerConfig`).

For most robot code, migration is an import change:

```java
// Before (2026)
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import yams.mechanisms.config.ArmConfig;
import yams.mechanisms.positional.Arm;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;
```

```java
// After (2027, Commands v2)
import org.wpilib.command2.SubsystemBase;
import yams.commands2.config.SmartMotorControllerConfig;
import yams.commands2.mechanisms.Arm;
import yams.core.mechanisms.config.ArmConfig;
import yams.core.motorcontrollers.SmartMotorController;
import yams.core.motorcontrollers.local.SparkWrapper;
import yams.core.telemetry.enums.TelemetryVerbosity;
```

Method names, constructor argument order and config builder methods are the same in `yams.commands2` and `yams.commands3`. With Commands v3, the class that owns the mechanism implements `org.wpilib.command3.Mechanism` instead of extending `SubsystemBase`, and passes itself to `new SmartMotorControllerConfig(this)` the same way. See `examples/commands3/simple_robot-2027` in the YAMS repository for a complete Commands v3 robot.

## Renamed and changed APIs

### Mechanism constructors now take the `SmartMotorController` directly

Every positional/velocity mechanism (`Arm`, `Elevator`, `Pivot`, `FlyWheel`, ...) used to take a single-argument constructor, with the `SmartMotorController` attached to the config via `.withSmartMotorController(...)`. That setter is gone. Pass the `SmartMotorController` as a second constructor argument instead.

```java
// Before (2026)
SmartMotorController smc = new TalonFXWrapper(new TalonFX(1), DCMotor.getKrakenX60(1), motorConfig);
FlyWheelConfig config = new FlyWheelConfig()
    .withDiameter(Inches.of(4))
    .withSmartMotorController(smc);
FlyWheel flywheel = new FlyWheel(config);
```

```java
// After (2027)
SmartMotorController smc = new TalonFXWrapper(new TalonFX(1, new CANBus(CANPort.CAN_S0)), DCMotor.getKrakenX60(1), motorConfig);
FlyWheelConfig config = new FlyWheelConfig()
    .withDiameter(Inches.of(4));
FlyWheel flywheel = new FlyWheel(config, smc);
```

This applies identically to `Arm`, `Elevator`, and `Pivot`: swap `new Mechanism(config.withSmartMotorController(smc))` for `new Mechanism(config, smc)` everywhere.

### Moment of inertia moved to `SmartMotorControllerConfig`

`withMass(Mass)` was removed from `ArmConfig`, `ElevatorConfig`, and `FlyWheelConfig`. Configure the simulated moment of inertia on the `SmartMotorControllerConfig` instead, either from a length + mass estimate (the same `SingleJointedArmSim.estimateMOI(...)` calculation used previously) or a known MOI value straight from CAD.

```java
// Before (2026)
FlyWheelConfig config = new FlyWheelConfig()
    .withDiameter(Inches.of(4))
    .withMass(Pounds.of(2));
```

```java
// After (2027)
SmartMotorControllerConfig motorConfig = new SmartMotorControllerConfig(this)
    // ...
    .withMomentOfInertia(Inches.of(4), Pounds.of(2)); // length, weight: same estimateMOI() as before

FlyWheelConfig config = new FlyWheelConfig()
    .withDiameter(Inches.of(4));
```

For an `Arm`, use the arm's length in place of diameter: `.withMomentOfInertia(Inches.of(30), Pounds.of(8))`. If you already know the MOI (e.g. from CAD), skip the estimate entirely with `.withMomentOfInertia(MomentOfInertia)`.

### Starting position moved to `SmartMotorControllerConfig`

`withStartingPosition(Angle)` (and the `Distance` overload on `ElevatorConfig`) was removed from the mechanism configs. Set it on `SmartMotorControllerConfig` instead:

```java
// Before (2026)
ArmConfig config = new ArmConfig()
    .withHardLimits(Degrees.of(-180), Degrees.of(180))
    .withStartingPosition(Degrees.of(-180));
```

```java
// After (2027)
SmartMotorControllerConfig motorConfig = new SmartMotorControllerConfig(this)
    // ...
    .withStartingPosition(Degrees.of(-180));

ArmConfig config = new ArmConfig()
    .withHardLimits(Degrees.of(-180), Degrees.of(180));
```

### `isNear(...)` Trigger factory renamed to `near(...)`

In Java, the Trigger factory `isNear(...)` was renamed to `near(...)`, matching the other Trigger factories (`gte()`, `lte()`, `between()`, `max()`, `min()`). The boolean check with the same arguments (for use outside a `Command`/`Trigger`, for example in a sensor supplier) is still called `isNear(...)`.

```java
// Before (2026)
arm.isNear(Degrees.of(80), Degrees.of(2)).onTrue(indexer.run());

// After (2027)
arm.near(Degrees.of(80), Degrees.of(2)).onTrue(indexer.run());

// Unchanged: boolean check
boolean atTarget = arm.isNear(Degrees.of(80), Degrees.of(2));
```

The C++ API still names the Trigger factory `IsNear(...)`.

### Live tuning registration

The "Live Tuning" command is registered automatically against the config's `Subsystem` (or Commands v3 `Mechanism`) when the motor controller sets up its telemetry with `TelemetryVerbosity.HIGH`, which YAMS mechanisms do for you. If you set up telemetry yourself with `setupTelemetry(NetworkTable, NetworkTable)`, call `setupLiveTuning()` on the config afterwards. Before the telemetry is set up the call does nothing:

```java
SmartMotorControllerConfig motorConfig = new SmartMotorControllerConfig(this)
    .withTelemetry("ArmMotor", TelemetryVerbosity.HIGH);
SmartMotorController motor = new TalonFXWrapper(armMotor, DCMotor.getKrakenX60(1), motorConfig);

motor.setupTelemetry(telemetryTable, tuningTable);
motorConfig.setupLiveTuning();
```

Tunable fields are in degrees and RPM (meters and meters per second for linear mechanisms), while the read-only fields next to them are in rotations and rotations per second.

See [Live tuning](../how-to-tuning/live-tuning/) for how to use it.

### `withIdleMode` renamed to `withZeroPower`

`SmartMotorControllerConfig.withIdleMode(MotorMode)` is now `withZeroPower(MotorMode)` (`WithZeroPower` in C++). `MotorMode` itself moved to `yams.core.motorcontrollers.enums.MotorMode`.

```java
// Before (2026)
.withIdleMode(MotorMode.BRAKE)

// After (2027)
.withZeroPower(MotorMode.BRAKE)
```

### External encoder zero offset getter renamed

The setter `withExternalEncoderZeroOffset(...)` is unchanged. Its getter `SmartMotorControllerConfig.getZeroOffset()` was renamed to `getExternalEncoderZeroOffset()` (`GetExternalEncoderZeroOffset()` in C++). The TalonFX wrapper no longer applies this offset to the rotor (internal) encoder; it only applies to the external encoder.

### Encoder getters on `SmartMotorController`

The external encoder getters were renamed, and new getters read only the motor's relative (internal) encoder:

| 2026                           | 2027 (Java)                             | 2027 (C++)                              |
| ------------------------------ | --------------------------------------- | --------------------------------------- |
| `getExternalEncoderPosition()` | `getExternalEncoderMechanismPosition()` | `GetExternalEncoderMechanismPosition()` |
| `getExternalEncoderVelocity()` | `getExternalEncoderMechanismVelocity()` | `GetExternalEncoderMechanismVelocity()` |
| (new)                          | `getRelativeMechanismPosition()`        | `GetRelativeMechanismPosition()`        |
| (new)                          | `getRelativeMechanismVelocity()`        | `GetRelativeMechanismVelocity()`        |

`getMechanismPosition()` and `getMechanismVelocity()` are unchanged: they read the external encoder when `withUseExternalFeedbackEncoder(true)` is set, otherwise the relative encoder. The external encoder getters return an empty `Optional` when no external encoder is attached.

### External encoder discontinuity point replaced by wrapping

`withExternalEncoderDiscontinuityPoint(...)` is removed. The range an absolute encoder reads now comes from [`withWrapping(min, max)`](../reference/editor/config/encoders.md#withwrapping), or from [`withContinuousWrapping(min, max)`](../reference/editor/config/closed-loop-control.md#withcontinuouswrapping), which also makes the closed loop take the shortest path:

```java
// Before
.withExternalEncoderDiscontinuityPoint(Rotations.of(0.5))

// After: the encoder reads [-0.5, 0.5) rotations
.withWrapping(Rotations.of(-0.5), Rotations.of(0.5))
```

`getContinuousWrapping()` and `getContinuousWrappingMin()` are now `getWrappingMax()` and `getWrappingMin()`, with `isWrappingContinuous()` (`GetWrappingMax`, `GetWrappingMin`, `IsWrappingContinuous` in C++). Continuous wrapping and soft limits cannot be used together, but `withWrapping` and soft limits can.

### Thrifty Nova closed loop

A PID, and a trapezoidal position profile with its feedforward, now run on the Nova itself. An LQR, an exponential profile, a velocity trapezoidal profile, a feedforward without a trapezoidal profile, an absolute encoder closing the loop over a range other than \[0, 1) without continuous wrapping, and continuous wrapping of an absolute encoder geared to the mechanism still run on the SystemCore. When the Nova runs the loop, only `SLOT_0` and `SLOT_1` exist, soft limits need both limits, and the temperature cutoff, closed loop maximum voltage, feedback synchronization threshold and control period are rejected. See [Where the loop runs](../reference/editor/config/closed-loop-control.md#where-the-loop-runs).

### FIRST A301 (new)

`A301Wrapper` (`yams.core.motorcontrollers.local`, C++ `yams/motorcontrollers/local/A301Wrapper.hpp`) drives the FIRST A301 smart motor through REVLib 2027's `com.revrobotics.spark.A301`. It takes no `DCMotor`: `new A301Wrapper(new A301(CANPorts.fromBusId(1), 20), config)`. Its closed loop always runs on the SystemCore, `withGearing` includes the A301's own gearbox, and it has no current limits, ramp rates, voltage compensation, vendor config or hardware followers. The [feature matrix](../reference/editor/feature-matrix.md#first-a301-a301wrapper) lists everything it supports.

### EasyCRT

`EasyCRTConfig` is gone. `EasyCRT` now takes the three tooth counts and the two readings, and works out its own tolerance:

```java
// Before (2026)
EasyCRTConfig config = new EasyCRTConfig(encoderA::get, encoderB::get)
    .withCommonDriveGear(1.0, 100, 19, 21)
    .withAbsoluteEncoderOffsets(offsetA, offsetB)
    .withMechanismRange(Rotations.of(-1.5), Rotations.of(1.5))
    .withMatchTolerance(Rotations.of(0.006));
Optional<Angle> angle = new EasyCRT(config).getAngleOptional();

// After (2027)
Optional<Angle> angle = new EasyCRT(100, 19, 21).solve(encoderA.get(), encoderB.get());
```

* Offsets and inversion are no longer EasyCRT settings. Set them on the encoders, or adjust the readings before passing them in. The old offsets were added to the raw reading.
* `solve(a, b)` assumes travel centered on zero. Use `solve(a, b, min, max)` otherwise.
* The solved angle is the angle of the gear both pinions mesh with. If the mechanism is geared to that gear (the old `commonRatio`), divide the result by the ratio.
* The status, error and iteration getters and the gear recommender are removed. The [EasyCRT](../understanding/easycrt.md) page shows how to choose gears and check your margin yourself.
* C++ changed the same way: `EasyCRT{100, 19, 21}.Solve(a, b)`.

### Swerve drive (Java)

* `SwerveDriveConfig.withGyro(...)` now takes a `Supplier<Rotation3d>` (the robot's attitude) instead of a `Supplier<Angle>`. Pass the gyro's full rotation, for example `.withGyro(pigeon::getRotation3d)`.
* `getGyroAngle()` was removed from `SwerveDriveConfig` and `SwerveDrive`. Use `getGyroRotation3d()` and take the heading from it, for example `drive.getGyroRotation3d().toRotation2d()`. The heading wraps at +/-180 degrees.
* If the gyro is mounted so the robot turning shows up as its roll or pitch, set `.withGyroHeadingAxis(GyroAxis.ROLL)` or `.withGyroHeadingAxis(GyroAxis.PITCH)` (`yams.core.mechanisms.config.enums.GyroAxis`, default `YAW`).
* Swerve APIs use WPILib 2027's `ChassisVelocities` and `SwerveModuleVelocity` instead of `ChassisSpeeds` and `SwerveModuleState`, for example `drive.setRobotRelativeChassisSpeeds(ChassisVelocities)`.
* `lockPose()` now turns every module toward the robot center (the X pattern) and holds it there.
* `SwerveInputStream` can publish telemetry and live tuning values with `.withTelemetry(name, verbosity)`. See [Swerve Input Stream](../reference/swerve-input-stream/).

The C++ swerve API still uses `GetGyroAngle()` and has no `GyroAxis` yet.

## Quick checklist

* [ ] Install WPILib 2027 (`2027_alpha7`) and the 2027 builds of your vendor libraries (Phoenix 6, REVLib, ThriftyLib).
* [ ] Replace every `edu.wpi.first.*` import with its `org.wpilib.*` equivalent, and `ChassisSpeeds`/`SwerveModuleState` with `ChassisVelocities`/`SwerveModuleVelocity`.
* [ ] Remove the 2026 `yams.json` vendordep and install exactly one of `https://cdn.yassrobotics.com/yams_commands2.json` or `https://cdn.yassrobotics.com/yams_commands3.json`.
* [ ] Update YAMS imports: mechanisms, `SmartMotorControllerConfig`, `SwerveDriveConfig`, `SwerveDrive` and `SwerveInputStream` from `yams.commands2` (or `yams.commands3`), everything else from `yams.core`.
* [ ] Import `ControlMode`/`MotorMode` from `yams.core.motorcontrollers.enums` and `TelemetryVerbosity` from `yams.core.telemetry.enums`.
* [ ] Replace `config.withSmartMotorController(smc)` + `new Mechanism(config)` with `new Mechanism(config, smc)`.
* [ ] Move every `.withMass(Mass)` call from a mechanism config to `.withMomentOfInertia(Distance, Mass)` (or `.withMomentOfInertia(MomentOfInertia)`) on that mechanism's `SmartMotorControllerConfig`.
* [ ] Move every `.withStartingPosition(...)` call from a mechanism config to the corresponding `SmartMotorControllerConfig`.
* [ ] Rename Trigger factory calls `isNear(...)` to `near(...)` (Java). Keep `isNear(...)` where you need a `boolean`.
* [ ] Rename `withIdleMode(...)` to `withZeroPower(...)`.
* [ ] Rename `getZeroOffset()` to `getExternalEncoderZeroOffset()`, and `getExternalEncoderPosition()`/`getExternalEncoderVelocity()` to `getExternalEncoderMechanismPosition()`/`getExternalEncoderMechanismVelocity()`.
* [ ] If you set up motor controller telemetry yourself, call `config.setupLiveTuning()` after `setupTelemetry(...)` to get the "Live Tuning" command.
* [ ] Replace `withExternalEncoderDiscontinuityPoint(...)` with `withWrapping(...)`, or `withContinuousWrapping(...)` where the closed loop should take the shortest path.
* [ ] Thrifty Nova: use only `SLOT_0` and `SLOT_1` when the Nova runs the loop.
* [ ] Replace `EasyCRTConfig` with `new EasyCRT(sharedGearTeeth, encoderATeeth, encoderBTeeth)` and `solve(encoderA, encoderB)`, and move offsets and inversion onto the encoders.
* [ ] Swerve (Java): pass a `Supplier<Rotation3d>` to `withGyro(...)`, replace `getGyroAngle()` with `getGyroRotation3d()`, and add `withGyroHeadingAxis(...)` if your gyro is not mounted flat.
* [ ] Double-check any hard-limit setter names: `withHardLimits(...)` (plural) is correct on `ArmConfig`/`ElevatorConfig`/`PivotConfig`; there has never been a singular `withHardLimit(...)`.
