# Migrating to YAMS 2027

YAMS 2027 is built on WPILib 2027 (`2027_alpha7`) and runs on the SystemCore. Migrating a 2026
robot project means updating WPILib and the vendor libraries, installing a new YAMS vendordep,
changing imports for the package split, and fixing a handful of renamed APIs. Each step is
covered below, with a [checklist](#checklist) at the end.

This guide takes a 2026 project to the latest YAMS release. Already on YAMS 2027? Update the
vendordep to the latest release; the [changelog](https://yams.yassrobotics.com/changelog) lists
what changed in each one.

## WPILib 2027

- WPILib Java packages are now `org.wpilib.*` (`org.wpilib.units.Units`,
  `org.wpilib.math.system.DCMotor`, `org.wpilib.command2.*`, `org.wpilib.command3.*`).
  `edu.wpi.first.*` no longer exists.
- Several WPILib types were renamed. The ones that show up in YAMS signatures are
  `ChassisSpeeds`, now `ChassisVelocities`, and `SwerveModuleState`, now `SwerveModuleVelocity`.
- In C++, WPILib uses `wpi::` namespaces (`wpi::units::turn_t`, `wpi::math::Rotation2d`,
  `wpi::cmd::Trigger`) instead of `frc::`, `frc2::` and `units::`. YAMS C++ include paths and the
  `yams::` namespaces did not change.
- Java 25 is required.
- Use the 2027 builds of the vendor libraries: Phoenix 6 `26.70` alpha, REVLib 2027 and
  ThriftyLib 2027. Vendor device constructors now take a CAN bus, for example
  `new TalonFX(1, new CANBus(CANPort.CAN_S0))` and
  `new SparkMax(CANPort.CAN_S0, 2, MotorType.kBrushless)`.
- Flash SPARK MAX and SPARK Flex firmware 27.0.0 to match REVLib 2027. It changes how mechanisms
  run, so check their gains: followers no longer run about 1% slower than their leader, nor apply
  voltage compensation twice; kS no longer turns the motor at a velocity setpoint of 0; and a SPARK
  MAX closing its loop on an absolute or alternate encoder keeps using it after a reboot.

## Vendordeps

There are now two YAMS vendordeps, one per WPILib command framework. Remove the 2026 `yams.json`
from your `vendordeps` folder and install **exactly one** of these (they conflict with each
other):

| Command framework | Vendordep URL |
| ----------------- | ------------- |
| Commands v2 | `https://cdn.yassrobotics.com/yams_commands2.json` |
| Commands v3 | `https://cdn.yassrobotics.com/yams_commands3.json` |

The Commands v3 vendordep is Java only, because WPILib has no C++ Commands v3. C++ projects use
the Commands v2 vendordep. Maven artifacts are served from `https://cdn.yassrobotics.com/`, so
replace any `yet-another-software-suite.github.io` maven or vendordep URL in your project.

For offline installs, download `<version>-YAMS-Offline.zip` from the GitHub release and extract
it into the WPILib `2027_alpha7` home folder (`C:\Users\Public\wpilib\2027_alpha7` on Windows,
`~/wpilib/2027_alpha7` on macOS and Linux).

## The core/commands2/commands3 split

YAMS's Java API is now split into three package trees:

- **`yams.core`**: mechanism and motor controller state, physics simulation, and telemetry. Has
  no dependency on WPILib's command-based framework (no `Subsystem`, no `Command`, no `Trigger`).
- **`yams.commands2`**: Subsystem-bound extensions of the `yams.core` classes for WPILib Commands
  v2. This is where `Command` and `Trigger` factories live (`setAngle()`, `runTo()`, `near()`,
  `max()`, and so on).
- **`yams.commands3`**: the same extensions for WPILib Commands v3, bound to a v3 `Mechanism`
  instead of a `Subsystem`.

If you write command-based robot code (almost everyone), **you should import from
`yams.commands2` or `yams.commands3`**, not `yams.core`. The `yams.core` classes exist so the
library's state and physics logic can be reused without pulling in the command framework; they
are not meant to be constructed directly by robot code.

### Package mapping

Old flat packages became `yams.core.*`, and a matching `yams.commands2.*` class was added
wherever `Subsystem`/`Command`/`Trigger` support is needed. For Commands v3, replace
`yams.commands2` with `yams.commands3` in the table:

| Old package (pre-split)            | New package for robot code            |
| ----------------------------------- | -------------------------------------- |
| `yams.mechanisms.positional.Arm`    | `yams.commands2.mechanisms.Arm`        |
| `yams.mechanisms.positional.Elevator` | `yams.commands2.mechanisms.Elevator` |
| `yams.mechanisms.positional.Pivot`  | `yams.commands2.mechanisms.Pivot`      |
| `yams.mechanisms.positional.DifferentialMechanism` | `yams.commands2.mechanisms.DifferentialMechanism` |
| `yams.mechanisms.positional.DoubleJointedArm` | `yams.commands2.mechanisms.DoubleJointedArm` |
| `yams.mechanisms.velocity.FlyWheel` | `yams.commands2.mechanisms.FlyWheel`   |
| `yams.mechanisms.swerve.SwerveDrive` | `yams.commands2.swerve.SwerveDrive`   |
| `yams.mechanisms.swerve.utility.SwerveInputStream` | `yams.commands2.swerve.SwerveInputStream` |
| `yams.motorcontrollers.SmartMotorControllerConfig` | `yams.commands2.config.SmartMotorControllerConfig` |
| `yams.mechanisms.config.SwerveDriveConfig` | `yams.commands2.config.SwerveDriveConfig` |
| `yams.mechanisms.config.ArmConfig`, `ElevatorConfig`, `PivotConfig`, `FlyWheelConfig`, etc. | unchanged, now under `yams.core.mechanisms.config` |
| `yams.motorcontrollers.local.SparkWrapper`, `yams.motorcontrollers.remote.TalonFX*Wrapper` | unchanged, now under `yams.core.motorcontrollers.*` |
| `yams.exceptions.*`, `yams.gearing.*`, `yams.math.*`, `yams.units.*`, `yams.telemetry.*` | unchanged, now under `yams.core.*` |
| `SmartMotorControllerConfig.MotorMode`, `SmartMotorControllerConfig.ControlMode` | `yams.core.motorcontrollers.enums.MotorMode`, `ControlMode` |
| `SmartMotorControllerConfig.TelemetryVerbosity` | `yams.core.telemetry.enums.TelemetryVerbosity` |

Mechanism-specific configs (`ArmConfig`, `ElevatorConfig`, `PivotConfig`, `FlyWheelConfig`,
`DifferentialMechanismConfig`, `SwerveModuleConfig`) do not need a Subsystem, so they only exist
under `yams.core`. Only `SmartMotorControllerConfig` and `SwerveDriveConfig` need the Subsystem
binding and have a `yams.commands2.config` counterpart.

### What actually changes in your code

For most robot code, the split is an import change:

```java
// Before
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.mechanisms.positional.Arm;

// After
import yams.commands2.config.SmartMotorControllerConfig;
import yams.commands2.mechanisms.Arm;
```

The config builder methods keep their names, apart from the renames listed under
[Renamed and removed APIs](#renamed-and-removed-apis):

```java
SmartMotorControllerConfig motorConfig = new SmartMotorControllerConfig(this)
    .withClosedLoopController(4, 0, 0)
    .withSoftLimits(Degrees.of(-30), Degrees.of(100))
    .withTelemetry("ArmMotor", TelemetryVerbosity.HIGH);

SmartMotorController motor =
    new TalonFXSWrapper(new TalonFXS(1, new CANBus(CANPort.CAN_S0)), DCMotor.getNEO(1), motorConfig);
ArmConfig config = new ArmConfig().withLength(Meters.of(0.135));
Arm arm = new Arm(config, motor);
```

## Mechanism configs

Anything on `ArmConfig`, `ElevatorConfig`, `PivotConfig` and `FlyWheelConfig` that duplicated a
`SmartMotorControllerConfig` setting moved to the motor config:

- Mechanisms take the `SmartMotorController` as a second constructor argument
  (`new Arm(config, motor)`). `withSmartMotorController(...)` is gone.
- `withMass(...)` is replaced by `SmartMotorControllerConfig.withMomentOfInertia(...)`, either
  `(Distance, Mass)` for an estimate or a `MomentOfInertia` from CAD.
- `withStartingPosition(...)` moved to `SmartMotorControllerConfig`.
- Set the moment of inertia once. `withMomentOfInertia(...)`, `withWheelMomentOfInertia(...)` and
  the arm or flywheel `withExponentialProfile(Voltage, DCMotor, MomentOfInertia)` all set it, and
  setting it a second time throws a `SmartMotorControllerConfigurationException`, since the
  simulation only reads the first value.
- `withRelativePosition(Transform3d)` on a `MechanismPositionConfig` places the mechanism in 3D,
  turned the way it is mounted, and `withParent(Supplier<Pose3d>)` mounts it on another mechanism,
  such as an arm on `elevator::getPose3d`.

## Renamed and removed APIs

| 2026 | 2027 |
| ---- | ---- |
| `withIdleMode(MotorMode)` | `withZeroPower(MotorMode)` (C++ `WithZeroPower`) |
| `SmartMotorControllerConfig.getZeroOffset()` | `getExternalEncoderZeroOffset()` (the `withExternalEncoderZeroOffset(...)` setter is unchanged) |
| `SmartMotorController.getExternalEncoderPosition()` | `getExternalEncoderMechanismPosition()` |
| `SmartMotorController.getExternalEncoderVelocity()` | `getExternalEncoderMechanismVelocity()` |
| `SwerveDrive.getGyroAngle()`, `SwerveDriveConfig.getGyroAngle()` (Java) | `getGyroRotation3d()` |
| `SwerveDriveConfig.withGyro(Supplier<Angle>)` (Java) | `withRotation(Supplier<Rotation3d>)`, for example `withRotation(gyro::getRotation3d)`. `withGyro(Supplier<Rotation3d>)` does the same (C++ `WithRotation` and `WithGyro`) |
| `isNear(...)` Trigger factory (Java) | `near(...)`, see below |
| `withExternalEncoderDiscontinuityPoint(...)` | `withWrapping(min, max)`, see [Wrapping](#wrapping) |
| `getContinuousWrapping()`, `getContinuousWrappingMin()` | `getWrappingMax()`, `getWrappingMin()`, with `isWrappingContinuous()` |
| `getRelativeMechanismPosition()` on a mechanism | `getPose3d()` (C++ `GetPose3d()`): the mechanism's `Pose3d` relative to the robot, for an AdvantageScope robot model |
| `SwerveModule.lock(Rotation2d)` | `setAzimuthAngle(Rotation2d)`, which turns the wheel without driving it |
| `SwerveDriveConfig.withGyroOffset(Angle)`, `getGyroOffset()` (C++ `WithGyroOffset`, `GetGyroOffset`) | removed. The pose estimator keeps the offset between the gyro and the pose: set the robot's heading with the starting pose (`withStartingPose`), `resetOdometry(Pose2d)` or `zeroOdometryHeading()`. See [Field relative driving uses the odometry rotation](#field-relative-driving-uses-the-odometry-rotation) |
| Swerve drive telemetry `gyro` (degrees, `double`) | `gyro` (`struct:Rotation2d`), the gyro's rotation, with `withGyro()` (C++ `WithGyro()`). New beside it: `rotation` (`struct:Rotation2d`), the robot's rotation from odometry, used for the module states, with `withRotation()` (C++ `WithRotation()`). Both are on from `LOW` up |

New alongside the renames:

- `SmartMotorController.getRelativeMechanismPosition()` / `getRelativeMechanismVelocity()` read
  only the relative encoder. `getMechanismPosition()` / `getMechanismVelocity()` still prefer the
  external encoder when one is configured.
- `TalonFXWrapper` and `TalonFXSWrapper` no longer set a rotor offset from the zero offset.
- `NovaWrapper` (`yams.core.motorcontrollers.local.NovaWrapper`) supports the Thrifty Nova. See
  [Thrifty Nova closed loop](#thrifty-nova-closed-loop).
- `withTemperatureCutoff(...)` is only accepted where the closed loop runs on the SystemCore, which
  checks it: an LQR or ADRC on any motor controller, or an exponential profile on a SPARK.

### Wrapping

`withWrapping(min, max)` sets the range an absolute external encoder reads, such as
[-0.5, 0.5) or [0, 1) rotations, without changing the closed loop. `withContinuousWrapping(min,
max)` sets the same range and also makes the closed loop take the shortest path to a setpoint,
which can be the other way round. Use `withWrapping` on an arm whose encoder must read across zero,
and `withContinuousWrapping` on a swerve azimuth or turret that can turn all the way round.

```java
// Before
.withExternalEncoderDiscontinuityPoint(Rotations.of(0.5))

// After: the encoder reads [-0.5, 0.5) rotations
.withWrapping(Rotations.of(-0.5), Rotations.of(0.5))
```

- The range is one rotation, and an absolute encoder reads it only within what its vendor accepts;
  anything else is rejected with a `SmartMotorControllerConfigurationException`:
  - CANcoder and CANdi PWM: a maximum from 0 to 1 rotation (their `AbsoluteSensorDiscontinuityPoint`).
  - SPARK absolute encoder: a range from [-1, 0) to [0, 1) (its range offset, the middle of the range).
  - SPARK detached encoder: [-0.5, 0.5) or [0, 1).
  - Thrifty Nova: [0, 1) when the Nova closes its loop on the encoder.
- Without wrapping, a SPARK or Nova absolute encoder reads [0, 1), and a CANcoder or CANdi keeps its
  own range.
- Continuous wrapping and soft limits cannot be used together; wrapping and soft limits can.
- `getContinuousWrapping()` and `getContinuousWrappingMin()` are now `getWrappingMax()` and
  `getWrappingMin()`, and `isWrappingContinuous()` says whether the closed loop wraps. C++ has
  `WithWrapping`, `GetWrappingMax`, `GetWrappingMin` and `IsWrappingContinuous`.

### Thrifty Nova closed loop

A PID, and a trapezoidal position profile with its feedforward, run on the Nova itself, at 1 kHz.
When the Nova runs the loop:

- Only closed loop slots `SLOT_0` and `SLOT_1` exist, and the closed loop tolerance is the slot's
  allowable error.
- Mechanism limits are the Nova's soft limits, and need both an upper and a lower limit.
- `withTemperatureCutoff`, `withClosedLoopControllerMaximumVoltage`,
  `withFeedbackSynchronizationThreshold` and `withClosedLoopControlPeriod` are rejected, since only
  the SystemCore's closed loop applies them.

The Nova only applies a feedforward in a motion profile, so a feedforward without a trapezoidal
profile runs on the SystemCore, with the PID, and a YAMS alert says so. Add
`withTrapezoidalProfile(...)` to run both on the Nova. An LQR, an ADRC, an exponential profile, a
velocity trapezoidal profile, an absolute encoder closing the loop over a range other than [0, 1)
without continuous wrapping, and continuous wrapping of an absolute encoder geared to the mechanism
also run on the SystemCore, where every option works. `getClosedLoopControllerPeriod()` says where
the loop runs: 1 ms on the Nova, or the closed loop control period on the SystemCore. Gains tuned
with the loop in one place need tuning again in the other.

### EasyCRT

`EasyCRTConfig` is gone. `EasyCRT` now takes the three tooth counts and the two readings, and works
out its own tolerance:

```java
// Before
EasyCRTConfig config = new EasyCRTConfig(encoderA::get, encoderB::get)
    .withCommonDriveGear(1.0, 100, 19, 21)
    .withAbsoluteEncoderOffsets(offsetA, offsetB)
    .withMechanismRange(Rotations.of(-1.5), Rotations.of(1.5))
    .withMatchTolerance(Rotations.of(0.006));
Optional<Angle> angle = new EasyCRT(config).getAngleOptional();

// After
Optional<Angle> angle = new EasyCRT(100, 19, 21).solve(encoderA.get(), encoderB.get());
```

- Offsets and inversion are no longer EasyCRT settings. Set them on the encoders, or adjust the
  readings before passing them in. The old offsets were added to the raw reading.
- `solve(a, b)` assumes travel centered on zero. Use `solve(a, b, min, max)` otherwise.
- The solved angle is the angle of the gear both pinions mesh with. If the mechanism is geared to
  that gear (the old `commonRatio`), divide the result by the ratio.
- The status, error and iteration getters and the gear recommender are removed. `EasyCRTGuide.md`
  shows how to choose gears and check your margin yourself.
- C++ changed the same way: `EasyCRT{100, 19, 21}.Solve(a, b)`.

### Swerve gyro (Java)

`getGyroRotation3d()` returns the robot's attitude, and the heading is its yaw, so it wraps at
+/-180 degrees instead of counting continuously:

```java
// Before
new Rotation2d(drive.getGyroAngle())
drive.getGyroAngle().in(Radians)

// After
drive.getGyroRotation3d().toRotation2d()
drive.getGyroRotation3d().getZ()
```

A gyro mounted on its side can be used as the heading source with
`SwerveDriveConfig.withGyroHeadingAxis(GyroAxis)` (`YAW` by default, `ROLL` or `PITCH`). C++ is
unchanged and still uses `GetGyroAngle()`.

### Field relative driving uses the odometry rotation

Field relative driving (`setFieldRelativeChassisSpeeds`), `getFieldRelativeSpeed()`, driving to a
pose and `SwerveInputStream`'s heading, aim and locked-heading control use the robot's rotation from
odometry, `SwerveDrive.getRotation()` (C++ `GetRotation()`), instead of the gyro. It's the heading of
`getPose()`, so vision measurements that correct the pose's heading also correct which way "forward"
is. Without vision, it is the gyro's heading, so nothing changes. If vision's heading is noisy, give
it a larger heading standard deviation in `addVisionMeasurement`.

`resetOdometry(Pose2d)` (C++ `ResetOdometry`) resets only the pose. It no longer changes the gyro
or the simulated gyro, since the pose estimator keeps the offset between the gyro and the pose. For
the same reason `withGyroOffset` is removed: a heading offset belongs in the starting pose or a
`resetOdometry` call. `getGyroRotation3d()` and the `gyro` telemetry keep reading the gyro after a reset, while
`getRotation()` reads the reset pose's heading. `zeroOdometryHeading()` (C++ `ZeroOdometryHeading()`)
turns the pose to face 0 so field relative "forward" is the way the robot faces, and `zeroGyro()`
does the same.

### Trigger factory rename

`isNear(...)` as a Trigger factory was renamed to `near(...)` for consistency with the other
Trigger factories (`gte()`, `lte()`, `between()`, `max()`, `min()`), none of which had an `is`
prefix. The boolean check with the same name (for use outside a `Command`/`Trigger` context) is
still called `isNear(...)`. C++ keeps `IsNear(...)`.

```java
// Before
arm.isNear(Degrees.of(80), Degrees.of(2)).onTrue(indexer.run());

// After
arm.near(Degrees.of(80), Degrees.of(2)).onTrue(indexer.run());
```

## Live Tuning

A "Live Tuning" command is registered automatically, against the Subsystem (Commands v2) or
Mechanism (Commands v3) the config holds, when the motor controller sets up its telemetry with
`TelemetryVerbosity.HIGH` through `setupTelemetry()`, which is what YAMS mechanisms call. If you
set up telemetry yourself with `setupTelemetry(NetworkTable, NetworkTable)`, opt in explicitly.
Call `setupLiveTuning()` after the telemetry is set up; before that the config has no controller
attached and the call does nothing.

```java
SmartMotorControllerConfig motorConfig = new SmartMotorControllerConfig(this)
    .withTelemetry("ArmMotor", TelemetryVerbosity.HIGH);
SmartMotorController motor = new TalonFXWrapper(talon, DCMotor.getKrakenX60(1), motorConfig);

motor.setupTelemetry(telemetryTable, tuningTable);
motorConfig.setupLiveTuning();
```

Tunable fields do not use the same units as the read-only fields next to them:

| Quantity | Read-only fields | Tunable fields |
| -------- | ---------------- | -------------- |
| Angular position (setpoint, soft limits) | Rotations | Degrees |
| Angular velocity (setpoint, profile max velocity) | Rotations per second | RPM |
| Angular acceleration (profile max acceleration) | Rotations per second squared | RPM per second |
| Linear mechanisms (linear closed loop control) | Meters, meters per second | Meters, meters per second |

So a flywheel setpoint of 3000 RPM is entered as `3000`, while `MechanismVelocity` reads `50`.

A `SwerveDrive`'s Live Tuning command is at `NT:/Tuning/<name>/Live Tuning`, like every other
mechanism's, where it was `NT:/Mechanisms/<name>/tuning`. Move a dashboard button that starts it. It
requires the drive, so the default drive command pauses while it runs.

## Checklist

1. Update the project to WPILib 2027 (`2027_alpha7`), Java 25 and the 2027 vendor libraries, and
   flash SPARK MAX and SPARK Flex firmware 27.0.0.
2. Replace `edu.wpi.first.*` imports with `org.wpilib.*`, and pass a CAN bus to vendor device
   constructors.
3. Remove the old `yams.json` vendordep and install the Commands v2 or Commands v3 vendordep from
   `cdn.yassrobotics.com`.
4. Change YAMS imports to `yams.commands2.*` (or `yams.commands3.*`) and `yams.core.*`.
5. Pass the `SmartMotorController` to the mechanism constructor, and move mass and starting
   position to `SmartMotorControllerConfig`.
6. Apply the renames: `withZeroPower`, `getExternalEncoderZeroOffset`,
   `getExternalEncoderMechanismPosition` / `Velocity`, `getGyroRotation3d`, and `near(...)` for
   Triggers.
7. If you set up motor controller telemetry yourself, call `setupLiveTuning()` after
   `setupTelemetry(...)` to get the "Live Tuning" command.
8. Replace `withExternalEncoderDiscontinuityPoint(...)` with `withWrapping(...)`, or
   `withContinuousWrapping(...)` where the closed loop should take the shortest path.
9. Replace a mechanism's `getRelativeMechanismPosition()` with `getPose3d()`, and
   `SwerveModule.lock(...)` with `setAzimuthAngle(...)`.
10. Set each motor config's moment of inertia once.
11. On a Thrifty Nova, give a mechanism with a feedforward a trapezoidal profile to keep its loop on
    the Nova, and use only `SLOT_0` and `SLOT_1` there.
12. Move any dashboard button for the swerve drive's tuning command to
    `NT:/Tuning/<name>/Live Tuning`.

## Why this split

Separating state/physics (`yams.core`) from command-framework bindings (`yams.commands2`,
`yams.commands3`) keeps the mechanism and motor controller logic reusable outside of a command
framework, and makes the supported public surface explicit. A future release will add module
boundaries that prevent `yams.core` from being used directly, so migrating now avoids a breaking
change later.
