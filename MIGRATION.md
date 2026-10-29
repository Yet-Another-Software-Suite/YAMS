# Migrating to YAMS 2027

YAMS 2027 is built on WPILib 2027 (`2027_alpha7`) and runs on the SystemCore. Migrating a 2026
robot project means updating WPILib and the vendor libraries, installing a new YAMS vendordep,
changing imports for the package split, and fixing a handful of renamed APIs. Each step is
covered below, with a [checklist](#checklist) at the end.

Already on YAMS 2027 v2026.10.09? See
[Updating from v2026.10.09 to v2026.10.15](#updating-from-v20261009-to-v20261015).

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

## Renamed and removed APIs

| 2026 | 2027 |
| ---- | ---- |
| `withIdleMode(MotorMode)` | `withZeroPower(MotorMode)` (C++ `WithZeroPower`) |
| `SmartMotorControllerConfig.getZeroOffset()` | `getExternalEncoderZeroOffset()` (the `withExternalEncoderZeroOffset(...)` setter is unchanged) |
| `SmartMotorController.getExternalEncoderPosition()` | `getExternalEncoderMechanismPosition()` |
| `SmartMotorController.getExternalEncoderVelocity()` | `getExternalEncoderMechanismVelocity()` |
| `SwerveDrive.getGyroAngle()`, `SwerveDriveConfig.getGyroAngle()` (Java) | `getGyroRotation3d()` |
| `SwerveDriveConfig.withGyro(Supplier<Angle>)` (Java) | `withGyro(Supplier<Rotation3d>)`, for example `withGyro(gyro::getRotation3d)` |
| `isNear(...)` Trigger factory (Java) | `near(...)`, see below |

New alongside the renames:

- `SmartMotorController.getRelativeMechanismPosition()` / `getRelativeMechanismVelocity()` read
  only the relative encoder. `getMechanismPosition()` / `getMechanismVelocity()` still prefer the
  external encoder when one is configured.
- `TalonFXWrapper` and `TalonFXSWrapper` no longer set a rotor offset from the zero offset.
- `NovaWrapper` (`yams.core.motorcontrollers.local.NovaWrapper`) supports the Thrifty Nova.

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

## Updating from v2026.10.09 to v2026.10.15

If you already moved to YAMS 2027 with v2026.10.09, these are the changes that can affect your
code or your tuning. The vendordep URLs are the same; update the version in your vendordep (or
reinstall it from `cdn.yassrobotics.com`).

### Changes that can need code or gain changes

- **EasyCRT takes tooth counts and two readings.** `EasyCRTConfig` is removed; see
  [EasyCRT](#easycrt) above. `EasyCRTConfigGuide.md` is now `EasyCRTGuide.md`.
- **An `ArmFeedforward`'s kV and kA are per rotation everywhere.** YAMS documents its feedforward
  gains per rotation, and the Talon and SPARK wrappers already sent them to the motor controller
  that way. Only the closed loop on the SystemCore (such as the Thrifty Nova's) read them per
  radian. It now reads them per rotation too, so the same kV and kA give 2π times less voltage
  there. If you tuned an arm's kV or kA on the SystemCore with v2026.10.09, multiply them by 2π
  (`Rotations.of(1).in(Radians)`) to keep the same behaviour.
- **The temperature cutoff is only accepted where it is checked.** `withTemperatureCutoff(...)` is
  enforced by the closed loop on the SystemCore. A SPARK now rejects it with a trapezoidal profile,
  which runs on the SPARK, and accepts it with an exponential profile or LQR. A TalonFX or TalonFXS
  accepts it with an LQR controller, where it was rejected before.
- **`SwerveModule.lock(Rotation2d)` is now `setAzimuthAngle(Rotation2d)`.** It turns the wheel
  without driving it. `SwerveDrive.setAzimuthAngles(Rotation2d)` turns every module to one angle,
  which azimuth live tuning now uses, so it turns the modules again.
- **The external encoder telemetry is only published when there is an external encoder.**
  `externalencoder/position` and `externalencoder/velocity` keep their names, and hold the
  mechanism position and velocity.

### New

- **`SwerveDrive.driveToPose(pose, translationTolerance, rotationTolerance)`** in Commands v2, as
  Commands v3 already had: it ends once the robot is within the tolerances. `isNear(pose,
  translationTolerance, rotationTolerance)` and `stop()` are on the shared `SwerveDrive` for both.
- **`SwerveInputStream` aim options**, carried over from YAGSL 2026: `withAimLookahead(Time)`,
  `withAimFeedforward(SimpleMotorFeedforward)`, `withAimHeadingOffset(Rotation2d)`,
  `isAimLocked(Angle)` and the `aimLock(Angle)` Trigger, `getTargetVector(Pose2d)` and
  `calculateAngularVelocity(Angle)`. Aiming without them behaves as before.
- **`SmartMechanism.getMeasurementSetpoint()`**: the setpoint as a distance, for linear
  mechanisms.
- **The C++ swerve drive matches the Java one**: the 3D gyro and gyro heading axis, anti-tipping,
  module speed desaturation, the coupling ratio, feedforward forces and the default velocity
  deadband.

## Updating from v2026.10.15

### Changes that can need code changes

- **Flash SPARK MAX and SPARK Flex firmware 27.0.0** to match REVLib 2027. REVLib 2027 no longer
  sets conversion factors, the zero centered duty cycle or the hall sensor averaging, and this
  firmware drops them from the SPARK too; YAMS already works without them. The firmware also changes
  how mechanisms run, so check their gains: followers no longer run about 1% slower than their
  leader, nor apply voltage compensation twice when the leader and follower both have it; kS no
  longer turns the motor at a velocity setpoint of 0; and a SPARK MAX closing its loop on an
  absolute or alternate encoder keeps using it after a reboot, where it could switch sensors in
  brake mode, when inverted, or with a control type other than duty cycle.
- **The `SwerveDrive` tuning command is now `NT:/Tuning/<name>/Live Tuning`**, named `Live Tuning`,
  like every other mechanism's. It was `NT:/Mechanisms/<name>/tuning`. Move a dashboard button or
  anything else that starts it, such as setting `running` to `true`. It now also requires the drive's
  subsystem (Commands v2, C++) or mechanism (Commands v3), so it pauses the default drive command
  while it runs; before, the default drive command kept pointing the wheels back where they were.
- **`getRelativeMechanismPosition()` is replaced by `getPose3d()`** (`GetPose3d()` in C++) on every
  mechanism. It returns the mechanism's `Pose3d` relative to the robot, worked out from its motor's
  position, so it works on the real robot as well as in simulation: an arm's or pivot's axis turned to
  its angle, an elevator's carriage at its height, or a flywheel's axle turned by its position. Publish
  it as a component of an AdvantageScope robot model. `DoubleJointedArm.getUpperPose3d()` gives the
  upper arm's.
- **`MechanismPositionConfig` places the mechanism in 3D.** `withRelativePosition(Transform3d)` also
  sets how the mechanism is turned where it is mounted, and `withParent(Supplier<Pose3d>)` mounts it
  on another mechanism, such as an arm on `elevator::getPose3d`. In C++, `MechanismPositionConfig` is
  now this class (it was an unused, undefined named-setpoint class), and `ArmConfig`, `ElevatorConfig`,
  `PivotConfig` and `FlyWheelConfig` take one with `WithMechanismPositionConfig(...)`.

### Fixes

- **A TalonFX's loosely coupled followers switch closed loop slot with it.** `setClosedLoopSlot`
  on a TalonFX now passes the slot on to the followers from `withLooselyCoupledFollowers(...)`, as
  the TalonFXS, SPARK and Thrifty Nova already did. C++ already did this.
- **A TalonFX or TalonFXS vendor config wins over the device's saved settings.** With
  `withResetPreviousConfig(false)`, the Java wrappers read the settings saved on the device into the
  `TalonFXConfiguration` (or `TalonFXSConfiguration`) given to `withVendorConfig(...)`, which
  replaced the vendor config's settings. A vendor config is now the base, as in C++, and the
  device's settings are only read when there is no vendor config.
- **Simulated SPARKs follow their profile and feedforward.** REVLib's simulation does not run
  MAXMotion, and works out kCos from the setpoint instead of the position, so in simulation a
  SPARK with a trapezoidal profile runs the profile on the SystemCore and sends kV, kA and an
  arm's gravity as an arbitrary feedforward. On the robot it still uses MAXMotion. See
  [wpilibsuite/SystemcoreTesting#425](https://github.com/wpilibsuite/SystemcoreTesting/issues/425).
- **Simulated SPARKs start where they should and report their real voltage.** A simulated SPARK
  started at its starting position divided by the gear ratio, and reported a voltage that was off
  by the gear ratio.
- **Every motor controller reacts the same to the same gains.** The SPARK MAX, SPARK Flex,
  TalonFX, TalonFXS and Thrifty Nova are now tested to give the same voltage for the same kP, kI,
  kD, kS, kV, kA and kG in YAMS units, from every closed loop slot. This also fixed an elevator
  feedforward under velocity control on the SystemCore (the Nova), which left out kV.
- A mechanism iterating its simulation no longer replaces the shared `BatterySim` voltage.
- `ExponentialProfilePIDController.createConstraints(...)` is fixed.
- `DifferentialMechanism.set(...)` mixes its two outputs correctly.
- C++ swerve fixes that Java already had, and C++ swerve azimuth live tuning.
- The vendordep JSONs now require the matching WPILib command library (`command2` or `command3`).

## Why this split

Separating state/physics (`yams.core`) from command-framework bindings (`yams.commands2`,
`yams.commands3`) keeps the mechanism and motor controller logic reusable outside of a command
framework, and makes the supported public surface explicit. A future release will add module
boundaries that prevent `yams.core` from being used directly, so migrating now avoids a breaking
change later.
