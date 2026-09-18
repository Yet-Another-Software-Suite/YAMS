# Migrating to the core/commands2 split

YAMS's Java API is now split into two package trees:

- **`yams.core`**: mechanism and motor controller state, physics simulation, and telemetry. Has
  no dependency on WPILib's command-based framework (no `Subsystem`, no `Command`, no `Trigger`).
- **`yams.commands2`**: Subsystem-bound extensions of the `yams.core` classes. This is where
  `Command` and `Trigger` factories live (`setAngle()`, `runTo()`, `near()`, `max()`, and so on).

If you write command-based robot code (almost everyone), **you should import from
`yams.commands2`**, not `yams.core`. The `yams.core` classes exist so the library's state and
physics logic can be reused without pulling in the command framework; they are not meant to be
constructed directly by robot code.

## Package mapping

Old flat packages became `yams.core.*`, and a matching `yams.commands2.*` class was added
wherever `Subsystem`/`Command`/`Trigger` support is needed:

| Old package (pre-split)            | New package for robot code            |
| ----------------------------------- | -------------------------------------- |
| `yams.mechanisms.positional.Arm`    | `yams.commands2.mechanisms.Arm`        |
| `yams.mechanisms.positional.Elevator` | `yams.commands2.mechanisms.Elevator` |
| `yams.mechanisms.positional.Pivot`  | `yams.commands2.mechanisms.Pivot`      |
| `yams.mechanisms.positional.DifferentialMechanism` | `yams.commands2.mechanisms.DifferentialMechanism` |
| `yams.mechanisms.positional.DoubleJointedArm` | `yams.commands2.mechanisms.DoubleJointedArm` |
| `yams.mechanisms.velocity.FlyWheel` | `yams.commands2.mechanisms.FlyWheel`   |
| `yams.mechanisms.swerve.SwerveDrive` | `yams.commands2.swerve.SwerveDrive`   |
| `yams.motorcontrollers.SmartMotorControllerConfig` | `yams.commands2.config.SmartMotorControllerConfig` |
| `yams.mechanisms.config.SwerveDriveConfig` | `yams.commands2.config.SwerveDriveConfig` |
| `yams.mechanisms.config.ArmConfig`, `ElevatorConfig`, `PivotConfig`, `FlyWheelConfig`, etc. | unchanged, now under `yams.core.mechanisms.config` |
| `yams.motorcontrollers.local.SparkWrapper`, `yams.motorcontrollers.remote.TalonFX*Wrapper` | unchanged, now under `yams.core.motorcontrollers.*` |
| `yams.exceptions.*`, `yams.gearing.*`, `yams.math.*`, `yams.units.*`, `yams.telemetry.*` | unchanged, now under `yams.core.*` |

Mechanism-specific configs (`ArmConfig`, `ElevatorConfig`, `PivotConfig`, `FlyWheelConfig`,
`DifferentialMechanismConfig`, `SwerveModuleConfig`) do not need a Subsystem, so they only exist
under `yams.core`. Only `SmartMotorControllerConfig` and `SwerveDriveConfig` need the Subsystem
binding and have a `yams.commands2.config` counterpart.

## What actually changes in your code

For most robot code, migration is an import change plus one constructor swap:

```java
// Before
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.mechanisms.positional.Arm;

// After
import yams.commands2.config.SmartMotorControllerConfig;
import yams.commands2.mechanisms.Arm;
```

Everything else, method names, constructor argument order, config builder methods, stays the
same:

```java
SmartMotorControllerConfig motorConfig = new SmartMotorControllerConfig(this)
    .withClosedLoopController(4, 0, 0)
    .withSoftLimits(Degrees.of(-30), Degrees.of(100))
    .withTelemetry("ArmMotor", TelemetryVerbosity.HIGH);

SmartMotorController motor = new TalonFXSWrapper(new TalonFXS(1), DCMotor.getNEO(1), motorConfig);
ArmConfig config = new ArmConfig().withLength(Meters.of(0.135));
Arm arm = new Arm(config, motor);
```

## Trigger factory rename

`isNear(...)` as a Trigger factory was renamed to `near(...)` for consistency with the other
Trigger factories (`gte()`, `lte()`, `between()`, `max()`, `min()`), none of which had an `is`
prefix. The boolean check with the same name (for use outside a `Command`/`Trigger` context) is
still called `isNear(...)`.

```java
// Before
arm.isNear(Degrees.of(80), Degrees.of(2)).onTrue(indexer.run());

// After
arm.near(Degrees.of(80), Degrees.of(2)).onTrue(indexer.run());
```

## Live Tuning is now explicit

Previously, enabling tunable telemetry fields automatically registered a "Live Tuning" command
against whatever Subsystem the config held. That is now an explicit opt-in call:

```java
SmartMotorControllerConfig motorConfig = new SmartMotorControllerConfig(this)
    .withTelemetry("ArmMotor", TelemetryVerbosity.HIGH);
motorConfig.setupLiveTuning();
```

## Why this split

Separating state/physics (`yams.core`) from command-framework bindings (`yams.commands2`) keeps
the mechanism and motor controller logic reusable outside of `wpilibNewCommands`, and makes the
supported public surface (`yams.commands2`) explicit. A future release will add module boundaries
that prevent `yams.core` from being used directly, so migrating now avoids a breaking change
later.
