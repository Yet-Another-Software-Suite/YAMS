# Overview

The YAMS Java API is split into a framework-independent core and two command framework bindings:

* **`yams.core.*`**: mechanism and motor controller state, physics simulation, telemetry, configs, motor controller wrappers, gearing, math, units and exceptions. It does not depend on a command framework.
* **`yams.commands2.*`**: WPILib Commands v2 bindings. The mechanisms, `SmartMotorControllerConfig`, `SwerveDriveConfig`, `SwerveDrive` and `SwerveInputStream` are bound to a `Subsystem` and return Commands v2 `Command`s and `Trigger`s.
* **`yams.commands3.*`**: WPILib Commands v3 bindings. The same classes, bound to a Commands v3 `Mechanism` and returning Commands v3 `Command`s and `Trigger`s.

Install exactly one YAMS vendordep: `https://cdn.yassrobotics.com/yams_commands2.json` for Commands v2, or `https://cdn.yassrobotics.com/yams_commands3.json` for Commands v3. Robot code imports the mechanisms, `SmartMotorControllerConfig`, `SwerveDriveConfig`, `SwerveDrive` and `SwerveInputStream` from `yams.commands2` or `yams.commands3`, and everything else from `yams.core`. Each mechanism accepts a `SmartMotorController` and a mechanism config built with a fluent builder pattern, and publishes telemetry to NetworkTables.

**Packages**

| Package                                                   | Classes                                                                                                               |
| --------------------------------------------------------- | --------------------------------------------------------------------------------------------------------------------- |
| `yams.commands2.mechanisms` / `yams.commands3.mechanisms` | Arm, Elevator, Pivot, FlyWheel, DoubleJointedArm, DifferentialMechanism                                               |
| `yams.commands2.config` / `yams.commands3.config`         | SmartMotorControllerConfig, SwerveDriveConfig                                                                         |
| `yams.commands2.swerve` / `yams.commands3.swerve`         | SwerveDrive, SwerveInputStream                                                                                        |
| `yams.commands2.telemetry` / `yams.commands3.telemetry`   | SwerveInputStreamTelemetry, SmartMotorControllerCommandRegistry, CommandTunable (Commands v3 only)                    |
| `yams.core.mechanisms.config`                             | ArmConfig, ElevatorConfig, PivotConfig, FlyWheelConfig, DifferentialMechanismConfig, SensorConfig, SwerveModuleConfig |
| `yams.core.mechanisms.config.enums`                       | GyroAxis                                                                                                              |
| `yams.core.mechanisms.swerve`                             | SwerveModule                                                                                                          |
| `yams.core.motorcontrollers`                              | SmartMotorController, YMotors                                                                                         |
| `yams.core.motorcontrollers.local`                        | SparkWrapper, NovaWrapper, A301Wrapper                                                                                |
| `yams.core.motorcontrollers.remote`                       | TalonFXWrapper, TalonFXSWrapper                                                                                       |
| `yams.core.motorcontrollers.enums`                        | ControlMode, MotorMode                                                                                                |
| `yams.core.motorcontrollers.simulation`                   | Sensor, SensorData, BatterySim                                                                                        |
| `yams.core.telemetry`                                     | SmartMotorControllerTelemetryConfig                                                                                   |
| `yams.core.telemetry.enums`                               | TelemetryVerbosity                                                                                                    |
| `yams.core.gearing`                                       | GearBox, Sprocket, MechanismGearing                                                                                   |
| `yams.core.math`                                          | SmartMath, ExponentialProfilePIDController, LQRConfig, LQRController, DerivativeTimeFilter                            |
| `yams.core.units`                                         | YUnits, EasyCRT                                                                                                       |
| `yams.core.exceptions`                                    | Configuration exceptions                                                                                              |

{% hint style="info" %}
The old flat packages (`yams.mechanisms.*`, `yams.motorcontrollers.*`, `yams.gearing.*` and so on) are gone. See `MIGRATION.md` in the YAMS repository for the full package mapping. WPILib 2027 classes are under `org.wpilib.*` (for example `org.wpilib.math.system.DCMotor` and `org.wpilib.units.Units`).
{% endhint %}

## Commands v2 and Commands v3

The configuration and state APIs are the same in both bindings, and the mechanism classes expose the same factory names. The differences are what a mechanism is bound to and which framework's types the factories return:

|                                                                   | Commands v2 (`yams.commands2`)                                           | Commands v3 (`yams.commands3`)                                                                                                         |
| ----------------------------------------------------------------- | ------------------------------------------------------------------------ | -------------------------------------------------------------------------------------------------------------------------------------- |
| Config binding                                                    | `new SmartMotorControllerConfig(subsystem)` / `withSubsystem(Subsystem)` | `new SmartMotorControllerConfig(mechanism)` / `withMechanism(Mechanism)`                                                               |
| Owner getter                                                      | `getSubsystem()`                                                         | `getMechanism()`                                                                                                                       |
| Command factories (`setAngle`, `run`, `runTo`, ...)               | Return `org.wpilib.command2.Command`                                     | Return `org.wpilib.command3.Command`                                                                                                   |
| Trigger factories (`near`, `gte`, `lte`, `between`, `max`, `min`) | Return `org.wpilib.command2.button.Trigger`                              | Return `org.wpilib.command3.Trigger`                                                                                                   |
| Live tuning command                                               | Commands v2 `Command` on the `Subsystem`                                 | Commands v3 `Command` on the `Mechanism`; `SwerveInputStream` and `SwerveDrive` tuning commands are published through `CommandTunable` |

The `isNear(...)` Trigger factory was renamed to `near(...)`. The boolean check `isNear(...)` still exists for use outside a Trigger. Live tuning is registered automatically at `TelemetryVerbosity.HIGH`; at other verbosities call `config.setupLiveTuning()`.
