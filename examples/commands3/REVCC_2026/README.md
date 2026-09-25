# REV ION 2026 StarterBot (YAMS port, Commands v3)

A port of the REV Robotics 2026 ION FRC StarterBot to WPILib 2027, Commands v3 and YAMS.

## Original source

- Repository: https://github.com/REVrobotics/2026-REV-ION-FRC-StarterBot

## Commands v3 version

This is the Commands v3 version of [`examples/commands2/REVCC_2026`](../../commands2/REVCC_2026). The hardware setup, YAMS configuration, setpoints and bindings are the same as that port. What differs from the v2 port:

- **Mechanisms instead of subsystems.** Each v2 `...Subsystem` is now a `...Mechanism` class in `mechanisms/` that implements `org.wpilib.command3.Mechanism`. It uses the `yams.commands3` classes (`SmartMotorControllerConfig`, `FlyWheel`, `SwerveDriveConfig`, `SwerveDrive`).
  - The v2 `setIntakePower(...)`-style setters are replaced by command factories, for example `runAtPower(power)`. Each one sets the motor, parks, and stops the motor in `whenCanceled`, which does the same job as the v2 `startEnd`.
  - The shooter has `runFlywheel()` (stops at 0 RPM, closed loop), `holdShootSpeed()` (coasts on stop) and `spinUp()` (ends once `isFlywheelSpinning` is true).
  - Mechanisms have no `periodic()`. `Robot.robotPeriodic()` calls each mechanism's `updateTelemetry()` before running the scheduler, and `Robot.simulationPeriodic()` calls `simIterate()`.
- **OpModes instead of `RobotContainer`.** `Robot` extends `OpModeRobot` and owns the mechanisms and the controller.
  - The bindings and the field-relative drive default are in the `@Teleop` opmode `opmodes/teleop/DefaultTeleop`.
  - The S-curve auto is the `@Autonomous` opmode `opmodes/auto/ExampleAuto`. It starts `Autos.exampleAuto` when the robot is enabled.
- **Coroutines.**
  - `FuelCommands` builds `Command.requiring(...)` coroutines that `awaitAll` the per-mechanism commands.
  - `shoot` awaits `spinUp()`, then awaits the flywheel and feeder together.
  - The Y toggle uses `FuelCommands.shootAndIntake`, a no-requirements coroutine that awaits `shoot` and `intake` together. It replaces v2's `alongWith`.
  - `Autos.exampleAuto` resets odometry, then awaits each `driveToPoseCommand` in turn.
  - `driveToPoseCommand` awaits YAMS `driveToPose(...).until(...)` and stops the modules when it finishes or is canceled.
- **Dashboard.**
  - The Intake, Extake, Feeder and Flywheel buttons are published through YAMS `CommandTunable`, because v3 commands are not tunables themselves.
  - The command names are the ones the factories give (for example "Intaking" instead of "Intake - Intaking").
  - The scheduler is logged with `Telemetry.log("Scheduler", ...)` instead of being published as a tunable.
- **Behavior differences from the v2 port.**
  - The drive default command and the controller bindings only exist while the teleop opmode is running. In v2 they were global.
  - Outside teleop the mechanisms run the v3 default `idle()` command.
  - v2's `utilityInit` `cancelAll()` is gone. v3 cancels commands and bindings that were scheduled in an opmode when that opmode exits.

## What changed

### Layout and framework

- The package changed from `frc.robot` to `first.robot`. `Main.java` moved to `first/Main.java`.
- The sources now live in `java/` and `deploy/` instead of `src/main/java` and `src/main/deploy`.
- It was migrated to the WPILib 2027 APIs (`org.wpilib.*`) and Commands v3 (`org.wpilib.command3.*`):
  - `TimedRobot` became `OpModeRobot`, with `@Teleop` and `@Autonomous` opmodes.
  - `CommandXboxController` became the command3 `CommandNiDsXboxController`.
  - The SPARKs are built with `CANPorts.fromBusId(1)`.
- The original `2026-Starter-Bot-SmartDashboard.xml` layout is not included. The dashboard entries are now YAMS telemetry and `Tunables`.

### Files

| Original | Port | Notes |
| --- | --- | --- |
| `Configs.java` | (removed) | All SPARK configuration now lives in each mechanism's YAMS `SmartMotorControllerConfig` |
| `subsystems/IntakeSubsystem.java` | `mechanisms/IntakeMechanism.java`, `mechanisms/ConveyorMechanism.java` | Split so each mechanism can be live tuned on its own |
| `subsystems/ShooterSubsystem.java` | `mechanisms/ShooterMechanism.java`, `mechanisms/FeederMechanism.java` | Split: the shooter keeps only the flywheel |
| (commands inside the subsystems) | `commands/FuelCommands.java` | New. `intake`, `extake`, `feed`, `shoot` and `shootAndIntake` span several mechanisms, so they live here |
| `subsystems/EasySwerveModule.java` | `mechanisms/EasySwerveModule.java` | Now a static factory that returns a YAMS `SwerveModule` |
| `subsystems/DriveSubsystem.java` | `mechanisms/DriveMechanism.java` | Rewritten on YAMS `SwerveDrive` |
| `RobotContainer.java` | `Robot.java`, `opmodes/teleop/DefaultTeleop.java` | Mechanisms and dashboard buttons are in `Robot`. The bindings are in the teleop opmode |
| `commands/Autos.java` | `commands/Autos.java`, `opmodes/auto/ExampleAuto.java` | The auto is a coroutine, run by an autonomous opmode |
| `Constants.java`, `Robot.java` | same names | Adapted (see below) |

### Mechanisms

- **Drive** (`DriveMechanism` + `EasySwerveModule`)
  - The original used hand-rolled kinematics, odometry, desaturation, optimization and software chassis angular offsets.
  - The port uses a YAMS `SwerveDrive` with four YAMS `SwerveModule`s. Each NEO is wrapped in a `SparkWrapper`.
  - **Drive motor:** closed-loop velocity.
    - kP and feedforward are converted from REV's values (kP 0.04 per m/s, kV = 12 V / free speed).
    - Brake mode, 60 A limit and the original inversion are kept.
  - **Turn motor:** closed-loop position on the Through Bore absolute encoder.
    - kP 2π per rotation, the same as REV's 1 per radian.
    - Continuous wrapping over [-0.5, 0.5) rotations.
    - The chassis angular offset is applied as the encoder zero offset instead of in software.
    - The Through Bore V2 pulse widths are passed through `withVendorConfig`.
    - New: a `kTurningMotorReduction = 20.0` constant for the relative encoder and simulation.
  - **Gyro:** changed from the ADIS16470 to the Systemcore `OnboardIMU` (the ADIS16470 does not exist on Systemcore). The turn rate now uses the Z axis.
  - **API:**
    - `drive(x, y, rot, fieldRelative)` is replaced by `getInputStream(...)` (a YAMS `SwerveInputStream`) plus `driveCommand(...)`.
    - `setXCommand` uses `lockPose`, and `zeroHeadingCommand` uses `zeroGyro`.
    - New `driveToPoseCommand(...)` and `stop()`.
  - New: YAMS telemetry (including the `SwerveDrive` field widget) and simulation through `simIterate()`.
- **Intake** (`IntakeMechanism`, SPARK Flex CAN 2) and **Conveyor** (`ConveyorMechanism`, SPARK Flex CAN 4)
  - Each is modeled as an open-loop YAMS `FlyWheel`.
  - Coast, 0.5 s ramp and 40 A are kept. The conveyor stays inverted.
- **Shooter** (`ShooterMechanism`, SPARK Flex CAN 6 + follower CAN 7)
  - A closed-loop YAMS `FlyWheel`, with the inverted follower attached through `withFollowers`.
  - kP converted from 0.0002 per RPM. kV derived from a new `kVortexFreeSpeed = 6784 RPM` constant, which replaces `kVortexKv = 565`.
  - The MAXMotion velocity profile (5000 RPM, 10000 RPM/s) is set with `withTrapezoidalProfile`. REV's `allowedProfileError(1)` has no counterpart.
  - Inverted, coast, 1.0 s ramps and 80 A are kept.
- **Feeder** (`FeederMechanism`, SPARK Flex CAN 5)
  - An open-loop YAMS `FlyWheel`. Inverted, coast, 1.0 s ramp and 60 A are kept.
- **All mechanisms**
  - New: YAMS telemetry at HIGH verbosity and simulation.
  - The roller gearing, diameters and moments of inertia are estimates used only for sim and telemetry.

### Commands, bindings and constants

- The bindings are unchanged:
  - Left stick: X lock.
  - Start: zero heading.
  - Right trigger: intake.
  - Left trigger: extake.
  - Y: toggles shoot along with intake.
  - Drive default: field-relative drive with a 0.1 deadband.
- `FuelCommands` keeps the original setpoints: intake ±0.6, conveyor ±0.7, flywheel 5000 RPM, feeder 0.95.
- `isFlywheelSpinning`, `isFlywheelSpinningBackwards` and `isFlywheelStopped` are kept as command3 `Trigger`s. They now compare typed `kShootRpm`/`kVelocityTolerance` constants against the YAMS flywheel speed.
- The dashboard buttons (Intake, Extake, Feeder, Flywheel) are published with `Tunables.publish` under the same names.
  - The "Intake | ..." and "Shooter | ..." SmartDashboard entries are replaced by YAMS telemetry.
  - The mechanisms themselves are not published.
- CAN IDs, setpoints, deadband, track width, wheelbase, gear teeth and the motor-on-bottom flags are unchanged.
- Several constants are now typed units (`kMaxSpeed`, `kMaxAngularSpeed`, `kWheelDiameter`, `kFreeSpeed`, flywheel RPMs).
  - The chassis offsets are expressed as 315/45/225/135 degrees, which equal the original radians.
  - `kDriveKinematics` and `AutoConstants` were removed.

### Behavior differences

- **Auto path is different.**
  - The original followed one smooth S-curve with `TrajectoryGenerator` and `SwerveControllerCommand`, at 3 m/s and 3 m/s². WPILib 2027 dropped `SwerveControllerCommand`.
  - The port resets odometry, then drives to (1, 1), (2, -1) and (3, 0) in sequence with `driveToPoseCommand`. It settles within 5 cm / 3° at each point.
  - Speed is capped only by the 4.8 m/s chassis limit.
- **SPARK configuration is not reset.** The original reset the SPARKs to safe parameters and persisted them on every boot. `SparkWrapper` does not reset unless asked to, and persists only while disabled.
- **Shooter stop.** After `shoot`, the shooter is stopped with duty cycle 0 instead of `stopMotor()`. In coast mode, that coasts down the same way.
- **Command requirements.** Intake/extake require Intake and Conveyor, and feed/shoot require Shooter and Feeder. A command that needs any one of them cancels the combined command.
- **Retune the closed-loop gains.** They were converted by unit math from REV's values and should be checked on a real robot.
