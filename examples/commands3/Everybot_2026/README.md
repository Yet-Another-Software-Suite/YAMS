# Everybot 2026 (YAMS port, Commands v3)

A port of the Robonauts Everybot 2026 code to WPILib 2027, Commands v3 and YAMS.

## Original source

- Repository: https://github.com/Robonauts-Everybot/FRC-Everybot-2026-Code/

## Commands v3 version

This is the Commands v3 version of [`examples/commands2/Everybot_2026`](../../commands2/Everybot_2026). The hardware setup, constants, bindings and behavior match that port. The differences from the v2 port are:

- `Robot` extends `OpModeRobot` instead of `TimedRobot`, and `RobotContainer` is gone. `Robot` holds the mechanisms and controllers, sets the stop default commands and runs `Scheduler.getDefault().run()`.
- The subsystems became `org.wpilib.command3.Mechanism` classes in `mechanisms/` (`CANDriveMechanism`, `IntakeLauncherMechanism`, `IndexerMechanism`, `ClimberMechanism`). They are still split one YAMS mechanism per class so live tuning works.
- Mechanisms have no `periodic()` in v3, so `Robot.robotPeriodic()` calls each mechanism's `updateTelemetry()` and `Robot.simulationPeriodic()` calls each `simIterate()`.
- The 10 command classes became coroutine command factories and plain mechanism methods:
  - `ClimbUp`/`ClimbDown` are `climber.climbUp()`/`climbDown()`, which set the climber power in a `while (true)` loop.
  - `Drive` is `drive.arcadeDrive(controller)`, which reads the joysticks and sends them every loop in a `while (true)` loop. `AutoDrive` is `drive.arcadeDrive(xSpeed, zRotation)`, which sends fixed speeds every loop until it is interrupted.
  - Each fuel mechanism has its own command factories for its roller actions: `intakeLauncher.intake()`, `eject()` and `launch()`, and `indexer.intake()`, `feed()` and `holdBack()`. Each one sets the roller power once, parks, and stops the roller when canceled.
  - `Intake`, `Eject` and `LaunchSequence` are coroutine commands in `commands/FuelCommands` built with `Command.noRequirements(...)`. `Intake` and `Eject` `awaitAll` the two roller commands. `LaunchSequence` forks `intakeLauncher.launch()` and `indexer.holdBack()`, waits 0.75 s, then awaits `indexer.feed()`, which interrupts its sibling `holdBack()`. `SpinUp` and `Launch` are those two steps. Because nested commands in v3 only own a mechanism while they run, the fuel commands never hold a mechanism they are not driving.
- Each mechanism overrides `idle()` with a `while (true)` loop that holds it stopped, at `Command.LOWEST_PRIORITY`, which `Robot` sets as the default command. The teleop arcade drive default also runs at the lowest priority.
- The bindings moved to the `@Teleop` opmode `opmodes/teleop/DefaultTeleop`. It also sets the joystick arcade drive as the drive default command while it is active.
- `ExampleAuto` is now the `@Autonomous` opmode `opmodes/auto/ExampleAuto`. It is one `Command.noRequirements(...)` coroutine. It races `drive.arcadeDrive(0.5, 0.0)` against a 3 s `Command.waitFor(...)` with `awaitAny`, then races `FuelCommands.launchSequence(...)` against a 10 s wait the same way: spin up for 0.75 s and feed for the remaining 9.25 s. Each mechanism's stop default command takes over as soon as its step ends, so the drive is held stopped while launching and the rollers stop at the end. It starts when the robot is enabled. Pick it on the Driver Station instead of it being hardcoded in `getAutonomousCommand()`.
- The v3 controller class is `org.wpilib.command3.button.CommandNiDsXboxController`, and the POV triggers still go through `getHID().povUp()/povDown()`.
- The drive only runs joystick arcade drive while the teleop opmode is active. Outside teleop it holds the drive stopped. In the v2 port the joystick drive was the global default, so it also ran after `ExampleAuto` finished.

## What changed

### Layout and framework

- The package changed from `frc.robot` to `first.robot`. `Main.java` moved to `first/Main.java`.
- The sources now live in `java/` and `deploy/` instead of `src/main/java` and `src/main/deploy`.
- It was migrated to the WPILib 2027 APIs (`org.wpilib.*`) and Commands v3 (`org.wpilib.command3.*`):
  - `CommandXboxController` became `CommandNiDsXboxController`, and the POV triggers go through `getHID().povUp()/povDown()`.
  - `TimedRobot` became `OpModeRobot`. `robotInit()` became the `Robot()` constructor.
  - `SendableChooser` was removed because 2027 uses opmodes. The one auto the original offered is an `@Autonomous` opmode.

### Files

| Original | Port | Notes |
| --- | --- | --- |
| `subsystems/CANFuelSubsystem.java` | `mechanisms/IntakeLauncherMechanism.java`, `mechanisms/IndexerMechanism.java` | Split so each mechanism is its own `Mechanism` and can be live tuned on its own |
| `subsystems/CANDriveSubsystem.java` | `mechanisms/CANDriveMechanism.java` | Rewritten on YAMS `SmartMotorController`s |
| `subsystems/ClimberSubsystem.java` | `mechanisms/ClimberMechanism.java` | Rewritten on a YAMS `SmartMotorController` |
| `commands/Intake.java`, `Eject.java`, `SpinUp.java`, `Launch.java`, `LaunchSequence.java` | `commands/FuelCommands.java` | Coroutine command factories that take `(IntakeLauncherMechanism, IndexerMechanism)` and run the mechanisms' roller commands; `SpinUp` and `Launch` are steps of the launch sequence |
| `commands/Drive.java`, `AutoDrive.java` | `CANDriveMechanism.arcadeDrive(controller)`, `arcadeDrive(xSpeed, zRotation)` | Command factories on the drive |
| `commands/ClimbUp.java`, `ClimbDown.java` | `ClimberMechanism.climbUp()`, `climbDown()` | Command factories on the climber |
| `commands/ExampleAuto.java` | `opmodes/auto/ExampleAuto.java` | `@Autonomous` opmode |
| `RobotContainer.java` | `Robot.java`, `opmodes/teleop/DefaultTeleop.java` | Mechanisms in `Robot`, bindings in the `@Teleop` opmode |
| `Constants.java` | `Constants.java` | Same values; `ClimbConstatns` renamed to `ClimbConstants`; current limits typed as `Current`; new drive gear ratio, wheel diameter and nominal voltages |

### Mechanisms

Every motor is a SPARK MAX wrapped in a YAMS `SparkWrapper` and driven open loop, as in the original.

- **Drive** (`CANDriveMechanism`, CAN 1 to 4)
  - Uses two leader `SmartMotorController`s (`DCMotor.getCIM(2)`) with the followers attached through `withFollowers`.
  - Settings match the original: 12 V voltage compensation, 60 A limit, brake mode, left side inverted.
  - `DifferentialDrive` is fed the wrappers' `setDutyCycle`.
  - New: gear ratio 8.45 and 6 in wheels. These values are only used for telemetry and simulation.
- **Intake/Launcher** (`IntakeLauncherMechanism`, CAN 5 and 6)
  - Modeled as a YAMS `FlyWheel` (`DCMotor.getNEO(2)`), with 80 A, 12 V compensation and coast.
  - The right motor (6) leads, and the left motor (5) is an inverted follower. In the original, the two motors were commanded separately.
- **Indexer** (`IndexerMechanism`, CAN 8)
  - Modeled as a YAMS `FlyWheel` (`DCMotor.getCIM(1)`) with an 80 A limit.
- **Climber** (`ClimberMechanism`, CAN 7)
  - A plain `SmartMotorController` (`DCMotor.getCIM(1)`) with 40 A and brake mode.
- **All mechanisms**
  - New: YAMS telemetry at HIGH verbosity and simulation through `simIterate()`.
  - The motor types for brushed motors, the roller diameters and the moments of inertia are estimates used only for sim and telemetry.

### Commands, bindings and constants

- The bindings are unchanged:
  - Left bumper: Intake.
  - Right bumper: LaunchSequence.
  - A: Eject.
  - POV up/down: ClimbUp/ClimbDown.
  - Drive default (teleop): arcade drive with 0.7 / 0.8 scaling.
- The drive, the climber, the intake/launcher and the indexer each have a stop default command. The original had one for the combined fuel subsystem.
- `ExampleAuto` is unchanged: drive at 0.5 for 3 s, then LaunchSequence (0.75 s spin-up, then launch) for 10 s.
- The SmartDashboard tuning values were removed because YAMS live tuning replaces them. The commands read the roller percentages straight from `Constants.FuelConstants`; tune them live with YAMS, then copy the values back into the constants.
- CAN IDs, percentages, current limits and controller ports are unchanged.

### Behavior differences

- **Eject speed.** The original Eject read the `"Intaking intake roller value"` dashboard key, so by default it ran the rollers at -0.6. The port uses `INTAKE_EJECT_PERCENT` (-0.8), which the original only used as the fallback when that key was missing. The feeder still runs at 0.6 (`INDEXER_LAUNCHING_PERCENT`).
- **Drive outside teleop.** The original's joystick drive was the global default command. The port only runs it in the teleop opmode; in other modes the drive default holds the drive stopped.
- **Roller commands.** Like the original, the fuel commands set the roller powers and stop the rollers when they end. The stop default commands then take over and resend 0 every loop.
- **Ownership.** The original commands and auto required every mechanism for their whole run. The port's fuel commands and auto only own each mechanism while that mechanism's own command runs, so a stop default command drives any mechanism the routine is not using. The robot does the same thing either way.
- `setCANTimeout(250)` and the explicit REV `ResetMode`/`PersistMode` flags are no longer called. The controllers are built with the 2027 `CANPorts.fromBusId(1)` API.
