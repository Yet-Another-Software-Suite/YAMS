# Kitbot 2026 (YAMS port, Commands v3)

A port of the FIRST 2026 KitBot code to WPILib 2027, Commands v3 and YAMS.

## Original source

- Download: https://firstfrc.blob.core.windows.net/frc2026/KitBot/2026-kitbot-code.zip
- The zip ships two variants, `2026KitBotClasses` and `2026KitBotInline`. This port is based on **`2026KitBotInline`** (inline command factories, `Autos.exampleAuto(...)`, `driveArcade` returning a `Command`).

## What changed

### Layout and framework

- The package changed from `frc.robot` to `first.robot`. `Main.java` moved to `first/Main.java`.
- The sources now live in `java/` and `deploy/` instead of `src/main/java` and `src/main/deploy`.
- It was migrated to the WPILib 2027 APIs (`org.wpilib.*`):
  - `TimedRobot` became `OpModeRobot`, and `CommandScheduler` became the Commands v3 `Scheduler`.
  - `CommandXboxController` became the Commands v3 `CommandNiDsXboxController`.
  - `robotInit()` became the `Robot()` constructor.
- `RobotContainer` was removed. The mechanisms, fuel commands and controllers are fields on `Robot`.
- `SendableChooser` was removed because 2027 uses opmodes. The teleop bindings are the `KitBot Teleop` opmode and the one example auto is the `Example Auto` opmode.

### Files

| Original | Port | Notes |
| --- | --- | --- |
| `Robot.java`, `RobotContainer.java` | `Robot.java` | `OpModeRobot` holding the mechanisms and controllers; runs the scheduler, YAMS telemetry and simulation |
| (bindings inside `RobotContainer`) | `opmodes/teleop/KitBotTeleop.java` | New. `@Teleop` opmode with the button bindings and the arcade drive default command |
| `subsystems/CANDriveSubsystem.java` | `mechanisms/CANDriveMechanism.java` | A Commands v3 `Mechanism` rewritten on YAMS `SmartMotorController`s |
| `subsystems/CANFuelSubsystem.java` | `mechanisms/FeederMechanism.java`, `mechanisms/IntakeLauncherMechanism.java` | Split so each roller is its own mechanism and can be live tuned on its own |
| (logic inside `CANFuelSubsystem`) | `commands/FuelCommands.java` | New. Combines the rollers' own commands into the intake, eject and spin-up-and-launch commands |
| `commands/Autos.java` | `opmodes/auto/ExampleAuto.java` | `@Autonomous` opmode running the example auto as a coroutine |
| `Constants.java` | `Constants.java` | Same values; current limits and roller voltages typed as `Current`/`Voltage`; new drive gear ratio, wheel diameter and nominal voltage |

### Mechanisms

- **Drive** (`CANDriveMechanism`)
  - Uses the same four brushed SPARK MAX controllers (CAN 1 to 4). Each leader is wrapped in a YAMS `SparkWrapper` (`DCMotor.getCIM(2)`), and its follower is attached with `withFollowers`.
  - The wrappers run open loop, with 12 V voltage compensation, a 60 A current limit and the left side inverted, as in the original.
  - `DifferentialDrive` is fed the wrappers' `setDutyCycle`.
  - New: gear ratio 8.45 and 6 in wheels. These values are only used for telemetry and simulation.
  - New: YAMS telemetry (`LeftDrive`, `RightDrive`) and simulation through `simIterate()`.
- **Feeder** (`FeederMechanism`, CAN 6) and **Intake/Launcher** (`IntakeLauncherMechanism`, CAN 5)
  - Each is modeled as a YAMS `FlyWheel` on a `SparkWrapper`, running open loop with a 60 A limit. The intake/launcher stays inverted.
  - Each exposes one named command per roller action (`intake()`, `eject()`, `launch()`, plus `spinUp()` on the feeder). Each sets its voltage with `setVoltageSetpoint` and holds it until interrupted.
  - Each overrides `idle()` with a lowest-priority command that stops the roller (`setDutyCycleSetpoint(0)`). `Robot` sets it as the roller's default command, so a roller stops whenever no fuel command is using it.
  - New: YAMS telemetry and simulation. The motor type (CIM), roller diameters (2 in, 4 in) and moments of inertia are estimates used only for sim and telemetry.

### Commands, bindings and constants

- The SmartDashboard tuning values were removed because YAMS live tuning replaces them. The roller voltages are read straight from `Constants.FuelConstants`; tune them live with YAMS, then copy the values back into the constants.
- The bindings are unchanged:
  - Left bumper: intake.
  - Right bumper: spin up for 1 s, then launch.
  - A: eject.
  - Drive default (teleop only): arcade drive with 0.7 / 0.8 scaling.
- The auto keeps the original steps: drive at 0.5 for 0.25 s, stop, spin up 1 s, launch 9 s, stop.
- CAN IDs, current limits and every roller voltage are unchanged.

### Behavior differences

- `setCANTimeout(250)` and the explicit REV `ResetMode`/`PersistMode` flags are no longer called. The controllers are built with the 2027 `CANPorts.fromBusId(1)` API.
- Every fuel command runs a command on both rollers, so starting one fuel command still interrupts another, as it did with the single original subsystem.
- The auto now reaches the launch step. In the original, the "stop driving" step used the same never-ending `driveArcade` command, so the sequence never got past it; here the auto drives for 0.25 s, then commands the motors to zero once and moves on.
- Outside teleop the drive's default command holds the motors at zero (feeding motor safety) instead of reading the driver's joysticks.

## Commands v3 version

This is the Commands v3 version of [`examples/commands2/Kitbot_2026`](../../commands2/Kitbot_2026). It matches the v2 port's hardware, constants, bindings and auto. What differs from the v2 port:

- Subsystems became `org.wpilib.command3.Mechanism` classes in the `mechanisms` package. Their YAMS telemetry and simulation calls moved from `periodic()`/`simulationPeriodic()` overrides into methods that `Robot.robotPeriodic()` and `Robot.simulationPeriodic()` call.
- `RobotContainer` is gone. The bindings live in the `KitBot Teleop` opmode and the auto in the `Example Auto` opmode, so they only exist while that opmode is selected.
- Each roller mechanism has its own command factories (`Feeder.Intake`, `IntakeLauncher.Launch`, and so on) that set a voltage and `park()`, and a lowest-priority `idle()` default command that stops the roller.
- The fuel commands in `FuelCommands` are `Command.noRequirements` coroutines that run those roller commands as children. `Fuel.Intake` and `Fuel.Eject` `awaitAll` both rollers' commands. `Fuel.SpinUpAndLaunch` `fork`s the launcher's launch command and the feeder's spin-up command, `wait`s 1 s, then `await`s the feeder's launch command, which replaces the spin-up command. Each roller is only owned while its child command runs, so when a fuel command is canceled the rollers' idle commands stop them. This replaces the v2 `runEnd`/`finallyDo` stops.
- The drive commands (`Drive.Arcade`, `Drive.ArcadeFixed`, `Drive.Idle`) are `while (true)` loops that call `arcadeDrive` and `yield()` every loop to keep motor safety fed. `Drive.Idle` runs at the lowest priority.
- The auto is one `Command.noRequirements` coroutine written as plain sequential code instead of a `SequentialCommandGroup`. It `awaitAny`s the fixed-speed drive command against a 0.25 s `Command.waitFor`, then `awaitAny`s `Fuel.SpinUpAndLaunch` against a 10 s wait (1 s spin-up plus 9 s launch). Between and after the steps, the default commands keep the drive and rollers stopped. It starts when the robot is enabled in the `Example Auto` opmode.
- Every command is named (`Drive.Arcade`, `Fuel.Intake`, `Fuel.SpinUpAndLaunch`, and so on).
