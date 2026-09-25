# Kitbot 2026 (YAMS port)

A port of the FIRST 2026 KitBot code to WPILib 2027 and YAMS.

## Original source

- Download: https://firstfrc.blob.core.windows.net/frc2026/KitBot/2026-kitbot-code.zip
- The zip ships two variants, `2026KitBotClasses` and `2026KitBotInline`. This port is based on **`2026KitBotInline`** (inline command factories, `Autos.exampleAuto(...)`, `driveArcade` returning a `Command`).

## What changed

### Layout and framework

- The package changed from `frc.robot` to `first.robot`. `Main.java` moved to `first/Main.java`.
- The sources now live in `java/` and `deploy/` instead of `src/main/java` and `src/main/deploy`.
- It was migrated to the WPILib 2027 APIs (`org.wpilib.*`):
  - `CommandXboxController` became `CommandNiDsXboxController`.
  - `robotInit()` became the `Robot()` constructor.
  - `testInit/testPeriodic` became `utilityInit/utilityPeriodic`.
- `SendableChooser` was removed because 2027 uses opmodes. `getAutonomousCommand()` always returns the one example auto.

### Files

| Original | Port | Notes |
| --- | --- | --- |
| `subsystems/CANDriveSubsystem.java` | `subsystems/CANDriveSubsystem.java` | Rewritten on YAMS `SmartMotorController`s |
| `subsystems/CANFuelSubsystem.java` | `subsystems/FeederSubsystem.java`, `subsystems/IntakeLauncherSubsystem.java` | Split so each roller is its own subsystem and can be live tuned on its own |
| (logic inside `CANFuelSubsystem`) | `commands/FuelCommands.java` | New. Holds the intake/eject/spin-up/launch/stop logic and command factories that span both rollers |
| `commands/Autos.java` | `commands/Autos.java` | Takes `FuelCommands` instead of `CANFuelSubsystem` |
| `Constants.java` | `Constants.java` | Same values; current limits and roller voltages typed as `Current`/`Voltage`; new drive gear ratio, wheel diameter and nominal voltage |

### Subsystems

- **Drive** (`CANDriveSubsystem`)
  - Uses the same four brushed SPARK MAX controllers (CAN 1 to 4). Each leader is wrapped in a YAMS `SparkWrapper` (`DCMotor.getCIM(2)`), and its follower is attached with `withFollowers`.
  - The wrappers run open loop, with 12 V voltage compensation, a 60 A current limit and the left side inverted, as in the original.
  - `DifferentialDrive` is fed the wrappers' `setDutyCycle`.
  - New: gear ratio 8.45 and 6 in wheels. These values are only used for telemetry and simulation.
  - New: YAMS telemetry (`LeftDrive`, `RightDrive`) and simulation through `simIterate()`.
- **Feeder** (`FeederSubsystem`, CAN 6) and **Intake/Launcher** (`IntakeLauncherSubsystem`, CAN 5)
  - Each is modeled as a YAMS `FlyWheel` on a `SparkWrapper`, running open loop with a 60 A limit. The intake/launcher stays inverted.
  - Voltages are applied with `setVoltageSetpoint`. Stop is `setDutyCycleSetpoint(0)`.
  - New: YAMS telemetry and simulation. The motor type (CIM), roller diameters (2 in, 4 in) and moments of inertia are estimates used only for sim and telemetry.

### Commands, bindings and constants

- The SmartDashboard tuning values were removed because YAMS live tuning replaces them. The roller voltages are read straight from `Constants.FuelConstants`; tune them live with YAMS, then copy the values back into the constants.
- The bindings are unchanged:
  - Left bumper: intake.
  - Right bumper: spin up for 1 s, then launch.
  - A: eject.
  - Drive default: arcade drive with 0.7 / 0.8 scaling.
- The auto keeps the original steps: drive at 0.5 for 0.25 s, stop, spin up 1 s, launch 9 s, stop.
- CAN IDs, current limits and every roller voltage are unchanged.

### Behavior differences

- **Auto fix.** In the original, the "stop driving" step used `driveArcade(() -> 0, () -> 0)`, a `run` command that never ends, so the auto never reached spin-up and launch. The port adds a `stopCommand()` that ends right away, so the auto now launches.
- `setCANTimeout(250)` and the explicit REV `ResetMode`/`PersistMode` flags are no longer called. The controllers are built with the 2027 `CANPorts.fromBusId(1)` API.
- Every fuel command requires both roller subsystems, so scheduling behaves as it did with the single original subsystem.
