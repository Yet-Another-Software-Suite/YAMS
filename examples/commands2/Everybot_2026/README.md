# Everybot 2026 (YAMS port)

A port of the Robonauts Everybot 2026 code to WPILib 2027 and YAMS.

## Original source

- Repository: https://github.com/Robonauts-Everybot/FRC-Everybot-2026-Code/

## What changed

### Layout and framework

- The package changed from `frc.robot` to `first.robot`. `Main.java` moved to `first/Main.java`.
- The sources now live in `java/` and `deploy/` instead of `src/main/java` and `src/main/deploy`.
- It was migrated to the WPILib 2027 APIs (`org.wpilib.*`):
  - `CommandXboxController` became `CommandNiDsXboxController`, and the POV triggers go through `getHID().povUp()/povDown()`.
  - `robotInit()` became the `Robot()` constructor.
  - `testInit/testPeriodic` became `utilityInit/utilityPeriodic`.
- `SendableChooser` was removed because 2027 uses opmodes. `getAutonomousCommand()` always returns `ExampleAuto`.

### Files

| Original | Port | Notes |
| --- | --- | --- |
| `subsystems/CANFuelSubsystem.java` | `subsystems/IntakeLauncherSubsystem.java`, `subsystems/IndexerSubsystem.java` | Split so each mechanism is its own subsystem and can be live tuned on its own |
| `subsystems/CANDriveSubsystem.java` | `subsystems/CANDriveSubsystem.java` | Rewritten on YAMS `SmartMotorController`s |
| `subsystems/ClimberSubsystem.java` | `subsystems/ClimberSubsystem.java` | Rewritten on a YAMS `SmartMotorController` |
| `commands/*.java` (10 commands) | `commands/*.java` | Same commands; the fuel commands take `(IntakeLauncherSubsystem, IndexerSubsystem)` instead of `CANFuelSubsystem` |
| `Constants.java` | `Constants.java` | Same values; `ClimbConstatns` renamed to `ClimbConstants`; current limits typed as `Current`; new drive gear ratio, wheel diameter and nominal voltages |

### Subsystems

Every motor is a SPARK MAX wrapped in a YAMS `SparkWrapper` and driven open loop, as in the original.

- **Drive** (`CANDriveSubsystem`, CAN 1 to 4)
  - Uses two leader `SmartMotorController`s (`DCMotor.getCIM(2)`) with the followers attached through `withFollowers`.
  - Settings match the original: 12 V voltage compensation, 60 A limit, brake mode, left side inverted.
  - `DifferentialDrive` is fed the wrappers' `setDutyCycle`.
  - New: gear ratio 8.45 and 6 in wheels. These values are only used for telemetry and simulation.
- **Intake/Launcher** (`IntakeLauncherSubsystem`, CAN 5 and 6)
  - Modeled as a YAMS `FlyWheel` (`DCMotor.getNEO(2)`), with 80 A, 12 V compensation and coast.
  - The right motor (6) leads, and the left motor (5) is an inverted follower. In the original, the two motors were commanded separately.
- **Indexer** (`IndexerSubsystem`, CAN 8)
  - Modeled as a YAMS `FlyWheel` (`DCMotor.getCIM(1)`) with an 80 A limit.
- **Climber** (`ClimberSubsystem`, CAN 7)
  - A plain `SmartMotorController` (`DCMotor.getCIM(1)`) with 40 A and brake mode.
- **All subsystems**
  - New: YAMS telemetry at HIGH verbosity and simulation through `simIterate()`.
  - The motor types for brushed motors, the roller diameters and the moments of inertia are estimates used only for sim and telemetry.

### Commands, bindings and constants

- The bindings are unchanged:
  - Left bumper: Intake.
  - Right bumper: LaunchSequence.
  - A: Eject.
  - POV up/down: ClimbUp/ClimbDown.
  - Drive default: arcade drive with 0.7 / 0.8 scaling.
- The climber, the intake/launcher and the indexer each have a stop default command. The original had one for the combined fuel subsystem.
- `ExampleAuto` is unchanged: drive at 0.5 for 3 s, then LaunchSequence (0.75 s spin-up, then launch) for 10 s.
- The SmartDashboard tuning values were removed because YAMS live tuning replaces them. The commands read the roller percentages straight from `Constants.FuelConstants`; tune them live with YAMS, then copy the values back into the constants.
- CAN IDs, percentages, current limits and controller ports are unchanged.

### Behavior differences

- **Eject speed.** The original Eject read the `"Intaking intake roller value"` dashboard key, so by default it ran the rollers at -0.6. The port uses `INTAKE_EJECT_PERCENT` (-0.8), which the original only used as the fallback when that key was missing. The feeder still runs at 0.6 (`INDEXER_LAUNCHING_PERCENT`).
- `setCANTimeout(250)` and the explicit REV `ResetMode`/`PersistMode` flags are no longer called. The controllers are built with the 2027 `CANPorts.fromBusId(1)` API.
