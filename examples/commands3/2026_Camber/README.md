# 2026 Camber (YAMS port, Commands v3)

A port of Team 9658 Camber Robotics' 2026 robot code to WPILib 2027, Commands v3 and YAMS. It is a
swerve shooter robot with a distance-based shot table, Limelight pose estimation and PathPlanner
autos.

## Original source

- Repository: https://github.com/9658-Camber-Robotics/2026-KitBot

## What changed

The hardware, gains and constants are ported the same way as the `commands2` version; see its
README for the swerve, Limelight, shooter and indexer details. This file covers what is different
for Commands v3.

### Layout and framework

- The package changed from `frc.robot` to `first.robot`. `Main.java` moved to `first/Main.java`.
- The sources now live in `java/` and `deploy/`. The PathPlanner paths, autos and settings are
  unchanged. The YAGSL JSON files were moved into `Constants.SwerveDrive.Modules`.
- It was migrated to the WPILib 2027 APIs (`org.wpilib.*`):
  - `TimedRobot` became `OpModeRobot`, and `CommandScheduler` became the Commands v3 `Scheduler`.
  - `CommandXboxController` became the Commands v3 `CommandNiDsXboxController`.
  - `SmartDashboard.putNumber` became `Telemetry.log`.
- `RobotContainer` was removed. The mechanisms, controllers and shooter commands are fields on
  `Robot`, which also sets the default commands and registers the named commands.
- `SendableChooser` was removed because 2027 uses opmodes. Each PathPlanner auto is its own
  `@Autonomous` opmode: `Left Auto`, `Middle Auto` and `Right Auto`. The bindings are the
  `Camber Teleop` opmode.
- Every device is on `CANPort.CAN_S0`, because Systemcore has no "rio" bus.
- Vendordeps: YAGSL is replaced by the YAMS `SwerveDrive` and YALL by LimelightLib 2027. PathPlannerLib
  is not used; see below.

### Files

| Original | Port | Notes |
| --- | --- | --- |
| `Robot.java`, `RobotContainer.java` | `Robot.java` | `OpModeRobot` holding the mechanisms, default commands and named commands |
| (bindings inside `RobotContainer`) | `opmodes/teleop/CamberTeleop.java` | `@Teleop` opmode with the original bindings |
| (`getAutonomousCommand`) | `opmodes/auto/LeftAuto.java`, `MiddleAuto.java`, `RightAuto.java` | One `@Autonomous` opmode per PathPlanner auto |
| `subsystems/SwerveSubsystem.java` | `mechanisms/SwerveMechanism.java` | YAMS `SwerveDrive`, Limelight fusion and PathPlanner path following |
| `subsystems/ShooterSubsystem.java`, `subsystems/IndexerSubsystem.java` | `mechanisms/ShooterMechanism.java`, `mechanisms/IndexerMechanism.java` | YAMS `FlyWheel`s |
| `commands/IntakeCommand.java`, `OuttakeCommand.java`, `ShootAndIndexCommand.java`, `AutoShoot.java` | `commands/ShooterCommands.java` | Coroutine command factories with the same logic |
| `commands/AutoAimCommand.java`, `driveDirectAngle` in `RobotContainer` | `commands/Drive.java` | `autoAim` and `driveDirectAngle` |
| (PathPlannerLib) | `pathplanner/*.java` | Reads the PathPlanner files; see below |
| `subsystems/ClimbSubsystem.java` | (removed) | It was fully commented out |
| `utils/AllianceFlipUtil.java`, `utils/FieldConstants.java` | same names | WPILib 2027 APIs. `AllianceFlipUtil.flip` is new, for paths |

### PathPlanner

PathPlannerLib's 2027 build is made for Commands v2, which cannot be used in the same project as
Commands v3. The `pathplanner` package reads the same `deploy/pathplanner` files instead:

- `PathPlannerPath` turns each pair of waypoints into the Bézier curve the GUI draws and time
  parameterizes it with WPILib's trajectory tools. It honors the path's global constraints,
  constraint zones, and start and end velocities.
- The heading moves from the ideal starting rotation, through any rotation targets, to the goal
  rotation, in step with the robot's progress along the path.
- Event markers fork their command when the robot reaches them. A zoned marker's command is
  canceled when the robot leaves the zone, and any marker command still running is canceled when
  the path ends, as PathPlannerLib does.
- `AutoBuilder` and `NamedCommands` mirror PathPlannerLib's API. Sequential, parallel, race,
  deadline, path, named and wait commands are supported. Autos with `resetOdom` reset odometry to
  the start of their first path.
- Named commands are registered as factories (`Supplier<Command>`), so an auto can use the same name
  more than once.
- `SwerveMechanism.followPath` follows a path with the same PID 5/5 as the original
  `PPHolonomicDriveController`, plus the path's velocity as feedforward. Red alliance paths are
  mirrored with `AllianceFlipUtil`.

### Mechanisms and commands

- Each mechanism's `periodic()` and `simulationPeriodic()` is called from `Robot`.
- Default commands, set in `Robot` for every mode:
  - The drivetrain runs `Drive.driveDirectAngle`.
  - The shooter holds the operator's right stick velocity, as before.
  - The indexer is stopped at the lowest priority.
- `ShooterCommands` requires the shooter and indexer and stops both when a command ends or is
  canceled, as the original `end()` methods did.
- `Drive.driveDirectAngle` and `Drive.autoAim` each own a YAMS `SwerveInputStream` and set its sticks
  every loop. `driveDirectAngle` keeps the last heading while the right stick is near the center, as
  YAGSL did.

### Behavior differences

Beyond those listed in the `commands2` README:

- **Path following.** The robot follows paths with PID plus the path velocity. PathPlannerLib's
  per-module force feedforwards and its own trajectory generator are not used, so timing near
  sharp turns can differ slightly from the GUI preview.
- **No auto chooser or `Index Balls` dashboard command.** Pick an autonomous opmode on the driver
  station instead.
- **No pathfinding or setpoint generator.** `driveToPose` drives straight to the pose with the YAMS
  drive-to-pose PID, and `driveWithSetpointGenerator` was removed; neither was used.
