# 2026 Camber (YAMS port, Commands v3)

A port of Team 9658 Camber Robotics' 2026 robot code to WPILib 2027, Commands v3 and YAMS. It is a
swerve shooter robot with a distance-based shot table, Limelight pose estimation and three
autos.

## Original source

- Repository: https://github.com/9658-Camber-Robotics/2026-KitBot

## What changed

The hardware, gains and constants are ported the same way as the `commands2` version; see its
README for the swerve, Limelight, shooter and indexer details. This file covers what is different
for Commands v3.

### Layout and framework

- The package changed from `frc.robot` to `first.robot`. `Main.java` moved to `first/Main.java`.
- The sources now live in `java/` and `deploy/`. The YAGSL JSON files were moved into
  `Constants.SwerveDrive.Modules`. The PathPlanner autos were converted to code and the
  `deploy/pathplanner` files removed; see below.
- It was migrated to the WPILib 2027 APIs (`org.wpilib.*`):
  - `TimedRobot` became `OpModeRobot`, and `CommandScheduler` became the Commands v3 `Scheduler`.
  - `CommandXboxController` became the Commands v3 `CommandNiDsXboxController`.
  - `SmartDashboard.putNumber` became `Telemetry.log`.
- `RobotContainer` was removed. The mechanisms, controllers and shooter commands are fields on
  `Robot`, which also sets the shooter and indexer default commands.
- `SendableChooser` was removed because 2027 uses opmodes. Each PathPlanner auto is now its own
  `@Autonomous` opmode: `Left Auto`, `Middle Auto` and `Right Auto`. There are two teleop opmodes,
  `HeadingTeleop` (the original `driveDirectAngle`) and `AngularVelocityTeleop`, which share the
  original bindings.
- Every device is on `CANPort.CAN_S0`, because Systemcore has no "rio" bus.
- Vendordeps: YAGSL is replaced by the YAMS `SwerveDrive` and YALL by LimelightLib 2027. PathPlannerLib
  is not used, because its 2027 build is made for Commands v2; see below.

### Files

| Original | Port | Notes |
| --- | --- | --- |
| `Robot.java`, `RobotContainer.java` | `Robot.java` | `OpModeRobot` holding the mechanisms and default commands |
| (bindings inside `RobotContainer`) | `opmodes/teleop/HeadingTeleop.java`, `AngularVelocityTeleop.java`, `TeleopBindings.java` | `@Teleop` opmodes that each build a YAMS `SwerveInputStream` for the driver, plus the original bindings they share |
| `deploy/pathplanner/autos/*.auto`, `paths/*.path`, named commands in `RobotContainer` | `opmodes/auto/LeftAuto.java`, `MiddleAuto.java`, `RightAuto.java`, `AutoSteps.java` | One `@Autonomous` opmode per PathPlanner auto, driving the paths with drive to pose |
| `subsystems/SwerveSubsystem.java` | `mechanisms/SwerveMechanism.java` | YAMS `SwerveDrive`, Limelight fusion and drive to pose |
| `subsystems/ShooterSubsystem.java`, `subsystems/IndexerSubsystem.java` | `mechanisms/ShooterMechanism.java`, `mechanisms/IndexerMechanism.java` | YAMS `FlyWheel`s |
| `commands/IntakeCommand.java`, `OuttakeCommand.java`, `ShootAndIndexCommand.java`, `AutoShoot.java` | `commands/ShooterCommands.java` | Coroutine command factories with the same logic |
| `commands/AutoAimCommand.java`, `driveDirectAngle` in `RobotContainer` | `SwerveMechanism.driveAimedAt`, `opmodes/teleop/HeadingTeleop.java` | Auto aim and direct angle driving |
| `subsystems/ClimbSubsystem.java` | (removed) | It was fully commented out |
| `utils/AllianceFlipUtil.java`, `utils/FieldConstants.java` | same names | WPILib 2027 APIs |

### Autos

PathPlannerLib's 2027 build is made for Commands v2, which cannot be used in the same project as
Commands v3, so each PathPlanner auto was converted to an `@Autonomous` opmode that drives with the
YAMS `SwerveDrive.driveToPose`, through `SwerveMechanism.driveToPose(pose, translationTolerance,
rotationTolerance)`:

- Every path in these autos was a single segment from one waypoint to the next, so each path is one
  drive to pose to the path's end, facing the path's goal rotation. The poses are written in each
  opmode, blue-origin, and flipped for the red alliance with `AllianceFlipUtil` when the step runs.
- A path end the auto drives straight on from uses a 0.3 m / 15 degree tolerance, so the robot does
  not stop there. A path end the robot shoots from, or the last one, uses 0.05 m / 3 degrees.
- The autos reset odometry to the start of their first path, as `resetOdom` did, through
  `SwerveMechanism.resetPose`.
- The named commands are called directly: `ShootBalls` is `AutoSteps.shootBalls` (shoot at the
  autonomous RPM for 4 s), `Stop` is `ShooterCommands.stopCommand`, and `wait` is `coroutine.wait`.
  `ShootBallsOdom` was registered but no auto used it.
- `StartIntake` event markers became `AutoSteps.driveIntaking`, which runs the intake between two
  fractions of the path, measured by the robot's distance to the path's end, and stops it at the end
  of the path at the latest.
- The drive config gains a translation controller with the same P of 5 as the original
  `PPHolonomicDriveController`.

### Mechanisms and commands

- Each mechanism's `periodic()` and `simulationPeriodic()` is called from `Robot`.
- Default commands, set in `Robot` for every mode:
  - The shooter holds the operator's right stick velocity, as before.
  - The indexer is stopped at the lowest priority.
- `ShooterCommands` requires the shooter and indexer and stops both when a command ends or is
  canceled, as the original `end()` methods did.
- The teleop opmode builds a YAMS `SwerveInputStream` from the driver's sticks, hands it to
  `SwerveMechanism.setInputStream`, and sets `SwerveMechanism.driveInputStream` as the drivetrain's
  default command. Autos therefore never read the sticks.
- Auto aim (`SwerveMechanism.driveAimedAt`) copies the active stream and adds YAMS aim control, so it
  translates with the teleop's sticks while facing the hub.

### Behavior differences

Beyond those listed in the `commands2` README:

- **Autos drive straight between waypoints.** PathPlanner followed Bézier curves with a velocity
  profile, rotating along the way; drive to pose drives straight to each path's end with PID and
  turns to the end heading at the same time. The paths' constraint zones (slower sections while
  intaking) are not applied, and event marker zones are approximated by distance to the path's end.
- **No auto chooser or `Index Balls` dashboard command.** Pick an autonomous opmode on the driver
  station instead.
- **No pathfinding or setpoint generator.** `driveToPose` drives straight to the pose with the YAMS
  drive-to-pose PID, and `driveWithSetpointGenerator` was removed.
- **Direct angle driving.** `HeadingTeleop` uses YAMS controller heading axes. Releasing the right
  stick stops the robot from turning instead of holding the last heading, and the original 0.5
  heading stick radius deadband is replaced by the stream's 0.1 deadband.
