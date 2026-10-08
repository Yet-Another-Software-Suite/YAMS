# BroncBotz 3481 FRC2025 (YAMS port, Commands v3)

A port of BroncBotz 3481's 2025 Reefscape robot code (the `comp` branch) to WPILib 2027, Commands v3 and YAMS. The original repository has no license file; check with the team before reusing this outside YAMS.

## Original source

- Repository: https://github.com/BroncBotz3481/FRC2025, branch `comp`

## What changed

### Layout and framework

- The package changed from `frc.robot` to `first.robot`. `Main.java` moved to `first/Main.java`.
- It was migrated to the WPILib 2027 APIs (`org.wpilib.*`) and to Commands v3 (`org.wpilib.command3.*`):
  - Robot: `OpModeRobot` with `@Teleop` and `@Autonomous` opmodes in place of `TimedRobot` and `RobotContainer`.
  - Controllers: `CommandNiDsXboxController`, with the D-pad reached through `getHID().povLeft()` and so on.
  - Driver station: `MatchState`.
  - Renamed members: `Rotation2d.ZERO`, `ChassisVelocities`, `Color.RED` and so on.
  - Dashboard: the scheduler is logged with `Telemetry.log`.
- Every CAN device is on `CANPort.CAN_S0`, because Systemcore has no "rio" bus.
- Vendordeps:
  - YAGSL, YALL, PathPlannerLib, maple-sim, Phoenix 5, Studica and ThriftyLib are no longer used.
  - ReduxLib 2027 is kept for the Canandgyro.
  - LimelightLib replaces YALL.
  - Grapple's LaserCAN library has no 2027 build. See `util/DistanceSensor` below.
- `deploy/launchpad/launch.py`, the driver station program for the Launchpad operator board, is unchanged. The YAGSL `deploy/swerve` files and the PathPlanner files were removed.

### Files

| Original | Port | Notes |
| --- | --- | --- |
| `HWMap.java`, `AListOfIDS`, YAGSL module JSON | `Ports.java` | CAN IDs and analog encoder channels |
| `Constants.java`, `Setpoints.java`, `AlignmentConstants.java`, YAGSL JSON | `Constants.java` | One place for gains, setpoints and limits |
| `RobotMath.java` | (removed) | YAMS gearing does the unit conversions |
| `subsystems/SwerveSubsystem.java` (YAGSL) | `mechanisms/Swerve.java`, `mechanisms/Vision.java` | Rewritten on YAMS `SwerveDrive`. Vision is a helper the drivetrain updates |
| `subsystems/ElevatorSubsystem.java` | `mechanisms/Elevator.java` | YAMS `Elevator` |
| `subsystems/CoralArmSubsystem.java`, `AlgaeArmSubsystem.java` | `mechanisms/CoralArm.java`, `AlgaeArm.java` | YAMS `Arm`s |
| `subsystems/CoralIntakeSubsystem.java` | `mechanisms/CoralIntake.java` | YAMS `Pivot` (wrist) and `FlyWheel` (roller) |
| `subsystems/AlgaeIntakeSubsystem.java` | `mechanisms/AlgaeIntake.java` | YAMS `FlyWheel` |
| `subsystems/LaserCanSim.java` | `util/DistanceSensor.java` | Stand-in for the LaserCANs, simulated with a YAMS `Sensor` |
| `subsystems/ClimberSubsystem.java`, `FloorIntakeSubsystem.java` | (removed) | Deprecated and unused in the original |
| `systems/TargetingSystem.java` | `util/ReefTargeting.java` | Branch, level and side selection and the scoring poses |
| `systems/ScoringSystem.java`, `LoadingSystem.java` | `commands/SuperstructureCommands.java` | Coroutine commands without requirements of their own; only the commands that were bound or used in autonomous |
| `controllers/Launchpad.java`, `ButtonColours.java` | `util/Launchpad.java` | Same NetworkTables protocol. The colours moved in, and `bind(x, y, colour, command)` replaces the paired LED and trigger calls |
| `systems/field/FieldConstants.java`, `AllianceFlipUtil.java` | `util/FieldConstants.java`, `util/AllianceFlipUtil.java` | Only the reef positions are kept |
| `utils/ProfiledHolonomicDriveController.java` | (removed) | Replaced by YAMS drive to pose with a speed cap |
| `Robot.java`, `RobotContainer.java` | `Robot.java` | An `OpModeRobot` that holds the mechanisms, controllers and default commands |
| `RobotContainer` bindings | `opmodes/teleop/AngularVelocityTeleop.java`, `HeadingTeleop.java`, `TeleopBindings.java` | Each teleop builds the driver's `SwerveInputStream`, sets it on the drivetrain, and makes driving from it the drivetrain's default command. The shared bindings (slow mode, alliance relative toggle, operator and Launchpad) are in `TeleopBindings` and are created with the opmode |
| `RobotContainer.justCoralL4Auto` | `opmodes/auto/CoralL4Auto.java` | Only the routine the robot ran |

### Mechanisms

Every motor is a NEO on a SPARK MAX wrapped in a YAMS `SparkWrapper`. Each is a Commands v3 `Mechanism` with HIGH verbosity YAMS telemetry, updated from `Robot.robotPeriodic()` and simulated from `Robot.simulationPeriodic()`. The original's SysId routines, SmartDashboard numbers and `Mechanism2d` side view were removed. YAMS telemetry covers the same values.

- **Swerve**
  - Built from the YAGSL files: modules at ±12 in, drive 6.12:1 on 4 in wheels, angle 12.8:1, drives inverted, 40 A drive and 20 A angle current limits, 0.25 s ramps, and the same absolute encoder offsets.
  - The YAGSL gains were converted to YAMS units. Drive kP went from per m/s to per wheel rotation per second, and angle kP from per degree to per rotation. Drive kV is 12 V at NEO free speed.
  - The Thrifty absolute encoders are read as WPILib `AnalogEncoder`s and seed the angle motors' encoders.
  - The gyro is the Canandgyro (CAN 25), through ReduxLib 2027.
  - The 7 m/s maximum speed is kept. The robot starts at (10, 4), as in the original.
- **Vision**
  - The Limelight MegaTag1 estimate goes into the pose estimator with the original's camera pose, reef tag filter and standard deviations (0.05, 0.05, 0.022).
  - Estimates more than 0.5 m from odometry are only accepted after 10 rejections in a row, as in the original.
- **Elevator** (CAN 13 and 14)
  - A YAMS `Elevator`. The right motor follows inverted.
  - The original's ProfiledPIDController and feedforward gains (kP 33.966, kD 9.4456, kS, kG, kV) and its 1 m/s, 0.5 m/s² profile are kept.
  - Gearing 12:1 onto a 22 tooth, 1/4 in pitch sprocket. Hard limits are 0 to 30 in, with a 16 lb carriage in simulation.
  - L1 and the human player station still lower the carriage onto the bottom stop at -10% duty cycle.
- **CoralArm** (CAN 15) and **AlgaeArm** (CAN 16)
  - YAMS `Arm`s with the original's gains, converted from per motor rotation to per arm rotation (112:1).
  - Limits, starting angles, 40 A limits, 0.5 s ramps and inversions are kept.
  - The through bore encoder seeds the motor encoder through `withExternalEncoder`, with the original's horizontal offsets. This replaces `synchronizeAbsoluteEncoder()`.
- **CoralIntake** (CAN 17 wrist, CAN 18 roller)
  - The wrist is a YAMS `Pivot` that closes its loop on the through bore encoder (kP 1, wrapping over [0, 1) rotation). Rest is 0.60 and active is 0.35.
  - The roller is an open loop YAMS `FlyWheel` with the original's duty cycles.
- **AlgaeIntake** (CAN 19)
  - An open loop YAMS `FlyWheel` with the original's ±0.8 and 0.2 hold duty cycles.

### Commands, bindings and autos

- **Driver**
  - Two teleop opmodes. In `Angular Velocity Teleop` the left stick translates and the right stick's X axis rotates, as in the original. In `Heading Teleop` the left stick translates and the robot faces the direction the right stick is pushed.
  - Driving is field relative. Left bumper slows translation from 0.8 to 0.4.
  - X and Y turn alliance relative control on and off. It starts off in each teleop.
  - A and B re-read an arm's absolute encoder and swing that arm to -40°.
- **Operator** (port 4): the original bindings.
  - A/B/X/Y: coral L1 to L4.
  - Bumpers: outtake coral and algae. Triggers: intake from the human player and intake algae.
  - D-pad: net, processor, and the low and high reef algae. Start: stow both arms.
- **Launchpad** (ports 1 to 3): the original's competition layout.
  - Coral level selection, branch side, auto score, and algae load and score.
  - Manual intake, roller and elevator pads, and arm presets.
  - Loaded indicators.
- **Autonomous**
  - The comp branch registered PathPlanner named commands and built an auto chooser, but `getAutonomousCommand()` always returned `justCoralL4Auto(ReefBranch.H)`. That routine only uses drive to pose.
  - It is the one routine kept, as the `Coral L4 H` opmode: lift the elevator clear, swing the coral arm out, score on L4 of branch H, then stow the arms.
  - PathPlanner is not used.
- **Coroutines:** the original's parallel groups with `until`, `withTimeout` and `withDeadline` are coroutine steps in `SuperstructureCommands`. Each step forks the mechanism commands and finishes on its condition, timeout or deadline command, which cancels what it forked.
- **WPILib 2027 alpha-7 scheduler workarounds:**
  - The autonomous routine cancels its forked elevator hold before scoring. If the scoring command interrupts a hold forked by the routine, the scheduler resolves the conflict at their common root and cancels the whole routine, leaving orphaned commands. The next conflicting command then crashes the robot loop.
  - `CoralArm.score()` holds its angle in its own loop instead of forking a hold command, for the same reason.

### Behavior differences

- **Bugs fixed:**
  - The field relative output of the driver's `SwerveInputStream` was applied as robot relative speeds, so the sticks drove relative to the robot and the alliance relative toggle flipped robot relative motion. The stream now drives the robot field relative.
  - The algae net shot waited for the processor angle, so it always ran its full 5 s timeout. It now waits for the net angle.
  - `algaeScored()` returned true without a LaserCAN reading, which ended outtaking at once. It now returns false.
  - The targeting system started without a level, so scoring waited out its timeout. It now starts at L4.
  - The closest branch was cached the first time it was looked up, so a later alliance change was missed. It is now looked up each time.
  - Selecting L2 or L1 on the Launchpad lit pads (0, 7) and (0, 8), which overwrote the loaded indicators. Only the L4 and L3 highlights remain.
- **Profiles:** the arms' profile constants were labeled 10 deg/s and 20 deg/s, but RPM values were fed into a controller working in rotations per second. The effective 600 deg/s and 1200 deg/s are kept.
- **Drive to pose:**
  - The heading tolerance is 1° instead of 0.5°. With 0.5°, the robot settled just outside the tolerance in simulation and never finished driving.
  - Speeds are capped at 1.3 m/s and 90 deg/s, as before, but without the original's acceleration limits.
- **No LaserCAN:**
  - Grapple has no 2027 library. On a real robot the coral and algae sensors report nothing, so "loaded" is always false and nothing is held.
  - The elevator no longer seeds its height from a LaserCAN, so it must start at the bottom.
  - Fill in `DistanceSensor.readHardwareMillimeters()` once a library exists.
- **Simulation:**
  - The robot starts with a coral.
  - Intaking at the human player station for 1 s loads a coral, and intaking algae for 0.5 s loads an algae. Spitting either out clears it.
  - Known issue: under WPILib 2027 alpha-7 the simulated arms and elevator are not stable. The arm angle read through the SPARK external encoder in simulation can jump far outside the hard limits. The autonomous routine usually still reaches branch H and scores, but not every run.
- **Removed:**
  - The tuning modes, SysId, the climber and floor intake, the PathPlanner autos and chooser, and the CanBridge TCP server.
  - The cycling "idiot button" colour on Launchpad pad (8, 3) and the Launchpad print pose pad.
