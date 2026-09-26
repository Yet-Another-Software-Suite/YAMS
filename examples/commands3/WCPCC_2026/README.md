# WCP 2026 Competitive Concept (YAMS port, Commands v3)

A port of the West Coast Products 2026 Competitive Concept robot code to WPILib 2027, Commands v3 and YAMS. The original is MIT licensed; see `LICENSE-WCP`.

## Original source

- Repository: https://github.com/wcpllc/2026CompetitiveConcept

## What changed

### Layout and framework

- The package changed from `frc.robot` to `first.robot`. `Main.java` moved to `first/Main.java`.
- `frc.util` was removed. Only `GeometryUtil` remains, now at `first.robot.util.GeometryUtil`.
- The sources now live in `java/` and `deploy/`. The Choreo project and `OutpostAndDepotTrajectory.traj` are unchanged, except that the project's codegen no longer includes the ChoreoLib helpers (`"useChoreoLib": false`).
- It was migrated to the WPILib 2027 APIs (`org.wpilib.*`) and to Commands v3 (`org.wpilib.command3.*`):
  - Robot: `OpModeRobot` with `@Teleop` and `@Autonomous` opmodes in place of `TimedRobot` and `RobotContainer`.
  - Controller: the Commands v3 `CommandNiDsXboxController`, with the D-pad reached through `getHID().povUp()/povDown()`.
  - Driver station: `MatchState`/`RobotState`.
  - Renamed members: `Rotation2d.ZERO`, `ChassisVelocities` and so on.
  - Dashboard: the scheduler is logged with `Telemetry.log`; other entries use `Tunables`.
  - Brownout: `setBrownoutVoltages(6.1 V, 6.6 V)`.
- The rio CAN bus is now `new CANBus(CANPort.CAN_S0)`, because Systemcore has no "rio" bus. The CANivore is still `"main"`.
- Vendordeps: CTRE's swerve API is no longer used. ChoreoLib 2027 and LimelightLib replace ChoreoLib 2026 and `LimelightHelpers`. Only ChoreoLib's trajectory loading (`choreo.Choreo`, `Trajectory`, `SwerveSample`) is used, because its `choreo.auto` classes are built on Commands v2.

### Files

| Original | Port | Notes |
| --- | --- | --- |
| `Robot.java`, `RobotContainer.java` | `Robot.java` | An `OpModeRobot` that holds the mechanisms, the controller and the default commands |
| `RobotContainer.configureBindings()` | `opmodes/teleop/DriverTeleop.java` | The teleop bindings, created when the opmode is selected |
| `commands/AutoRoutines.java` | `opmodes/auto/OutpostAndDepotAuto.java` | One `@Autonomous` opmode per routine, replacing the `AutoChooser` |
| `subsystems/*.java` | `mechanisms/*.java` | Each subsystem is a Commands v3 `Mechanism` |
| `subsystems/Intake.java` | `mechanisms/IntakePivot.java`, `mechanisms/IntakeRollers.java` | Split so each mechanism can be live tuned on its own. `intakeCommand()` moved to `MechanismCommands.intake()`, and agitating is part of feeding in `MechanismCommands` |
| `subsystems/Swerve.java` (`TunerSwerveDrivetrain`) | `mechanisms/Swerve.java` | Rewritten on YAMS `SwerveDrive`, with its own Choreo trajectory follower |
| `generated/TunerConstants.java` | (removed) | IDs moved to `Ports`; gains, ratios and offsets moved to `Constants.SwerveConstants` |
| `commands/SubsystemCommands.java` | `commands/MechanismCommands.java` | Multi-mechanism coroutine commands |
| `commands/ManualDriveCommand.java`, `commands/AimAndDriveCommand.java` | `commands/Drive.java` | Merged into one teleop drive loop, `Drive.teleop(swerve, controller)`, that drives through the `Swerve` input stream every loop and aims while the right trigger is held. `Drive.autoAim(swerve)` is the autonomous aim loop |
| `commands/PrepareShotCommand.java` | `commands/ShotMap.java`, `commands/MechanismCommands.java` | The shot map is a plain `ShotMap.forDistance(...)` lookup; the shot tracking loop is the `prepareShot()` factory in `MechanismCommands` |
| `LimelightHelpers.java` | (removed) | Replaced by the LimelightLib vendordep |
| `frc/util/SwerveTelemetry.java` | (removed) | Replaced by YAMS swerve telemetry |
| `frc/util/DriveInputSmoother.java`, `frc/util/ManualDriveInput.java` | (removed) | Replaced by `SwerveInputStream` |
| `frc/util/Stopwatch.java` | (removed) | Replaced by a `Debouncer` in `Drive` |
| `frc/util/GeometryUtil.java` | `util/GeometryUtil.java` | Same logic |
| `generated/ChoreoTraj.java`, `generated/ChoreoVars.java` | same names | Package change. `ChoreoTraj` was regenerated without the ChoreoLib helpers (`asAutoTraj`) |

### Mechanisms

Every motor is a TalonFX wrapped in a YAMS `TalonFXWrapper` (`DCMotor.getKrakenX60(1)`). Each mechanism has HIGH verbosity YAMS telemetry and simulation through `simIterate()`, called from `Robot.robotPeriodic()` and `Robot.simulationPeriodic()`. The original's `initSendable` telemetry and SmartDashboard sendables were removed.

- **Feeder** (CAN 13)
  - A closed-loop YAMS `FlyWheel` with kP 1 and kV 12/100 rps, as in the original.
  - Coast, 120 A stator / 50 A supply. The feed speed of 5000 RPM is kept.
- **Floor** (CAN 12) and **IntakeRollers** (CAN 11)
  - Open-loop YAMS `FlyWheel`s driven by voltage.
  - The original inversion, brake mode, current limits and voltages are kept.
- **IntakePivot** (CAN 10)
  - A YAMS `Arm`, replacing MotionMagicVoltage.
  - Gearing 50, kP 300, kV and the trapezoidal profile carried over from the Motion Magic cruise/accel.
  - Positions, homing (supply current above 6 A) and the 5° tolerance are kept.
  - New:
    - Hard limits of -10° to 110°.
    - A sim start at STOWED.
    - A sim-only `Sensor` that fakes the homing current spike.
- **Hanger** (CAN 18)
  - A YAMS `Elevator`, replacing MotionMagicVoltage per motor rotation.
  - Gearing 142:1 with a 6 in circumference. kP and kV are scaled by 142 to stay equivalent.
  - Positions, homing and the 1 in tolerance are kept.
  - New:
    - Hard limits of 0 to 7 in.
    - A 2 lb sim carriage.
    - A sim-only homing current `Sensor`.
- **Shooter** (CAN 14, 15, 16)
  - A closed-loop YAMS `FlyWheel`. The left motor leads, and the middle and right motors are attached with `withLooselyCoupledFollowers`, so each Talon still runs its own onboard velocity loop, as in the original.
  - kP 0.5, kI 2 and kV 12/100 are kept. The 0 V peak reverse voltage is set through `withVendorConfig`.
  - `isVelocityWithinTolerance` checks all three motors within 100 RPM.
  - The dashboard RPM is a `Tunables` entry, `"Shooter/Dashboard RPM"`.
- **Hood** (PWM 3 and 4)
  - WPILib 2027 removed `Servo`, so the hood drives `PWM.setPulseTimeMicroseconds` directly, mapping 0..1 onto 1000..2000 µs. It is not a YAMS mechanism.
  - The estimated-position model, clamp and tolerance are kept.
- **Limelight**
  - Uses LimelightLib (`com.limelightvision.Limelight`) with the same MegaTag2 translation + MegaTag1 rotation fusion and std devs.
  - Estimates are rejected with `!isValid()` instead of `tagCount == 0`.
  - The pose is published under `WCPCC_Limelight/<name>`.
- **Swerve**
  - The original extended CTRE's `TunerSwerveDrivetrain`. The port wraps a YAMS `SwerveDrive` with four `SwerveModule`s and the Pigeon 2 (CAN 45) as the gyro.
  - **Drive motors:** closed-loop velocity, gearing 5.891, 2 in wheel radius, brake, 120 A. kP 0.1 and kV 0.124 are scaled by the drive ratio.
  - **Steer motors:** gearing 12.1, PID(100, 0, 0.5), kS 0.1, kV 1.16, 60 A.
    - The loop closes on the CANcoder as external feedback, with the original offsets.
  - All CAN IDs and the ±10 in module locations are kept.
  - Operator perspective and `seedFieldCentric` are reimplemented in the mechanism.

### Commands, bindings and autos

- **Drive** (teleop)
  - The CTRE request state machine is replaced by a `SwerveInputStream`: 5.42 m/s, 1 rps, 0.15 deadband, alliance relative.
  - Heading hold uses `withTranslationOnly(...)` after a 0.25 s debounce, with heading PID 5.
  - The A/B/X/Y snap headings use `withHeading(...)` + `withHeadingControl(...)`.
- **Aiming** uses `SwerveInputStream.withAim(hub)`: in teleop `Drive.teleop` aims while the right trigger is held, and in autonomous `Drive.autoAim` aims in place. The aimed check uses `swerve.isFacing(hub, 5°)`.
- **MechanismCommands** adds `intake()`, and agitating the intake is part of feeding. The v2 `aimAndShoot` is `shootWhenAimed`, since the drive commands now do the aiming; its logic and timings, and those of `shootManually`, are unchanged.
- The shot map (`ShotMap`) is unchanged.
- **OutpostAndDepotAuto**: the same Choreo routine, speeds and timings.
  - The X/Y (10) and theta (7) path PID is kept.
  - Trajectories are mirrored for the red alliance and skipped while the alliance is unknown, as ChoreoLib's `AutoFactory` did.
  - The Choreo module force feedforwards are projected onto each module and passed to `setRobotRelativeChassisSpeeds(speeds, forces)`, in place of CTRE `ApplyFieldSpeeds`.
- **Bindings** are unchanged:
  - Right trigger: aim and shoot. Right bumper: manual shot.
  - Left trigger: intake. Left bumper: stow.
  - D-pad up/down: hanger.
  - A/B/X/Y: snap headings. Back: seed field centric.
  - Homing runs at auto and teleop start.

### Behavior differences

- **Stick response.** The original applied a 1.5 power curve with a radial deadband. The port cubes both axes, so the sticks feel softer.
- **Drive control.**
  - The original drove open-loop voltage with MotionMagicExpo steering and sent `SwerveRequest.Idle` with no input.
  - The port drives closed-loop velocity with PID + FF steering and always commands field-relative speeds.
- **Heading lock.** The original's rotational deadband is gone. Manual rotation is now detected from the raw stick instead of the smoothed input.
- **Not carried over from CTRE swerve:**
  - Steer/drive coupling compensation (the ratio of 4.909 is kept, but YAMS does not compensate for it yet).
  - The custom 0.1 odometry/vision std devs.
  - The high-frequency odometry thread.
  - The FPGA-time conversion of vision timestamps.
- **Dashboard keys changed.** The SmartDashboard sendables and the `DriveState/*` swerve telemetry tables are replaced by YAMS telemetry, so dashboard layouts built for the original will not match.
- **Autonomous selection.** The dashboard `AutoChooser` is gone; the routine is picked as an opmode on the driver station.
- **Hard limits and simulation are new.** The pivot and hanger have YAMS hard limits, and everything except the hood and Limelight is simulated.

## Commands v3 version

This is the Commands v3 version of `examples/commands2/WCPCC_2026`. The mechanisms, gains, bindings and timings match that port. What differs from it:

- **Mechanisms instead of subsystems.** Each v2 subsystem is a class implementing `org.wpilib.command3.Mechanism` in the `mechanisms` package, still with one YAMS mechanism each. Their `periodic()`/`simulationPeriodic()` methods are called from `Robot`, since Commands v3 has no subsystem periodic.
- **Opmodes instead of `RobotContainer`.** `Robot` extends `OpModeRobot` and sets the global defaults (manual drive, vision). `DriverTeleop` creates the bindings, and `OutpostAndDepotAuto` replaces `AutoRoutines` and its `AutoChooser`. Both schedule homing when they start and cancel it when they end, as disabling did in v2; the auto also cancels its routine.
- **Coroutines instead of decorators and groups.**
  - Single-mechanism commands (`spinUpCommand`, `positionCommand`, homing, the drive and vision loops) are coroutines with `waitUntil`/`park` or a `while (true)` loop that yields, and `whenCanceled` replaces `startEnd` and `handleInterrupt`.
  - `shootWhenAimed`, `shootManually` and `intake` require every mechanism they drive, like the v2 groups, and are written as sequential code: they set the rollers, shooter and pivot directly and `wait`/`waitUntil` between steps. Feeding (feeder, then floor rollers while rocking the intake) is a plain method both shooting commands call, so the v2 feed, floor feed and agitate commands are gone. `shootWhenAimed` does not require the swerve: it `fork`s `prepareShot()`, which sets the shooter and hood from the shot map every loop, and waits until the drive command faces the hub and the shooter and hood are at their setpoints.
  - The auto forks its trajectory followers and mechanism commands, resets odometry with a plain `Swerve.resetOdometry` call, and gives aim and shoot five seconds with a timed `waitUntil` before canceling it.
  - Driving commands take the driver controller instead of stick suppliers, and one `MechanismCommands` instance is shared by teleop and autonomous.
  - `Swerve` owns the one `SwerveInputStream`, and commands change it through `Swerve` methods (`setDriveInput`, `setAimTarget`, `setHeadingLock`, `setHoldHeading`, `driveFromInput`) every loop. `Drive.teleop` is the manual drive loop: it reads the driver controller every loop and handles the A/B/X/Y snap headings, Back (seed field centric), the rotation edge and the heading hold `Debouncer` instead of using triggers and bindings. While the right trigger is held (the shoot button, which runs `shootWhenAimed`) the same loop keeps the driver's translation and faces the hub. Autonomous has its own drive loop, `Drive.autoAim`, which the auto forks with `shootWhenAimed` for its five second shot.
- **Priority instead of interruption behavior.** Homing runs above the default priority, replacing `InterruptionBehavior.CANCEL_INCOMING`. The vision default command has the lowest priority so the auto can pause vision with `limelight.idle()`.
- **Choreo without `choreo.auto`.** ChoreoLib's `AutoFactory`, `AutoRoutine` and `AutoTrajectory` are built on Commands v2, so the auto loads the splits with `Choreo.loadTrajectory(...).getSplit(i)` and follows them with `Swerve.followTrajectory`, a coroutine that samples the trajectory each loop and holds the final sample, as `AutoTrajectory.cmd()` did. The trajectory triggers (`done()`, `doneDelayed()`, `atTime()`, `atTimeBeforeEnd()`, `active()`) became waits in one routine coroutine.
- **Shooter followers.** `setRPM()` now restarts the followers' closed loops, which `stop()` turns off. The TalonFX followers run their loops onboard, so this changes nothing on this robot, but it keeps the followers correct for controllers whose loop runs on the robot controller.
