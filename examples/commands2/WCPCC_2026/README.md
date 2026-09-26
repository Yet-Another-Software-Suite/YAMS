# WCP 2026 Competitive Concept (YAMS port)

A port of the West Coast Products 2026 Competitive Concept robot code to WPILib 2027 and YAMS. The original is MIT licensed; see `LICENSE-WCP`.

## Original source

- Repository: https://github.com/wcpllc/2026CompetitiveConcept

## What changed

### Layout and framework

- The package changed from `frc.robot` to `first.robot`. `Main.java` moved to `first/Main.java`.
- `frc.util` was removed. Only `GeometryUtil` remains, now at `first.robot.util.GeometryUtil`.
- The sources now live in `java/` and `deploy/`. The Choreo project and `OutpostAndDepotTrajectory.traj` are unchanged.
- It was migrated to the WPILib 2027 APIs (`org.wpilib.*`):
  - Controller: `CommandNiDsXboxController`, with the D-pad reached through `getHID().povUp()/povDown()`.
  - Driver station: `MatchState`/`RobotState`.
  - Renamed members: `Rotation2d.ZERO`, `ChassisVelocities` and so on.
  - Dashboard: `Tunables.publish` in place of `SmartDashboard.putData`.
  - Brownout: `setBrownoutVoltages(6.1 V, 6.6 V)`.
- The rio CAN bus is now `new CANBus(CANPort.CAN_S0)`, because Systemcore has no "rio" bus. The CANivore is still `"main"`.
- Vendordeps: CTRE's swerve API is no longer used. ChoreoLib 2027 and LimelightLib replace ChoreoLib 2026 and `LimelightHelpers`.

### Files

| Original | Port | Notes |
| --- | --- | --- |
| `subsystems/Intake.java` | `subsystems/IntakePivot.java`, `subsystems/IntakeRollers.java` | Split so each mechanism is its own subsystem and can be live tuned on its own. `intakeCommand()`/`agitateCommand()` moved to `SubsystemCommands.intake()`/`agitate()` |
| `subsystems/Swerve.java` (`TunerSwerveDrivetrain`) | `subsystems/Swerve.java` | Rewritten on YAMS `SwerveDrive` |
| `generated/TunerConstants.java` | (removed) | IDs moved to `Ports`; gains, ratios and offsets moved to `Constants.SwerveConstants` |
| `commands/ManualDriveCommand.java` | `commands/ManualDrive.java` | Still a `Command` subclass (the drivetrain default), now driving from a YAMS `SwerveInputStream` |
| `LimelightHelpers.java` | (removed) | Replaced by the LimelightLib vendordep |
| `frc/util/SwerveTelemetry.java` | (removed) | Replaced by YAMS swerve telemetry |
| `frc/util/DriveInputSmoother.java`, `frc/util/ManualDriveInput.java` | (removed) | Replaced by `SwerveInputStream` |
| `frc/util/Stopwatch.java` | (removed) | Replaced by `Trigger.debounce` |
| `frc/util/GeometryUtil.java` | `util/GeometryUtil.java` | Same logic |
| `generated/ChoreoTraj.java`, `generated/ChoreoVars.java` | same names | Package change only |

### Subsystems

Every motor is a TalonFX wrapped in a YAMS `TalonFXWrapper` (`DCMotor.getKrakenX60(1)`). Each mechanism has HIGH verbosity YAMS telemetry and simulation through `simIterate()`. The original's `initSendable` telemetry and SmartDashboard sendables were removed.

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
  - Operator perspective and `seedFieldCentric` are reimplemented in the subsystem.

### Commands, bindings and autos

- **ManualDrive**
  - The CTRE request state machine is replaced by a `SwerveInputStream`: 5.42 m/s, 1 rps, 0.15 deadband, alliance relative.
  - Heading hold uses `withTranslationOnly(...)` after a 0.25 s debounce, with heading PID 5.
  - The A/B/X/Y snap headings use `withHeading(...)` + `withHeadingControl(...)`.
- **AimAndDriveCommand** aims with `SwerveInputStream.withAim(hub)`. `isAimed()` uses `swerve.isFacing(hub, 5°)`.
- **SubsystemCommands** adds `intake()` and `agitate()`. The `aimAndShoot`/`shootManually` logic and timings are unchanged.
- The **PrepareShotCommand** shot map is unchanged.
- **AutoRoutines**: the same Choreo routine, speeds and timings.
  - The X/Y (10) and theta (7) path PID is kept.
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
- **Hard limits and simulation are new.** The pivot and hanger have YAMS hard limits, and everything except the hood and Limelight is simulated.
