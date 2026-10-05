# 2026 Camber (YAMS port)

A port of Team 9658 Camber Robotics' 2026 robot code to WPILib 2027 and YAMS. It is a swerve
shooter robot with a distance-based shot table, Limelight pose estimation and PathPlanner autos.

## Original source

- Repository: https://github.com/9658-Camber-Robotics/2026-KitBot

## What changed

### Layout and framework

- The package changed from `frc.robot` to `first.robot`. `Main.java` moved to `first/Main.java`.
- The sources now live in `java/` and `deploy/`. The PathPlanner paths, autos and settings are
  unchanged. The YAGSL JSON files in `deploy/swerve` were removed; their values are now in
  `Constants.SwerveDrive.Modules`.
- It was migrated to the WPILib 2027 APIs (`org.wpilib.*`):
  - `CommandXboxController` became `CommandNiDsXboxController`. The D-pad is reached through
    `getHID().povUp()` and so on.
  - Driver station: `MatchState`/`RobotState`. `silenceJoystickConnectionWarning` became
    `DriverStationBackend.silenceJoystickConnectionAlert`.
  - `testInit/testPeriodic` became `utilityInit/utilityPeriodic`.
  - `SmartDashboard.putNumber` became `Telemetry.log`, and `SmartDashboard.putData` became
    `Tunables.publish`.
  - Renamed members: `ChassisVelocities`, `Rotation2d.ZERO`, `DebounceType.FALLING`,
    `Fields.FRC_2026_REBUILT_ANDY_MARK`. `MathUtil.clamp` became `Math.clamp`.
- Every device is on `CANPort.CAN_S0`, because Systemcore has no "rio" bus.
- Vendordeps:
  - YAGSL is replaced by the YAMS `SwerveDrive`.
  - YALL (`limelight.*`) is replaced by LimelightLib 2027 (`com.limelightvision.*`).
  - PathPlannerLib 2026 is replaced by PathPlannerLib `2027.0.0-alpha-4` (`PathplannerLibSystemCoreAlpha.json`).
  - ReduxLib, ThriftyLib and Studica were not used and are not included.

### Files

| Original | Port | Notes |
| --- | --- | --- |
| `Robot.java`, `RobotContainer.java` | same names | Same bindings, named commands and auto. See below |
| `Constants.java` | `Constants.java` | Same values. The swerve module values from the YAGSL JSON files are added under `SwerveDrive.Modules`; the climber constants are removed |
| `subsystems/SwerveSubsystem.java` | `subsystems/SwerveSubsystem.java` | Rewritten on YAMS `SwerveDrive`; PathPlanner and Limelight setup kept |
| `subsystems/ShooterSubsystem.java`, `subsystems/IndexerSubsystem.java` | same names | YAMS `FlyWheel`s, as before |
| `subsystems/ClimbSubsystem.java` | (removed) | It was fully commented out |
| `commands/*.java` | same names | Same logic |
| `utils/AllianceFlipUtil.java`, `utils/FieldConstants.java` | same names | WPILib 2027 APIs only |
| `deploy/swerve/*.json` | (removed) | Moved to `Constants.SwerveDrive.Modules` |

### Subsystems

- **Swerve**
  - A YAMS `SwerveDrive` with four modules, each a drive NEO and an angle NEO on SPARK MAXes, plus
    a CANcoder that seeds the angle motor's encoder. The CAN IDs, CANcoder offsets, inversions,
    gearing (5.36 drive, 18.75 angle), 4 in wheels, current limits (40 A / 20 A) and 0.25 s ramp rates
    come from the JSON files.
  - The module locations are copied as-is. In the JSON files `frontleft` sits at the back left
    corner and so on; only the module names are affected.
  - Gains converted to YAMS units:
    - Drive kP 0.0020645 per m/s, re-expressed per wheel rotation per second.
    - Drive kV from YAGSL's feedforward of 12 V at the 4.6 m/s maximum speed.
    - Angle kP 0.01 per degree, re-expressed per rotation (3.6).
    - Heading kP 0.2218 times YAGSL's maximum angular velocity (4.6 m/s over the drive base radius),
      because YAGSL scaled the heading PID output by it.
  - The navX on the roboRIO SPI port is replaced by Systemcore's `OnboardIMU`.
  - PathPlanner's `AutoBuilder` is configured as before, with PID 5/5 and the module force
    feedforwards passed to `setRobotRelativeChassisSpeeds(speeds, forces)`.
  - `driveToPose` and `driveWithSetpointGenerator` are kept on PathPlannerLib.
- **Limelight**
  - Same pipeline, camera pose, AprilTag ID filter, external IMU mode and throttle, now through
    LimelightLib.
  - MegaTag1 estimates are used when the average tag ambiguity is below 0.3 and more than one tag is
    seen, as before. The ambiguity is averaged from the raw fiducials, as YALL did.
  - YALL's `result.valid` check became `PoseEstimate.isValid()`. The temperature comes from
    `getHardwareData().cpuTempCelsius`.
- **Shooter** (CAN 4, follower CAN 41 inverted) and **Indexer** (CAN 3)
  - Same YAMS `FlyWheel` configs, still in `Constants`. Each subsystem clones its config and calls
    `withSubsystem(this)`.
  - The shooter follower TalonFX is created in `ShooterSubsystem` instead of in `Constants`.
  - YAMS no longer has `FlyWheel.set(dutyCycle)`, so `setDutycycleCommand` runs
    `setDutyCycleSetpoint` itself.

### Commands, bindings and autos

- The bindings, the shot table, the named commands (`ShootBalls`, `ShootBallsOdom`, `Stop`,
  `StartIntake`) and the auto chooser are unchanged. `getAutonomousCommand()` still runs `Left Auto`.
- **Drive**
  - `SwerveInputStream` is now the YAMS one, with the same deadband (0.1), translation scale (0.8)
    and alliance relative control.
  - `driveDirectAngle` turns to the heading the right stick points at. The YAMS stream stops
    turning once the stick is released, so the port remembers the last heading while the stick is
    inside the JSON `angleJoystickRadiusDeadband` (0.5), as YAGSL did.
- **AutoAimCommand** aims with `SwerveInputStream.withAim(...)`.

### Behavior differences

- **PathPlannerLib cannot load yet.** PathPlannerLib `2027.0.0-alpha-4` gives all ten of
  `RobotConfig`'s alerts the same ID, and WPILib 2027 alpha 7 rejects the second one, so
  `RobotConfig` fails to load. `setupPathPlanner()` catches that and reports an error, and the robot
  runs without autos or the auto chooser until PathPlanner publishes a fix. `LeftAutoTest` is skipped
  until then. The `commands3` version of this port reads the PathPlanner files itself and runs the
  autos today.
- **Gyro.** The OnboardIMU replaces the navX.
- **Dashboard keys changed.** YAGSL telemetry is replaced by YAMS swerve telemetry.
- **Not carried over:** YAGSL's odometry thread. YAMS updates odometry in `periodic()`.
