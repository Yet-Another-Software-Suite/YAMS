# cpptest — YAMS C++ Example Robot

A complete FRC robot project demonstrating the YAMS C++ library. It covers positional
mechanisms (Arm, Pivot/Turret, Pivot/Hood, Elevator), a velocity mechanism (FlyWheel/Shooter
with a double-flywheel variant), a swerve drivetrain, and a differential drivetrain — all
built on `SmartMotorControllerConfig` and the YAMS mechanism API.

## Subsystems

### TurretSubsystem

A **Pivot** mechanism for a rotating turret assembly.

- Motor: TalonFX (CAN ID 12), KrakenX60
- Gearing: 144:15 × 5 × 1.08 reduction
- Feedforward: Arm (`kS=0.5`, `kG=5.0`)
- Commands: `TurretCmd(dutyCycle)`, `SetAngle(degree_t)`, `SysId()`
- Utility: `GetPose(robotPose)` and `GetVelocity(robotVelocity, robotAngle)` account for robot
  translation/rotation when computing field-relative turret state.

### ArmSubsystem

A **Arm** mechanism for a single-jointed arm.

- Motor: TalonFX (CAN ID 1), KrakenX60; CANcoder absolute encoder (ID 2)
- Gearing: 3:1 × 4:1
- Arm length: 0.135 m; soft limits −30° to +100°; hard limits −100° to 200°
- Profile: Trapezoid
- Sensor: digital beam-break on DIO 0
- Commands: `ArmCmd(dutyCycle)`, `SetAngle(degree_t)`, `SysId()`

### ElevatorSubsystem

An **Elevator** mechanism for vertical linear motion.

- Motor: SparkMax brushless (ID 2), NEO
- Gearing: 3:1 × 4:1; chain pitch 0.25 in, 22 teeth (circumference ≈ 0.1397 m)
- Height range: 0 m – 3 m; starting height: 0.5 m
- Profile: Exponential
- Commands: `ElevCmd(dutyCycle)`, `SetHeight(meter_t)`, `SysId()`

### ShooterSubsystem

A **FlyWheel** mechanism with a 4-inch (0.1016 m) diameter roller.

- Motors: 2× TalonFX (IDs 1, 2), NEO configuration
- Gearing: 3:1 × 4:1; closed-loop velocity control, coast mode
- Commands: `SetVelocity(degrees_per_second_t)`, `SetVelocity(supplier)`, `Set(dutyCycle)`
- Queries: `GetVelocity()`, `ReadyToShoot(tolerance)`

### HoodSubsystem

A **Pivot** mechanism for a shooter hood angle adjustment.

- Motor: TalonFXS (CAN ID 9), NEO (via TalonFXS wrapper)
- Gearing: 3:1 × 4:1; soft limits −30° to +100°; hard limits −100° to 200°
- Profile: Trapezoid
- Commands: `HoodCmd(dutyCycle)`, `SetAngle(degree_t)`, `SysId()`

### DoubleFlyWheelSubsystem

Independent velocity control for two stacked flywheels (e.g. top/bottom shooter rollers).

- Motors: 2× TalonFX — lower (ID 4), upper (ID 6), KrakenX60
- Gearing: 3:1 × 4:1; MOI: 0.00029264 kg·m²; coast mode
- Commands: `SetDutyCycle(lower, upper)`, `SetVoltage(lower, upper)`, `SetVelocity(lower, upper)`,
  `SetVelocity(lowerSupplier, upperSupplier)`

### SwerveSubsystem

A swerve drivetrain using four modules and a Pigeon 2 IMU (ID 14).

Per module — drive: SparkMax NEO; azimuth: SparkMax NEO; absolute encoder: CANcoder.

| Module      | Drive ID | Azimuth ID | Encoder ID |
|-------------|----------|------------|------------|
| Front-left  | 1        | 2          | 3          |
| Front-right | 4        | 5          | 6          |
| Back-left   | 7        | 8          | 9          |
| Back-right  | 10       | 11         | 12         |

- Drive gearing: 12:1 × 2:1; azimuth gearing: 21:1
- Wheel diameter: 4 in (circumference ≈ 0.1257 m)
- Module positions: ±24 in side, +24 in / −24 in front/back from centre
- Translation and rotation PID: `kP = 1.0`
- Commands: `DriveRobotRelative(supplier)`, `SetRobotRelativeChassisSpeeds()`,
  `DriveToPose()`, `Lock()`
- Queries: `GetPose()`, `GetFieldOrientedChassisSpeed()`, `GetGyroAngle()`

### DiffDriveSubsystem

A tank-drive differential drivetrain.

- Motors: 2× main SparkMax (IDs 21, 22) + 2× followers (IDs 23, 24), NEO; left side inverted
- Gearing: 3:1 × 4:1; wheel diameter: 4 in (circumference ≈ 0.1257 m); open-loop, coast mode
- Commands: `TankDrive(leftSupplier, rightSupplier)`, `ArcadeDrive(xSupplier, rotSupplier)`, `Stop()`
- Default command: `Stop()`

## Vendor dependencies

| Library           | Purpose                               |
|-------------------|---------------------------------------|
| Phoenix6          | TalonFX, TalonFXS, CANcoder, Pigeon 2 |
| REVLib            | SparkMax brushless motor controllers  |
| ReduxLib          | NetworkTables data compression        |
| ThriftyLib        | Utility helpers                       |
| StudicaLib        | Studia hardware support               |
| WPILibNewCommands | Command-based robot framework         |

## Building and deploying

This project links directly against the YAMS library sources located at `../../yams/cpp/` — no
separate install step is needed.

```bash
# Compile for the RoboRIO
./gradlew build

# Run desktop simulation with SimGUI + Driver Station
./gradlew simulateNative

# Deploy to a connected RoboRIO (team 2381 by default)
./gradlew deploy
```

On Windows use `gradlew.bat` instead of `./gradlew`.

## RobotContainer wiring

Only the **TurretSubsystem** is active by default — all others are commented out to keep the
example focused. To enable a different subsystem, uncomment its include, instantiate it in
`RobotContainer.h`, and wire its commands in `RobotContainer.cpp`. Example bindings for every
subsystem are already present in `RobotContainer.cpp` as commented-out blocks.

Active bindings (Xbox controller, port 0):

| Button   | Action                                   |
|----------|------------------------------------------|
| 1 (held) | Turret forward at 20 % duty cycle        |
| 2 (held) | Turret reverse at −20 % duty cycle       |
| 3 (held) | Run full SysId characterisation sequence |

Default command: `TurretCmd(0.0)` — holds the turret in place at zero output.
