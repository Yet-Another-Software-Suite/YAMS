# Unqualified Quokkas 2025 Wooper (YAMS port)

A port of Wooper, the Unqualified Quokkas 2025 Ri3D low cycle robot, to WPILib 2027 and YAMS. The original is MIT licensed; see `LICENSE-UQ`.

## Original source

- Repository: https://github.com/Unqualified-Quokkas/quokkas2025 (`Wooper/`)
- Only Wooper is ported. The other project in the repository, `quaxlyfr/QuaxlyFR`, is unfinished (gains and setpoints are FIXMEs) and is not included.

## What changed

- The package is now `first.robot` and the code uses the WPILib 2027 APIs.
- Every SPARK MAX is wrapped in a YAMS `SparkWrapper`. Followers are set with `withFollowers`.
- CAN IDs, inversions, brake modes, bindings, setpoints and the auto are unchanged.

| Original | Port | Notes |
| --- | --- | --- |
| `subsystems/DriveTrain.java` | `subsystems/DriveTrain.java` | Two open loop `SmartMotorController`s |
| `subsystems/Arm.java` | `subsystems/ArmSubsystem.java` | YAMS `Arm`. Renamed so it does not clash with `yams.commands2.mechanisms.Arm` |
| `subsystems/Intake.java` | `subsystems/Intake.java` | Open loop YAMS `FlyWheel` |
| `subsystems/Climber.java` | `subsystems/Climber.java` | Open loop `SmartMotorController` |
| `commands/Autos.java` | `commands/Autos.java` | Same routine |
| `Constants.java` | `Constants.java` | Arm positions typed as `Angle`; CAN IDs moved here |

## Arm

- The through bore encoder stays on DIO 0. On startup the arm waits 1 s, then seeds the motor encoder from it. YAMS then closes the loop on the motor encoder.
- `ArmConstants.kGearRatio` is a placeholder (100). Set it to your real reduction, because the seeded position depends on it.
- The PID gains are scaled by 12 in `Constants` to go from duty cycle to volts (kP 210, kD 9.6).
- The target clamp is now YAMS soft limits (0.05 to 0.422 rotations).
- The ±0.8 output clamp is not carried over. YAMS only limits closed loop voltage on SPARKs when using exponential profiles.

## Simulation estimates

Motor types (CIM for brushed motors, NEO for drive), drive gearing (8.45) and wheels (6 in), roller size, arm length and inertia are estimates. They only affect simulation and telemetry.
