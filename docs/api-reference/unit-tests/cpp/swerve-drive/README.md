# swerve-drive

The four module swerve drive, and the SwerveInputStream aiming and telemetry that run on it.

## Test files

| Test file                               | Tests | What it checks                                                                                                                    |
| --------------------------------------- | ----- | --------------------------------------------------------------------------------------------------------------------------------- |
| [SwerveDriveTest](swerve-drive-test.md) | 65    | Checks a simulated four module TalonFX swerve drive, plus SwerveInputStream telemetry, live tuning and aim options on that drive. |

## Run the group

From the `vendordep` folder:

```bash
./gradlew catchTests "-PcatchFilter=[SwerveDriveTest],[SwerveInputStreamAimTest],[SwerveInputStreamTelemetryTest]"
```
