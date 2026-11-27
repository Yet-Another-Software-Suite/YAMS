# swerve-drive

Four module swerve drives on a Pigeon2, with many mixes of motor controllers, and SwerveInputStream aiming and telemetry for Commands v2 and v3.

## Test classes

| Test class                                                                                      | Tests | What it checks                                                                                                                                                         |
| ----------------------------------------------------------------------------------------------- | ----- | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| [SwerveDriveTest](swerve-drive-test.md)                                                         | 33    | Drives a simulated four module swerve on 9 motor controller layouts and checks driving, spinning, odometry, gyro, live tuning, closed loop placement and its commands. |
| [SwerveInputStreamAimTest (Commands v2)](swerve-input-stream-aim-test-commands2.md)             | 8     | Checks the SwerveInputStream aim options: lookahead, heading offset, feedforward, aim lock and clone.                                                                  |
| [SwerveInputStreamAimTest (Commands v3)](swerve-input-stream-aim-test-commands3.md)             | 8     | Same aim tests on Commands v3, except the aimLock trigger is not checked because a v3 trigger only updates when the scheduler polls it.                                |
| [SwerveInputStreamTelemetryTest (Commands v2)](swerve-input-stream-telemetry-test-commands2.md) | 20    | Checks what each telemetry verbosity publishes and how live tuning applies, rejects and keeps SwerveInputStream values.                                                |
| [SwerveInputStreamTelemetryTest (Commands v3)](swerve-input-stream-telemetry-test-commands3.md) | 20    | Same telemetry and live tuning tests on Commands v3, with Live Tuning run on the v3 Scheduler.                                                                         |

## Run the group

From the `vendordep` folder:

```bash
./gradlew test --tests "yams.mechs.SwerveDriveTest" \
  --tests "yams.commands2.swerve.SwerveInputStreamAimTest" \
  --tests "yams.commands2.telemetry.SwerveInputStreamTelemetryTest" \
  --tests "yams.commands3.swerve.SwerveInputStreamAimTest" \
  --tests "yams.commands3.telemetry.SwerveInputStreamTelemetryTest"
```
