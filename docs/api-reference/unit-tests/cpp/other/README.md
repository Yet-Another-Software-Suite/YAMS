# other

Every tag no other group names: the math tests and mechanism poses. A new tag runs here until it gets its own group.

## Test files

| Test file                                   | Tests | What it checks                                                                                                                                               |
| ------------------------------------------- | ----- | ------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| [MechanismPoseTest](mechanism-pose-test.md) | 15    | Checks the 3D pose each mechanism reports against poses worked out by hand, including movement planes, turned mounts, nested mechanisms and elevator stages. |
| [LQRControllerTest](lqr-controller-test.md) | 35    | Checks LQRConfig accessors and that LQRController output has the right sign and stays within the voltage limit for flywheel, arm and elevator systems.       |
| [EasyCRTTest](easy-crt-test.md)             | 9     | Checks that EasyCRT finds a position from two absolute encoder readings across its unique range and rejects bad inputs.                                      |

## Run the group

From the `vendordep` folder: A `~` before a tag means "not": this group runs every test case without those tags.

```bash
./gradlew catchTests "-PcatchFilter=~[ArmTest]~[ElevatorTest]~[PivotTest]~[ShooterTest]~[SwerveDriveTest]~[SwerveInputStreamAimTest]~[SwerveInputStreamTelemetryTest]~[SMCParity]~[SMCRegression]~[SparkParity]~[NovaParity]~[A301Parity]~[TalonApi]~[TalonParity]~[FollowerSimulation]"
```
