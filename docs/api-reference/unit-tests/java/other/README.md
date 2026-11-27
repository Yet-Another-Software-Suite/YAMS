# other

Everything else: swerve config, mechanism poses, the battery simulation and the standalone math tests.

## Test classes

| Test class                                                       | Tests | What it checks                                                                                                                            |
| ---------------------------------------------------------------- | ----- | ----------------------------------------------------------------------------------------------------------------------------------------- |
| [MechanismPoseTest](mechanism-pose-test.md)                      | 20    | Checks each mechanism's 3D pose and multi stage elevator stage poses against poses worked out by hand.                                    |
| [SwerveDriveAntiTippingTest](swerve-drive-anti-tipping-test.md)  | 8     | Checks the anti tipping correction's threshold, direction, cap, how it is added to chassis speeds, and invalid settings.                  |
| [SwerveDriveGyroAxisTest](swerve-drive-gyro-axis-test.md)        | 7     | Checks that a side mounted gyro's heading, inversion, offset and tilt are rotated into the robot's frame.                                 |
| [SwerveModuleConfigTest](swerve-module-config-test.md)           | 6     | Checks swerve module optimization: quarter turn limit, hysteresis against flipping, and holding the wheel below the velocity deadband.    |
| [BatterySimTest](battery-sim-test.md)                            | 9     | Checks the simulated battery's sag, drain, charge curve, capacity derating and thread safety under heavy load from six motor controllers. |
| [MechanismBatterySimTest](mechanism-battery-sim-test.md)         | 1     | Checks that an idle Arm, Elevator, Pivot or FlyWheel simIterate does not wipe out battery sag from the rest of the robot.                 |
| [ChineseRemainderTheoremTest](chinese-remainder-theorem-test.md) | 9     | Checks EasyCRT solving a position over several rotations from two absolute encoders, including backlash, bad readings and invalid setups. |
| [RunToSupplierTest](run-to-supplier-test.md)                     | 2     | Checks that runTo with a supplier follows a changing target and ends only near the current one.                                           |

## Run the group

From the `vendordep` folder:

```bash
./gradlew test --tests "yams.core.mechanisms.*" \
  --tests "yams.core.motorcontrollers.simulation.*" \
  --tests "yams.ChineseRemainderTheoremTest" \
  --tests "yams.commands2.mechanisms.RunToSupplierTest"
```
