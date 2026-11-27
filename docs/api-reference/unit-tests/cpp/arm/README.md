# arm

An arm under duty cycle and position control on every motor controller and profile.

## Test files

| Test file              | Tests | What it checks                                                                                                                                  |
| ---------------------- | ----- | ----------------------------------------------------------------------------------------------------------------------------------------------- |
| [ArmTest](arm-test.md) | 4     | Checks that a single jointed arm moves under duty cycle and position control on every motor controller and profile, directly and through `Arm`. |

## Run the group

From the `vendordep` folder:

```bash
./gradlew catchTests "-PcatchFilter=[ArmTest]"
```
