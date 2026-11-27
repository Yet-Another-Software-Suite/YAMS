# elevator

An elevator under duty cycle and position control on every motor controller and profile.

## Test files

| Test file                        | Tests | What it checks                                                                                                                              |
| -------------------------------- | ----- | ------------------------------------------------------------------------------------------------------------------------------------------- |
| [ElevatorTest](elevator-test.md) | 4     | Checks that an elevator moves under duty cycle and position control on every motor controller and profile, directly and through `Elevator`. |

## Run the group

From the `vendordep` folder:

```bash
./gradlew catchTests "-PcatchFilter=[ElevatorTest]"
```
