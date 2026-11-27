# elevator

An elevator and its motor controller on every motor controller, with and without motion profiles.

## Test classes

| Test class                       | Tests | What it checks                                                                                                                                                                                                   |
| -------------------------------- | ----- | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| [ElevatorTest](elevator-test.md) | 5     | Checks that an elevator and its motor controller move under duty cycle and height commands on all six motor controllers with three profile setups, and that its measurement setpoint converts to drum rotations. |

## Run the group

From the `vendordep` folder:

```bash
./gradlew test --tests "yams.mechs.ElevatorTest"
```
