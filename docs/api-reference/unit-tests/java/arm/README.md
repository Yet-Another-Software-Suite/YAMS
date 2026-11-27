# arm

An arm and its motor controller on every motor controller, with and without motion profiles.

## Test classes

| Test class             | Tests | What it checks                                                                                                                                                                               |
| ---------------------- | ----- | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| [ArmTest](arm-test.md) | 5     | Checks that an arm and its motor controller move under duty cycle and position commands on all six motor controllers with three profile setups, and that an arm has no measurement setpoint. |

## Run the group

From the `vendordep` folder:

```bash
./gradlew test --tests "yams.mechs.ArmTest"
```
