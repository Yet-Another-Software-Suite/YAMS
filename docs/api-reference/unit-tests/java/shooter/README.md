# shooter

A flywheel shooter and its motor controller on every motor controller, with each velocity closed loop.

## Test classes

| Test class                     | Tests | What it checks                                                                                                                                                 |
| ------------------------------ | ----- | -------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| [ShooterTest](shooter-test.md) | 4     | Checks that a flywheel shooter and its motor controller respond to duty cycle and velocity commands on all six motor controllers with four closed loop setups. |

## Run the group

From the `vendordep` folder:

```bash
./gradlew test --tests "yams.mechs.ShooterTest"
```
