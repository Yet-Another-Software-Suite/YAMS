# shooter

A flywheel shooter under duty cycle and velocity control on every motor controller.

## Test files

| Test file                      | Tests | What it checks                                                                                                                                                      |
| ------------------------------ | ----- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| [ShooterTest](shooter-test.md) | 5     | Checks that a flywheel shooter spins up under duty cycle and velocity control on every motor controller, and that an idle shooter keeps the shared battery voltage. |

## Run the group

From the `vendordep` folder:

```bash
./gradlew catchTests "-PcatchFilter=[ShooterTest]"
```
