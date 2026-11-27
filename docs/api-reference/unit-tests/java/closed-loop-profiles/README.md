# closed-loop-profiles

Every motor controller following a trapezoidal profile from each closed loop slot.

## Test classes

| Test class                                            | Tests | What it checks                                                                                                      |
| ----------------------------------------------------- | ----- | ------------------------------------------------------------------------------------------------------------------- |
| [FeedforwardProfileTest](feedforward-profile-test.md) | 3     | Every motor controller follows a trapezoidal profile from each slot for a rotary mechanism, an arm and an elevator. |

## Run the group

From the `vendordep` folder:

```bash
./gradlew test --tests "yams.core.motorcontrollers.FeedforwardProfileTest"
```
