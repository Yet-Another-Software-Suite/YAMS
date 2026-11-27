# mechanism-limits

The min and max triggers of pivots, arms and elevators, and the starting position.

## Test classes

| Test class                                    | Tests | What it checks                                                                                                                                          |
| --------------------------------------------- | ----- | ------------------------------------------------------------------------------------------------------------------------------------------------------- |
| [MechanismLimitTest](mechanism-limit-test.md) | 13    | Checks pivot, arm and elevator min and max triggers, that the starting position seeds the simulation, and that starting outside the hard limits throws. |

## Run the group

From the `vendordep` folder:

```bash
./gradlew test --tests "yams.mechs.MechanismLimitTest"
```
