# pivot

A pivot and its motor controller on every motor controller, with and without motion profiles.

## Test classes

| Test class                 | Tests | What it checks                                                                                                                                   |
| -------------------------- | ----- | ------------------------------------------------------------------------------------------------------------------------------------------------ |
| [PivotTest](pivot-test.md) | 4     | Checks that a pivot and its motor controller move under duty cycle and position commands on all six motor controllers with three profile setups. |

## Run the group

From the `vendordep` folder:

```bash
./gradlew test --tests "yams.mechs.PivotTest"
```
