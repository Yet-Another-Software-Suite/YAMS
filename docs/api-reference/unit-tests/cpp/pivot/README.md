# pivot

A pivot on every motor controller, plus hard stops, loosely coupled followers, the starting position and continuous wrapping.

## Test files

| Test file                  | Tests | What it checks                                                                                                                                                                |
| -------------------------- | ----- | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| [PivotTest](pivot-test.md) | 8     | Checks pivot duty cycle and position control on every motor controller, plus hard stops, loosely coupled followers, the simulation starting position and continuous wrapping. |

## Run the group

From the `vendordep` folder:

```bash
./gradlew catchTests "-PcatchFilter=[PivotTest]"
```
