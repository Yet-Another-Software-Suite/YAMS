# hard-stops

A pivot driven into its hard stops, and a Talon set to a position, on every motor controller.

## Test classes

| Test class                        | Tests | What it checks                                                                                                                                                                          |
| --------------------------------- | ----- | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| [HardStopTest](hard-stop-test.md) | 4     | Checks that a simulated pivot stays on its hard stops on every motor controller, that a Talon keeps a set encoder position, and that starting positions and continuous wrapping behave. |

## Run the group

From the `vendordep` folder:

```bash
./gradlew test --tests "yams.mechs.HardStopTest"
```
