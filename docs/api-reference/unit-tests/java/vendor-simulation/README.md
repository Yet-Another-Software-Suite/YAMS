# vendor-simulation

The motor controllers with a vendor simulation (REV's SparkSim and Phoenix's) simulated through it: their encoders, applied output, current limits and closed loops.

## Test classes

| Test class                                        | Tests | What it checks                                                                                                                                                                               |
| ------------------------------------------------- | ----- | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| [VendorSimulationTest](vendor-simulation-test.md) | 4     | Checks that SPARK and Talon vendor simulations drive the pivot's encoder, output, current limit and closed loop, with no follower, a tightly coupled follower or a loosely coupled follower. |

## Run the group

From the `vendordep` folder:

```bash
./gradlew test --tests "yams.mechs.VendorSimulationTest"
```
