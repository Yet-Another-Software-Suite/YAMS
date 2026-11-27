# follower-simulation

How followers are simulated: A301s following an A301, and a mechanism simulating its motor controller's loosely coupled followers, so it has every motor's torque and the battery counts each motor's current once.

## Test classes

| Test class                                            | Tests | What it checks                                                                                                                                                                               |
| ----------------------------------------------------- | ----- | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| [FollowerSimulationTest](follower-simulation-test.md) | 10    | Checks that A301 followers and loosely coupled followers are simulated inside their leader's mechanism with the right combined motor, and that the battery counts each motor's current once. |

## Run the group

From the `vendordep` folder:

```bash
./gradlew test --tests "yams.mechs.FollowerSimulationTest"
```
