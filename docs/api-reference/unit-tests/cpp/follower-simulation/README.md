# follower-simulation

How followers are simulated: A301s following an A301, and a motor controller's loosely coupled followers in its mechanism's motor model.

## Test files

| Test file                                             | Tests | What it checks                                                                                                                                        |
| ----------------------------------------------------- | ----- | ----------------------------------------------------------------------------------------------------------------------------------------------------- |
| [FollowerSimulationTest](follower-simulation-test.md) | 4     | Checks that A301 followers and loosely coupled followers are added to the leader's motor model and that the battery counts each motor's current once. |

## Run the group

From the `vendordep` folder:

```bash
./gradlew catchTests "-PcatchFilter=[FollowerSimulation]"
```
