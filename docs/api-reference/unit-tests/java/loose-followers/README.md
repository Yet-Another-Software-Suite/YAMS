# loose-followers

Loosely coupled followers following, stopping and switching slot with their leader. Each runs its own closed loop on the motor controller or on the SystemCore.

## Test classes

| Test class                                                     | Tests | What it checks                                                                                                                                                          |
| -------------------------------------------------------------- | ----- | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| [LooselyCoupledFollowerTest](loosely-coupled-follower-test.md) | 2     | Checks that a loosely coupled follower follows, stops and restarts with its leader across six leader and follower pairs, and switches closed loop slot with its leader. |

## Run the group

From the `vendordep` folder:

```bash
./gradlew test --tests "yams.mechs.LooselyCoupledFollowerTest"
```
