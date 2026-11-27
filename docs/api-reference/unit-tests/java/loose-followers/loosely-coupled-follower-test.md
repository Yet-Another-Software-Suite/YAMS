# LooselyCoupledFollowerTest

**Group:** [loose-followers](./)\
**Source:** `vendordep/src/test/java/yams/mechs/LooselyCoupledFollowerTest.java`

Checks that a loosely coupled follower goes where its leader goes and stops when its leader stops, whether the follower's closed loop runs on the motor controller or on the SystemCore. It also checks that a follower switches closed loop slot with its leader.

## Setup

* The leader and the follower each drive their own simulated pivot (60:1 gearbox, 1 lb at 4 inches, hard limits of -180° to 180°), so the follower only moves as its own closed loop drives it.
* The config uses a kP of 40, a `SimpleMotorFeedforward` with all gains 0, a 40 A stator limit and brake mode. The Nova gets a trapezoidal profile, as it applies a feedforward only with a motion profile. The A301 runs on its 64:1 gearbox without current limits.

## Tests

| Test                                   | What it checks                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                              |
| -------------------------------------- | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `followerFollowsAndStopsWithItsLeader` | The leader is sent to 45° and the follower ends within 3° of it after 3 s. The leader is then sent back to 0° and stopped partway with a 0 duty cycle command: the follower's duty cycle stays under 0.01 for the next second, rather than driving on to 0°. Sent to 45° again, the follower follows again. Runs over six pairs: a SPARK MAX following a SPARK Flex, a Nova following a SPARK MAX, a SPARK MAX with an exponential profile following a TalonFX, a SPARK MAX with an LQR following a Nova, an A301 following a SPARK MAX, and a SPARK MAX following an A301. |
| `followerSwitchesSlotWithItsLeader`    | When the leader is switched to slot 1, slot 2 and back to slot 0, a SPARK MAX follower is on the same slot each time. Runs with a leader of each of SPARK MAX, SPARK Flex, TalonFX, TalonFXS, Nova and A301.                                                                                                                                                                                                                                                                                                                                                                |

## Run it

From the `vendordep` folder:

```bash
./gradlew test --tests "yams.mechs.LooselyCoupledFollowerTest"
```
