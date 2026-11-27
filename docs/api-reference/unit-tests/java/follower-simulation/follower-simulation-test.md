# FollowerSimulationTest

**Group:** [follower-simulation](./)\
**Source:** `vendordep/src/test/java/yams/mechs/FollowerSimulationTest.java`

Checks how followers are simulated, so a mechanism has the torque of every motor on it and the battery counts each motor's current once. It covers A301s following an A301, and a mechanism simulating its motor controller's loosely coupled followers.

## Setup

* Most tests use a simulated pivot on a 60:1 gearbox with 1 lb at 4 inches, a kP of 40 and brake mode. A301 tests use an open loop config on the A301's 64:1 gearbox.
* A follower "reads the leader's simulation" when its sim supplier is a `FollowerSimSupplier`. The battery "counts" a motor controller when adding 100 A under its ID lowers the `BatterySim` voltage.
* Each test starts with a reset battery and a 12 V input voltage.

## Tests

| Test                                                         | What it checks                                                                                                                                                                                                                                                                                                                                                                               |
| ------------------------------------------------------------ | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `a301FollowersAreInTheLeadersMotorAndSimulation`             | An A301 with an inverted A301 follower has twice one A301's stall torque and the same free speed. The follower reads the leader's simulation, is sent the leader's 6 V inverted, reads the same position after 25 steps, draws half of the current, and is not counted by the battery. Once the leader is closed, the follower simulates itself and is counted again.                        |
| `onlyAnA301WrapperCanFollowAnA301`                           | Giving an A301 a bare `A301` device as a follower, or a follower that already follows another A301, throws `SmartMotorControllerConfigurationException`.                                                                                                                                                                                                                                     |
| `mechanismSimulatesItsLooseFollowers`                        | A SPARK MAX with a loosely coupled SPARK MAX follower has a mechanism motor of two NEOs, while its own motor stays one NEO. The follower reads the pivot's simulation, the pivot reaches 45° within 3°, the follower's encoder is within 1° of the pivot, and the battery does not count the follower.                                                                                       |
| `followerSimIterateDoesNotStepTheMechanism`                  | While a pivot is moving, calling a loosely coupled follower's `simIterate()` 50 times does not change the pivot's position or velocity, and the battery does not count the follower. Runs with a follower of each of SPARK MAX, TalonFX, Nova and A301, led by a SPARK MAX.                                                                                                                  |
| `looseFollowerWithItsOwnFollowerSimulatesEveryMotorOfSparks` | Two SPARK MAXes, each with a SPARK MAX follower and a 40 A limit, the second a loose follower of the first, give a mechanism motor of four NEOs. Run to 45°, the pivot angle every 100 ms and the battery voltage match a single SPARK MAX with four NEOs and an 80 A limit. The loose follower draws half of the current, is not counted by the battery, and its encoder follows the pivot. |
| `looseFollowerWithItsOwnFollowerSimulatesEveryMotorOfA301s`  | Two A301s, each followed by an A301, the second a loose follower of the first, give a mechanism motor of four A301s, and every follower reads the pivot's simulation. Driven at 3 V, the pivot angle and battery voltage match one A301 followed by three. Each gearbox follower draws a quarter of the current and none of the followers are counted by the battery.                        |
| `looseFollowerWithoutAMechanismSimulatesItself`              | With no mechanism, as with two motors on separate shafts, a loosely coupled SPARK MAX follower simulates itself and the battery counts it.                                                                                                                                                                                                                                                   |
| `looseFollowerWithItsOwnMechanismSimulatesItself`            | When the follower is given a pivot of its own, it simulates itself and the battery counts it.                                                                                                                                                                                                                                                                                                |
| `mechanismMotorGivesAFollowersTorqueAtTheLeadersRotor`       | A follower geared 30:1 under a leader geared 60:1 adds half a NEO's torque at the leader's rotor: 1.5 times a NEO's stall torque, twice its stall current, and one NEO's free speed.                                                                                                                                                                                                         |
| `a301WithFollowersInAMechanism`                              | An A301 with an A301 follower on a pivot has a mechanism motor of two A301s. The follower reads the pivot's simulation and follows its position at 6 V, and the battery does not count it.                                                                                                                                                                                                   |

## Run it

From the `vendordep` folder:

```bash
./gradlew test --tests "yams.mechs.FollowerSimulationTest"
```
