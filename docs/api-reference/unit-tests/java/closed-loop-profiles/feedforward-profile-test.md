# FeedforwardProfileTest

**Group:** [closed-loop-profiles](./)\
**Source:** `vendordep/src/test/java/yams/core/motorcontrollers/FeedforwardProfileTest.java`

Checks that every motor controller follows a trapezoidal profile from each closed loop slot, given the same gains in YAMS units. Each feedforward is worked out from the simulated physics, so a feedforward in the right units keeps the mechanism on the profile with little left for kP to fix.

## Setup

* Uses the shared `ClosedLoopCases` helper: every slot of SPARK MAX, SPARK Flex, TalonFX, TalonFXS, Nova and A301.
* The slots that are not selected get four times the gains, so running another slot's gains, or a gain in the wrong units, pulls the mechanism off the profile.
* Each move holds the start for half a second, then runs for the profile's length plus one second. The mechanism must stay within four loops of travel at cruise velocity of the profile, and end within half that of the goal.
* The simulated battery is reset to 12 V.

## Tests

| Test                     | What it checks                                                                                                                                                                                                  |
| ------------------------ | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `rotaryFollowsProfile`   | A rotating mechanism with no gravity (0.1 kg m², `SimpleMotorFeedforward`, kP = 10) follows a 1 rotation per second, 4 rotations per second squared profile from 0 to 1.5 rotations.                            |
| `armFollowsProfile`      | An arm under gravity (0.1 kg m², 0.5 m long, `ArmFeedforward`, kP = 10) in a `SingleJointedArmSim` follows a 0.5 rotation per second, 2 rotations per second squared profile from -30 to 90 degrees.            |
| `elevatorFollowsProfile` | A 10 kg elevator with a 0.02 m drum (`ElevatorFeedforward`, kP = 40) in an `ElevatorSim` follows a 0.4 m/s, 1 m/s² profile from 0.1 m to 0.6 m. Skipped for the A301, whose motor cannot lift it at that speed. |

## Run it

From the `vendordep` folder:

```bash
./gradlew test --tests "yams.core.motorcontrollers.FeedforwardProfileTest"
```
