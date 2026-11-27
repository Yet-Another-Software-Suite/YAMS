# FollowerSimulationTest

**Group:** [follower-simulation](./)\
**Source:** `vendordep/src/test/native/cpp/FollowerSimulationTest.cpp`\
**Tags:** `[FollowerSimulation]`

Checks how followers are simulated. An A301 followed by other `A301Wrapper`s puts every A301 in its motor model, and the followers read its simulation. A motor controller's loosely coupled followers are added to its mechanism's motor model, `GetMechanismDCMotor()`. This matters because the simulated mechanism gets the torque of every motor, and the simulated battery counts each motor's current once. It mirrors the Java `FollowerSimulationTest`.

## Setup

* Every test builds `A301Wrapper`s on simulated A301s, in open loop mode, with a 64:1 gearbox unless a test says otherwise.
* The fixture resets the simulated battery before and after each test. A helper checks whether the battery counts a motor controller's current by adding 50 A for it and seeing if the battery keeps it.

## Tests

| Test                                                                      | What it checks                                                                                                                                                                                                                                                                                                                                                                 |
| ------------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| `FollowerSimulation.A301FollowersAreInTheLeadersMotorAndSimulation`       | An A301 leader with one inverted A301 follower has twice the stall torque of one A301 and the same free speed. Setting the leader to 6 V sends the follower -6 V. After 25 simulation steps the follower reads the leader's position, draws half the leader's current, and is not counted by the battery. Once the leader is destroyed, the battery counts the follower again. |
| `FollowerSimulation.OnlyAnA301WrapperCanFollowAnA301`                     | Giving an A301 leader a raw `A301` device (not an `A301Wrapper`) as a follower throws `SmartMotorControllerConfigurationException`.                                                                                                                                                                                                                                            |
| `FollowerSimulation.MechanismMotorHasTheLooseFollowers`                   | A 64:1 leader with a 32:1 loosely coupled follower has a mechanism motor with 1.5 times the stall torque of one A301 (the follower adds half an A301 at the leader's rotor), twice the stall current, and the same free speed. The leader's own `GetDCMotor()` is still one A301, and the battery counts the follower's current.                                               |
| `FollowerSimulation.LooseFollowerWithItsOwnFollowerIsInTheMechanismMotor` | A leader with one A301 follower, plus a loosely coupled follower that has its own A301 follower, has a mechanism motor equal to four A301s (stall torque, stall current, free current, resistance and Kv). The battery does not count either directly followed A301, because their current is already in their leaders'.                                                       |

## Run it

From the `vendordep` folder:

```bash
./gradlew catchTests "-PcatchFilter=[FollowerSimulation]"
```
