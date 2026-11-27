# MechanismBatterySimTest

**Group:** [other](./)\
**Source:** `vendordep/src/test/java/yams/core/motorcontrollers/simulation/MechanismBatterySimTest.java`

Checks that a mechanism's `simIterate()` leaves the shared `BatterySim` voltage in place. The battery sees every mechanism on the robot, so an idle mechanism must not reset the simulated battery to the voltage its own current draw alone would give.

## Setup

* Each mechanism runs on a SPARK MAX with a NEO, 12:1 gearing, and closed loop control, and sits idle.
* `BatterySim` is reset before and after each test.

## Tests

| Test                                     | What it checks                                                                                                                                                                                                               |
| ---------------------------------------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `simIterateKeepsTheSharedBatteryVoltage` | A 100 A load from "the rest of the robot" sags the battery by more than 1 V, and the idle mechanism's next `simIterate()` leaves the voltage within 0.5 V of that sag. Runs on an `Arm`, `Elevator`, `Pivot` and `FlyWheel`. |

## Run it

From the `vendordep` folder:

```bash
./gradlew test --tests "yams.core.motorcontrollers.simulation.MechanismBatterySimTest"
```
