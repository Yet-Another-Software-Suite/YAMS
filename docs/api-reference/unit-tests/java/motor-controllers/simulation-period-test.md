# SimulationPeriodTest

**Group:** [motor-controllers](./)\
**Source:** `vendordep/src/test/java/yams/core/motorcontrollers/SimulationPeriodTest.java`

Checks that closed loop control still works when the simulation period set with `withSimulationPeriod` differs from the robot loop. The simulation steps every 10 ms while the robot loop runs every 20 ms.

## Setup

* Each motor controller gets a 12:1 gearing, kP = 8, a `SimpleMotorFeedforward(0, 1)`, a 40 A stator limit, brake mode and a 10 ms simulation period, starting at 0 degrees.
* A `PeriodicScheduler` runs `simIterate()` every 10 ms and, separately, sets the setpoint and updates telemetry every 20 ms, on a virtual clock.
* The simulated battery is reset to 12 V before each test.

## Tests

| Test                                              | What it checks                                                                                                                                                                                              |
| ------------------------------------------------- | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `testSimIterateAccurateWithSlowerTelemetryPeriod` | After 3 seconds aiming for 80 degrees, `simIterate()` has run more often than the 20 ms loop, and the mechanism is within 5 degrees of 80. Runs on SPARK MAX, SPARK Flex, TalonFXS, TalonFX, Nova and A301. |

## Run it

From the `vendordep` folder:

```bash
./gradlew test --tests "yams.core.motorcontrollers.SimulationPeriodTest"
```
