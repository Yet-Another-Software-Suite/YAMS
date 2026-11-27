# VoltageLiveTuningTest

**Group:** [live-tuning](./)\
**Source:** `vendordep/src/test/java/yams/mechs/VoltageLiveTuningTest.java`

Checks the tuning voltage of a motor controller's "Live Tuning" command, whether the closed loop runs on the motor controller or on the SystemCore. The voltage must drive the leader and its loosely coupled follower, then hand control back to the closed loop when it is turned off or when Live Tuning ends.

## Setup

* A leader and a loosely coupled follower each drive their own simulated `Pivot` (60:1 gearing, kP = 40, `HIGH` telemetry), starting at 0 degrees.
* Runs over five pairs: SPARK Flex leading a SPARK MAX, TalonFX leading a SPARK MAX, SPARK MAX with an exponential profile leading a Nova, SPARK Flex leading an A301, and A301 leading a SPARK MAX.
* Each test first holds both pivots at 0 degrees, then schedules the leader's "Live Tuning" command and uses its `Tuning` table entries.

## Tests

| Test                                                           | What it checks                                                                                                                                                                                                                                                                                                                            |
| -------------------------------------------------------------- | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `voltageDrivesTheLeaderAndFollowerAndHandsBackToTheClosedLoop` | Turning `voltage/enabled` on sets `voltage/setpoint` back to 0 and both pivots stay within 3 degrees of 0. Typing 2 V moves both past 10 degrees. Turning the voltage off brings both back within 3 degrees of 0. Cancelling Live Tuning with the voltage on turns `voltage/enabled` off and both pivots return to within 3 degrees of 0. |
| `voltageSetpointIsHeldWithoutChanging`                         | After 1 V is typed once, the leader and follower both output 1 V (within 0.1 V) at every sample for 2 seconds.                                                                                                                                                                                                                            |

## Run it

From the `vendordep` folder:

```bash
./gradlew test --tests "yams.mechs.VoltageLiveTuningTest"
```
