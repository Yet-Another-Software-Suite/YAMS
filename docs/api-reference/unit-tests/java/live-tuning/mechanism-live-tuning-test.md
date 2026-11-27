# MechanismLiveTuningTest

**Group:** [live-tuning](./)\
**Source:** `vendordep/src/test/java/yams/core/telemetry/MechanismLiveTuningTest.java`

A mechanism with a telemetry name sets up its motor's telemetry itself. This class checks that the mechanism registers the motor's "Live Tuning" command when the motor's telemetry verbosity is `HIGH`, and not when it is lower.

## Setup

* Each case drives a simulated SPARK MAX with a NEO, a 12:1 gearing, a 40 A stator limit and closed loop control.
* The mechanisms are an `Arm`, an `Elevator`, a `Pivot` and a `FlyWheel`, each with `HIGH` mechanism telemetry.

## Tests

| Test                                    | What it checks                                                                                                                                                                                                                         |
| --------------------------------------- | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `mechanismTelemetryRegistersLiveTuning` | After the mechanism is created, the "Live Tuning" command exists for the motor's subsystem only when the motor's telemetry is `HIGH`. Runs on Arm, Elevator, Pivot and FlyWheel, each with `HIGH` and `LOW` motor telemetry (8 cases). |

## Run it

From the `vendordep` folder:

```bash
./gradlew test --tests "yams.core.telemetry.MechanismLiveTuningTest"
```
