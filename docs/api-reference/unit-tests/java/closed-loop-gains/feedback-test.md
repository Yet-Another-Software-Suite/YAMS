# FeedbackTest

**Group:** [closed-loop-gains](./)\
**Source:** `vendordep/src/test/java/yams/core/motorcontrollers/FeedbackTest.java`

Checks that kP, kI and kD give the same voltage on every motor controller, in YAMS units. kP is volts per mechanism rotation of error (per meter for a linear mechanism, per rotation per second for velocity control), kI is volts per rotation-second of summed error, and kD is volts per rotation per second of the error's rate of change.

## Setup

* Every test runs on SPARK MAX, SPARK Flex, TalonFX, TalonFXS, Nova and A301, using the shared `ClosedLoopCases` helper.
* The mechanism has a 1000 kg m² moment of inertia, so it barely moves while a gain is measured and the error stays as set.
* The simulated battery is reset to 12 V. Unless noted, the voltage is averaged from 0.5 to 0.8 seconds after the first command, with a tolerance of 5% plus 50 mV.

## Tests

| Test                       | What it checks                                                                                                                                                 |
| -------------------------- | -------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `positionProportionalGain` | With kP = 5 and a 0.2 rotation position error, the voltage is kP x error (1 V).                                                                                |
| `velocityProportionalGain` | With kP = 1 and a 1 rotation per second velocity setpoint, the voltage is kP x error (1 V).                                                                    |
| `linearProportionalGain`   | With a 0.2 m circumference, an `ElevatorFeedforward` with no gains (which makes the loop linear), kP = 20 and a 0.05 m error, the voltage is kP x error (1 V). |
| `positionIntegralGain`     | With kI = 4 and a steady 0.25 rotation error, the voltage grows by kI x error (1 V) each second, measured between 0.5 and 1 second, within 10%.                |
| `positionDerivativeGain`   | With kD = 4, the simulated mechanism moving at 0.25 rotations per second and a setpoint of 0, the voltage is -kD x velocity (-1 V).                            |

## Run it

From the `vendordep` folder:

```bash
./gradlew test --tests "yams.core.motorcontrollers.FeedbackTest"
```
