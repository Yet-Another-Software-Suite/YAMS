# LiveTuningTest

**Group:** [live-tuning](./)\
**Source:** `vendordep/src/test/java/yams/core/motorcontrollers/LiveTuningTest.java`

Checks live tuning on every motor controller wrapper and every tunable closed loop controller, for rotating and linear mechanisms. The tuning table must show the config's values, and values you enter there must be applied as given: the config holds them exactly, and the motor controller holds them in its own units.

## Setup

* Runs over SPARK MAX, SPARK Flex, TalonFX, TalonFXS, Nova and A301, each with PID, a trapezoidal profile and an exponential profile, each rotating and linear (36 cases).
* Each config has a 12:1 gearing, kP = 1.5, kI = 0.25, kD = 0.125 and a `SimpleMotorFeedforward(0.2, 0.75, 0.05)`. Linear cases add a 0.2 m circumference and a linear closed loop controller.
* Each device is reset to factory defaults before it is closed, since simulated devices keep their config for the rest of the run.

## Tests

| Test                         | What it checks                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                              |
| ---------------------------- | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `tuningUsesTheConfigsValues` | The tuning table shows the config's PID and feedforward gains, plus the profile's values: max velocity and acceleration in RPM and RPM per second (or m/s and m/s²) for a trapezoidal profile, or kV, kA and max input for an exponential profile. After new values are entered and `applyTuningValues()` runs, the config holds exactly the new gains and profile values. The device's kP is the entered kP in its own units: divided by 12 x the gear ratio on a SPARK, as given on a Talon (converted from per meter to per rotation when linear), and as given on the Nova and A301, whose loop runs on the SystemCore. |

## Run it

From the `vendordep` folder:

```bash
./gradlew test --tests "yams.core.motorcontrollers.LiveTuningTest"
```
