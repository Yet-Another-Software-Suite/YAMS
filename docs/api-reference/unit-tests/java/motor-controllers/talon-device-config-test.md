# TalonDeviceConfigTest

**Group:** [motor-controllers](./)\
**Source:** `vendordep/src/test/java/yams/core/motorcontrollers/TalonDeviceConfigTest.java`

Checks that the Talon wrappers' live setters write the same Talon configuration that `applyConfig` writes. It also checks when a temperature cutoff is allowed, and the velocity measurement delay a Talon reports.

## Setup

* Every test runs on TalonFX (Kraken X60) and TalonFXS (NEO).
* Each Talon gets a 12:1 gearing, a 40 A stator limit, brake mode and kP = 1, unless a test sets an LQR.
* The tests read configs back from the Talon to check them, and reset the Talon to a default config before closing it.

## Tests

| Test                                                      | What it checks                                                                                                                                                                                                                                   |
| --------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| `rampRateSettersSetEveryRamp`                             | `setOpenLoopRampRate(0.3 s)` sets the duty cycle, voltage and torque open loop ramps to 0.3 s. `setClosedLoopRampRate(0.4 s)` sets all three closed loop ramps to 0.4 s.                                                                         |
| `maxAccelerationWithVelocityProfileSetsJerk`              | With a velocity trapezoidal profile of 10 rotations per second squared and 100 per second cubed, Motion Magic gets acceleration 10 and jerk 100. `setMotionProfileMaxAcceleration(7)` then sets the jerk to 7 and leaves the acceleration at 10. |
| `temperatureCutoffNeedsAnLqrController`                   | A temperature cutoff with PID (which runs on the Talon) throws `IllegalArgumentException`. With an LQR controller (which runs on the SystemCore) it is accepted and the config reports 80 °C.                                                    |
| `velocityMeasurementDelayDependsOnWhereTheClosedLoopRuns` | With PID on the Talon, `getVelocityMeasurementDelay()` is 1 ms. With an LQR on the SystemCore, it is 1 ms plus half the velocity signal's update period.                                                                                         |

## Run it

From the `vendordep` folder:

```bash
./gradlew test --tests "yams.core.motorcontrollers.TalonDeviceConfigTest"
```
