# FeedforwardTest

**Group:** [closed-loop-gains](./)\
**Source:** `vendordep/src/test/java/yams/core/motorcontrollers/FeedforwardTest.java`

Checks that kS, kV, kA and kG give the same voltage on every motor controller, in YAMS units. kS and kG are volts, kV is volts per mechanism rotation per second, kA is volts per rotation per second squared (per meter for a linear mechanism), and an arm's kG is scaled by the cosine of its angle. It also catches a gain sent twice, or left out, when a simulated SPARK follows a profile.

## Setup

* Every test runs on SPARK MAX, SPARK Flex, TalonFX, TalonFXS, Nova and A301, using the shared `ClosedLoopCases` helper. Velocity tests give the Nova a velocity profile, since it applies a feedforward to velocity control there.
* The mechanism has a 1000 kg m² moment of inertia and kP, kI and kD are 0, so the voltage is the feedforward alone.
* Voltages are averaged over part of the run, with a tolerance of 5% plus 50 mV.

## Tests

| Test                                     | What it checks                                                                                                   |
| ---------------------------------------- | ---------------------------------------------------------------------------------------------------------------- |
| `staticGainWithVelocityControl`          | Velocity control at 1 rotation per second with kS = 0.5 applies 0.5 V.                                           |
| `staticGainWhileFollowingAProfile`       | Following a slow trapezoidal profile toward 10 rotations with kS = 0.5 applies 0.5 V, so kS is applied once.     |
| `velocityGainWithVelocityControl`        | Velocity control at 1 rotation per second with kV = 1 applies 1 V.                                               |
| `velocityGainWhileFollowingAProfile`     | Following a profile at its 1 rotation per second cruise velocity with kV = 1 applies 1 V.                        |
| `accelerationGainWhileFollowingAProfile` | Following a profile while it speeds up at 1 rotation per second squared with kA = 1 applies 1 V.                 |
| `linearVelocityGain`                     | With a 0.2 m circumference and an `ElevatorFeedforward` kV of 5, velocity control at 0.1 m/s applies 0.5 V.      |
| `elevatorGravity`                        | Holding an elevator at 0 m with kG = 0.8 applies 0.8 V. Runs with and without a trapezoidal profile.             |
| `armGravity`                             | Holding an arm with kG = 1 applies kG x cos(angle). Runs at 60 and 150 degrees, each with and without a profile. |

## Run it

From the `vendordep` folder:

```bash
./gradlew test --tests "yams.core.motorcontrollers.FeedforwardTest"
```
