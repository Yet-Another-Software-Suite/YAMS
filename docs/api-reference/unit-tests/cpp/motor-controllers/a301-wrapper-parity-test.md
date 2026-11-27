# A301WrapperParityTest

**Group:** [motor-controllers](./)\
**Source:** `vendordep/src/test/native/cpp/A301WrapperParityTest.cpp`\
**Tags:** `[A301Parity]`

Checks that the C++ `A301Wrapper` behaves like the Java `A301Wrapper`: it rejects the same config options, runs its closed loop on the SystemCore, reads its absolute encoder in the same range, and uses the same motor model.

## Setup

* Each test builds a new `A301` on CAN bus `CAN_S2`, with a 64:1 gearing, kP of 10 and closed loop mode.
* REVLib does not simulate the A301, so the closed loop test steps the wrapper's own `DCMotorSim` with `SimIterate()`.

## Tests

| Test                                                | What it checks                                                                                                                                                                                            |
| --------------------------------------------------- | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `A301Parity.RejectsWhatTheA301DoesNotHave`          | Building the wrapper throws for options the A301 does not have: a current limit, a ramp rate, voltage compensation, a vendor config, and an external encoder that is not the A301's own absolute encoder. |
| `A301Parity.ClosedLoopConvergesInSimulation`        | A position setpoint of 0.25 rotations is reached within 0.02 rotations after 3 s of closed loop and simulation steps.                                                                                     |
| `A301Parity.AbsoluteEncoderReadsInTheWrappingRange` | With the A301's own absolute encoder and a starting position of -0.2 rotations, the position reads 0.8 in \[0, 1) without wrapping, and -0.2 with wrapping over \[-0.5, 0.5).                             |
| `A301Parity.MotorIsTheA301s`                        | `YMotors::A301()` has a 13,500 RPM free speed and 5.27 Nm at peak power through 135:1, and the wrapper's `GetDCMotor()` is that motor.                                                                    |

## Run it

From the `vendordep` folder:

```bash
./gradlew catchTests "-PcatchFilter=[A301Parity]"
```
