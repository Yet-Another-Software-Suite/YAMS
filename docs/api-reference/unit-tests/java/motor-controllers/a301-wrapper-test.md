# A301WrapperTest

**Group:** [motor-controllers](./)\
**Source:** `vendordep/src/test/java/yams/core/motorcontrollers/A301WrapperTest.java`

Checks the `A301Wrapper`. It rejects options the A301 does not have, runs its closed loop on the SystemCore, reads its absolute encoder in the wrapping range, and uses the A301's own motor model.

## Setup

* Each A301 gets its 64:1 gearbox as the gearing, closed loop control and kP = 10.
* The simulated battery is reset to 12 V before each test, and every A301 a test creates is closed after it.

## Tests

| Test                                     | What it checks                                                                                                                                                                                                                                                                                                                                   |
| ---------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| `rejectsWhatTheA301DoesNotHave`          | The wrapper throws `SmartMotorControllerConfigurationException` for a stator limit, supply limit, open loop ramp, closed loop ramp, voltage compensation, vendor config, followers, or an external encoder zero offset. It throws `IllegalArgumentException` for another A301 as the external encoder: only its own absolute encoder is allowed. |
| `closedLoopRunsOnTheSystemCore`          | Aiming for 0.25 rotations for 3 seconds in simulation, the mechanism ends within 0.02 rotations of 0.25.                                                                                                                                                                                                                                         |
| `absoluteEncoderReadsInTheWrappingRange` | Using its own absolute encoder and starting at -0.2 rotations, the A301 reads 0.8 rotations in the default 0 to 1 range. With wrapping from -0.5 to 0.5 it reads -0.2, and an external encoder position is present.                                                                                                                              |
| `motorIsTheA301s`                        | `YMotors.getA301(1)` has a free speed of 13,500 RPM and 5.27 Nm at peak power through 135:1. The wrapper's `getDCMotor()` has the same free speed.                                                                                                                                                                                               |

## Run it

From the `vendordep` folder:

```bash
./gradlew test --tests "yams.core.motorcontrollers.A301WrapperTest"
```
