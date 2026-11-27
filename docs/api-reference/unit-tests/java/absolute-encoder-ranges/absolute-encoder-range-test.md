# AbsoluteEncoderRangeTest

**Group:** [absolute-encoder-ranges](./)\
**Source:** `vendordep/src/test/java/yams/core/motorcontrollers/AbsoluteEncoderRangeTest.java`

Checks that `withContinuousWrapping` sets the range an absolute encoder reports. Wrapping over -0.5 to 0.5 rotations makes the encoder report angles in that range, and wrapping over 0 to 1 rotation makes it report 0 to 1. This holds for every absolute encoder YAMS supports, under every closed loop controller.

## Setup

* Cases come from `AbsoluteEncoderCases.java`: each SPARK's absolute encoder (SPARK MAX, SPARK Flex), an absolute encoder each SPARK reads over CAN, a CANcoder and a CANdi (PWM1) on each of TalonFX and TalonFXS, a Nova's absolute encoder, a Thrifty CAN Encoder on a Nova, and an A301's own encoder.
* Each encoder is on the mechanism 1:1, behind 12:1 gearing, and closes the loop.
* Each runs under a PID, a trapezoidal profile, an exponential profile and an LQR, for both wrapping ranges. The two Nova encoders skip the -0.5 to 0.5 range under PID and trapezoidal profile, since a Nova running its own loop only reads 0 to 1 rotation. That leaves 84 cases.

## Tests

| Test                                     | What it checks                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                |
| ---------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `encoderReportsAnglesInTheWrappingRange` | First, the device holds the range: a CANcoder's or CANdi's `AbsoluteSensorDiscontinuityPoint` equals the wrapping maximum, a SPARK CAN encoder is zero centered only for -0.5 to 0.5, and a SPARK absolute encoder's range offset is the wrapping maximum minus 0.5. (Nova and A301 encoders have no range of their own, so only the next part applies.) Then the mechanism drives for 2.5 seconds to an angle only that range holds: -100° for -0.5 to 0.5, and 260° for 0 to 1. The simulated mechanism must be within 5° of it (either way round), and the encoder must report an angle inside the range, within 5° of the setpoint. Runs on all 84 cases. |

## Run it

From the `vendordep` folder:

```bash
./gradlew test --tests "yams.core.motorcontrollers.AbsoluteEncoderRangeTest"
```
