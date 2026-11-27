# ZeroOffsetTest

**Group:** [zero-offsets](./)\
**Source:** `vendordep/src/test/java/yams/core/motorcontrollers/ZeroOffsetTest.java`

Checks `withExternalEncoderZeroOffset`: the zero offset is the angle the absolute encoder reads, before the offset, where the mechanism is at zero. It makes sure the offset reaches each encoder, is applied once and the right way round, and is rejected or ignored when there is no absolute encoder to give it to.

## Setup

* Cases come from `AbsoluteEncoderCases`. The absolute encoders are each SPARK's absolute encoder, an absolute encoder each SPARK reads over CAN, a CANcoder and a CANdi on each of TalonFX and TalonFXS, a Nova's absolute encoder, a Thrifty CAN Encoder on a Nova, and an A301's own encoder (11 in all).
* The setups without an absolute encoder are the motor's encoder on SPARK MAX, SPARK Flex, TalonFX, TalonFXS, Nova and A301, a quadrature Through Bore Encoder on each SPARK, and a quadrature encoder on a Nova (9 in all).
* Each runs under a PID, a trapezoidal profile, an exponential profile and an LQR, behind 12:1 gearing. The zero offset is 40° and the setpoint is 100°.

## Tests

| Test                                              | What it checks                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  |
| ------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `encoderReadsZeroWhereTheMechanismIsAtZero`       | The encoder holds the 40° offset in its own terms: a CANcoder's `MagnetOffset`, a CANdi's PWM1 `AbsoluteSensorOffset`, a SPARK CAN encoder's duty cycle offset, a Thrifty zero offset for a Nova or CAN Encoder, or a SPARK absolute encoder's zero offset. (The A301 applies the offset to its readings instead.) After 2.5 seconds driving to 100°, the simulated mechanism and the mechanism reading are within 5° of 100° (within a rotation), and the encoder reads within 5° of 100°. Runs on each of the 11 absolute encoders under each of the 4 controllers: 44 cases. |
| `negativeZeroOffsetIsTheSameAngleWithinARotation` | A zero offset of -10° is stored as 350°.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        |
| `withoutAnAbsoluteEncoder`                        | With a 40° zero offset but no absolute encoder, a SPARK, Nova or A301 throws `SmartMotorControllerConfigurationException`. A Talon ignores the offset: the mechanism reads within 5° of 0° at the start, and after driving to 100° both the simulated mechanism and the reading are within 5° of 100°. Runs on each of the 9 setups under each of the 4 controllers: 36 cases.                                                                                                                                                                                                  |

## Run it

From the `vendordep` folder:

```bash
./gradlew test --tests "yams.core.motorcontrollers.ZeroOffsetTest"
```
