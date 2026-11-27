# ZeroOffsetWrappingTest

**Group:** [zero-offsets](./)\
**Source:** `vendordep/src/test/java/yams/core/motorcontrollers/ZeroOffsetWrappingTest.java`

Checks `withExternalEncoderZeroOffset` together with `withContinuousWrapping`. The zero offset moves the encoder's zero, and the wrapping range must move with it, so the range is measured from where the mechanism is at zero.

## Setup

* Cases come from `AbsoluteEncoderCases`: each SPARK's absolute encoder, an absolute encoder each SPARK reads over CAN, a CANcoder and a CANdi on each of TalonFX and TalonFXS, a Nova's absolute encoder, a Thrifty CAN Encoder on a Nova, and an A301's own encoder.
* Every case uses a 90° zero offset, a PID, and 12:1 gearing.
* With a wrapping maximum of 0.5 rotations the encoder should report -180° to 180°, and the setpoints are 120° and -120°. With a maximum of 1 rotation it should report 0° to 360°, and the setpoints are 300° and 30°.
* 120° and 300° are the key setpoints. If the range did not move with the zero, they would read -240° and -60°. The two Nova encoders only run the 0 to 1 rotation range, since a Nova closing its own loop reads only that range. That leaves 40 cases.

## Tests

| Test                                  | What it checks                                                                                                                                                                                                              |
| ------------------------------------- | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `wrappingRangeMovesWithTheZeroOffset` | After 2.5 seconds driving to the setpoint, the simulated mechanism is within 5° of it (either way round), and the encoder reports the setpoint itself, within 5°, not the same angle a rotation away. Runs on all 40 cases. |

## Run it

From the `vendordep` folder:

```bash
./gradlew test --tests "yams.core.motorcontrollers.ZeroOffsetWrappingTest"
```
