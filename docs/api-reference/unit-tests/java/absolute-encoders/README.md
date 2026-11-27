# absolute-encoders

External encoders on each motor controller: CANcoders and CANdis on Talons, the SPARK absolute encoder and Through Bore Encoders on SPARKs, and starting away from zero.

## Test classes

| Test class                                                 | Tests | What it checks                                                                                                                                                                                                                       |
| ---------------------------------------------------------- | ----- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| [CANcoderTest](cancoder-test.md)                           | 6     | Checks that a TalonFX with a CANcoder sets the CANcoder's discontinuity point from the wrapping range and rejects a range the CANcoder cannot read.                                                                                  |
| [CANdiTest](candi-test.md)                                 | 3     | Checks a CANdi on each PWM input as the feedback encoder of a TalonFX and a TalonFXS: the devices are configured, they stay configured when the config is applied again, the loop closes in simulation, and a PWM input is required. |
| [SparkAbsoluteEncoderTest](spark-absolute-encoder-test.md) | 4     | Checks that a SPARK MAX absolute encoder gets the right range offset from the wrapping range and rejects a range it cannot read.                                                                                                     |
| [ThroughBoreEncoderTest](through-bore-encoder-test.md)     | 3     | Checks a Through Bore Encoder on a SPARK MAX and a SPARK Flex, wired absolute, wired quadrature, and over CAN: how it is configured, that it reads the mechanism in simulation, and that a quadrature encoder rejects a zero offset. |
| [StartingPositionTest](starting-position-test.md)          | 1     | Checks that a mechanism starting at 30 degrees, with or without an absolute encoder, reads 30 degrees at the start and moves only 70 degrees to reach 100 degrees, across 80 cases.                                                  |

## Run the group

From the `vendordep` folder:

```bash
./gradlew test --tests "yams.core.motorcontrollers.CANcoderTest" \
  --tests "yams.core.motorcontrollers.CANdiTest" \
  --tests "yams.core.motorcontrollers.SparkAbsoluteEncoderTest" \
  --tests "yams.core.motorcontrollers.ThroughBoreEncoderTest" \
  --tests "yams.core.motorcontrollers.StartingPositionTest"
```
