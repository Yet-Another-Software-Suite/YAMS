# SparkAbsoluteEncoderTest

**Group:** [absolute-encoders](./)\
**Source:** `vendordep/src/test/java/yams/core/motorcontrollers/SparkAbsoluteEncoderTest.java`

Checks that a `SparkWrapper` with the SPARK MAX's absolute encoder as its external encoder sets the encoder's range offset from the wrapping range. REVLib's range offset is the middle of the range the encoder reports, so it decides whether the encoder reads -0.5 to 0.5 or 0 to 1 rotation.

## Setup

* Each test builds a simulated SPARK MAX (NEO) with 1:1 gearing and a PID of kP 4, using `sparkMax.getAbsoluteEncoder()` as the external encoder.
* The SPARK applies its config in the background, so the tests poll for up to half a second before checking a value.

## Tests

| Test                                                    | What it checks                                                                                                                                                                               |
| ------------------------------------------------------- | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `testRangeOffset0WithWrappingCenteredOnZero`            | Continuous wrapping over -0.5 to 0.5 rotations builds without throwing and sets the range offset to 0.                                                                                       |
| `testRangeOffset0_5WithWrappingFromZero`                | Continuous wrapping over 0 to 1 rotation builds without throwing and sets the range offset to 0.5.                                                                                           |
| `testWrappingSetsTheRangeOffsetWithoutPositionWrapping` | Plain wrapping (`withWrapping`) over -0.5 to 0.5 rotations sets the range offset to 0 and the SPARK's position wrapping input range to -0.5 to 0.5, but leaves position wrapping turned off. |
| `testExceptionWhenWrappingARangeTheEncoderCannotRead`   | Continuous wrapping over 0.5 to 1.5 rotations throws `SmartMotorControllerConfigurationException`, since the encoder can only read from -1 to 0 up to 0 to 1 rotation.                       |

## Run it

From the `vendordep` folder:

```bash
./gradlew test --tests "yams.core.motorcontrollers.SparkAbsoluteEncoderTest"
```
