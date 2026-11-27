# CANcoderTest

**Group:** [absolute-encoders](./)\
**Source:** `vendordep/src/test/java/yams/core/motorcontrollers/CANcoderTest.java`

Checks that a `TalonFXWrapper` with a CANcoder as its external encoder configures the CANcoder from the `SmartMotorControllerConfig`: its discontinuity point from the wrapping range, and its zero offset. It also checks that a wrapping range the CANcoder cannot read is rejected.

## Setup

* Each test builds a simulated TalonFX (Kraken X60) and a CANcoder, on a 12:1 mechanism.
* Each test starts from fresh mock hardware.

## Tests

| Test                                       | What it checks                                                                                                                                                        |
| ------------------------------------------ | --------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `testWrappingCenteredOnZeroApplied`        | Continuous wrapping over -0.5 to 0.5 rotations builds without throwing and sets the CANcoder's `AbsoluteSensorDiscontinuityPoint` to 0.5.                             |
| `testWrappingAppliedWithoutContinuousWrap` | Plain wrapping (`withWrapping`) over -0.5 to 0.5 rotations sets the CANcoder's `AbsoluteSensorDiscontinuityPoint` to 0.5, and the Talon's `ContinuousWrap` stays off. |
| `testWrappingFromZeroApplied`              | Continuous wrapping over 0 to 1 rotation builds without throwing and sets the `AbsoluteSensorDiscontinuityPoint` to 1.                                                |
| `testWrappingTheCANcoderCannotReadThrows`  | Continuous wrapping over 0.5 to 1.5 rotations throws `SmartMotorControllerConfigurationException`, since a CANcoder reads at most 0 to 1 rotation.                    |
| `testZeroOffsetApplied`                    | A 45° external encoder zero offset builds without throwing. It does not read the offset back from the CANcoder.                                                       |
| `testCANcoderConfiguredWithoutWrapping`    | A CANcoder with no wrapping at all builds without throwing.                                                                                                           |

## Run it

From the `vendordep` folder:

```bash
./gradlew test --tests "yams.core.motorcontrollers.CANcoderTest"
```
