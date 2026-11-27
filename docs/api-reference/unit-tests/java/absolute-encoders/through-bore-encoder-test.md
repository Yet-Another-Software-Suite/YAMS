# ThroughBoreEncoderTest

**Group:** [absolute-encoders](./)\
**Source:** `vendordep/src/test/java/yams/core/motorcontrollers/ThroughBoreEncoderTest.java`

Checks a REV Through Bore Encoder on a `SparkWrapper` around a SPARK MAX and a SPARK Flex, in each way it can be connected. The SPARK and the encoder must be configured from the `SmartMotorControllerConfig`, and in simulation the encoder must read the mechanism. Continuous wrapping with each connection is covered by [ContinuousWrappingTest](../continuous-wrapping/continuous-wrapping-test.md).

## Setup

* The three connections are: absolute (duty cycle output on the SPARK's absolute encoder input), quadrature (on the SPARK MAX's alternate encoder or the SPARK Flex's external encoder input, using REVLib's Through Bore preset), and CAN (a REVLib `DetachedEncoder`; the test uses a MAXSpline Encoder, REVLib's only detached encoder model).
* The absolute and CAN connections use continuous wrapping over -0.5 to 0.5 rotations.
* The parameterized cases cover SPARK MAX and SPARK Flex, each connection, with the SPARK closing its loop on the encoder ("feedback") or only reading it ("reporting"): 12 cases in all.
* The encoder is on the mechanism 1:1, behind 12:1 gearing, with a PID of kP 8.

## Tests

| Test                               | What it checks                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        |
| ---------------------------------- | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `configuresTheSparkAndTheEncoder`  | The SPARK's closed loop sensor is the primary encoder when reporting, and the absolute, alternate or external, or detached absolute encoder when used for feedback. Then, per connection: absolute sets the range offset to 0, and to 0.5 after applying wrapping over 0 to 1 rotation; quadrature has 8192 counts per revolution; CAN is zero centered, and not after applying wrapping over 0 to 1 rotation, and when used for feedback the SPARK holds the encoder's CAN ID. Runs on all 12 cases. |
| `readsTheMechanism`                | In simulation, after 2 seconds of driving to 100°, the simulated mechanism, the mechanism reading, and the Through Bore Encoder reading are all within 5° of 100°. Runs on all 12 cases.                                                                                                                                                                                                                                                                                                              |
| `quadratureEncoderHasNoZeroOffset` | A quadrature Through Bore Encoder used for feedback with a 30° zero offset throws `SmartMotorControllerConfigurationException`. Runs on SPARK MAX and SPARK Flex.                                                                                                                                                                                                                                                                                                                                     |

## Run it

From the `vendordep` folder:

```bash
./gradlew test --tests "yams.core.motorcontrollers.ThroughBoreEncoderTest"
```
