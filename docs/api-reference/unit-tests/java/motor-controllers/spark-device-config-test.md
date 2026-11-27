# SparkDeviceConfigTest

**Group:** [motor-controllers](./)\
**Source:** `vendordep/src/test/java/yams/core/motorcontrollers/SparkDeviceConfigTest.java`

Checks that values `SparkWrapper` computes reach the SPARK's own configuration, in the SPARK's units. Soft limits must be in rotations of the feedback sensor, and `setEncoderPosition` must send the absolute encoder's zero offset.

## Setup

* Each test builds a simulated SPARK MAX with a NEO, a 40 A stator limit, brake mode and kP = 1. Most tests use a 12:1 gearing.
* SPARK parameters apply in the background, so the tests wait until the SPARK reports the expected value.
* Each SPARK is reset to a default config before it is closed.

## Tests

| Test                                                  | What it checks                                                                                                                                                                                                                         |
| ----------------------------------------------------- | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `softLimitsAreInFeedbackSensorRotations`              | Soft limits of -0.5 and 0.75 mechanism rotations reach the SPARK as 12 times those values. `setMechanismLimits`, `setMechanismUpperLimit` and `setMechanismLowerLimit` update the SPARK the same way.                                  |
| `linearSoftLimitsAreInFeedbackSensorRotations`        | With a 0.2 m mechanism circumference, soft limits of -0.1 m and 0.2 m reach the SPARK as meters / circumference x 12. `setMeasurementUpperLimit` and `setMeasurementLowerLimit` update the SPARK the same way.                         |
| `setEncoderPositionSendsTheAbsoluteEncoderZeroOffset` | With the SPARK's absolute encoder as the external encoder (1:1 gearing), the zero offset starts at 0. After `setEncoderPosition(-0.3 rotations)` it is the old mechanism position minus -0.3.                                          |
| `temperatureCutoffNeedsTheSystemCoreClosedLoop`       | A temperature cutoff with a trapezoidal profile (which runs on the SPARK) throws `SmartMotorControllerConfigurationException`. With an exponential profile (which runs on the SystemCore) it is accepted and the config reports 80 °C. |

## Run it

From the `vendordep` folder:

```bash
./gradlew test --tests "yams.core.motorcontrollers.SparkDeviceConfigTest"
```
