# CANdiTest

**Group:** [absolute-encoders](./)\
**Source:** `vendordep/src/test/java/yams/core/motorcontrollers/CANdiTest.java`

Checks a CANdi as the external feedback encoder of a `TalonFXWrapper` and a `TalonFXSWrapper`. The Talon and the CANdi must be configured from the `SmartMotorControllerConfig`, stay configured when the config is applied again, and close the loop on the CANdi in simulation. Continuous wrapping with a CANdi is covered by [ContinuousWrappingTest](../continuous-wrapping/continuous-wrapping-test.md).

## Setup

* A CANdi has two PWM inputs and YAMS cannot tell which one the encoder is on, so each case picks it with a vendor config whose feedback source is `SyncCANdiPWM1` or `SyncCANdiPWM2`.
* The parameterized cases cover TalonFX (Kraken X60) and TalonFXS (NEO), PWM1 and PWM2, with no zero offset and with a 40° zero offset: 8 cases in all.
* The CANdi is on the mechanism, 1:1, behind 12:1 gearing, with continuous wrapping over -0.5 to 0.5 rotations and a PID of kP 8.

## Tests

| Test                            | What it checks                                                                                                                                                                                                                                                                                                                                                                                                                                         |
| ------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| `configuresTheTalonAndTheCANdi` | The Talon's feedback source is `FusedCANdiPWM1` or `FusedCANdiPWM2` (matching the PWM input), with the CANdi's ID, a rotor to sensor ratio of 12, and a sensor to mechanism ratio of 1. The CANdi's PWM input has a discontinuity point of 0.5 and the expected zero offset. This holds after construction and after applying the config again. Applying wrapping over 0 to 1 rotation then changes the discontinuity point to 1. Runs on all 8 cases. |
| `closesTheLoopOnTheCANdi`       | In simulation, after 2 seconds of driving to 100°, both the mechanism position and the CANdi reading are within 5° of 100°. Runs on all 8 cases.                                                                                                                                                                                                                                                                                                       |
| `requiresAPwmInput`             | A CANdi used for feedback without a vendor config choosing a PWM input throws `SmartMotorControllerConfigurationException`. Runs on TalonFX and TalonFXS.                                                                                                                                                                                                                                                                                              |

## Run it

From the `vendordep` folder:

```bash
./gradlew test --tests "yams.core.motorcontrollers.CANdiTest"
```
