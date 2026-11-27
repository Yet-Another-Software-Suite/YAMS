# motor-controllers

The parity and regression tests of each vendor's wrapper. Parity tests check the C++ wrapper does what the Java one does.

## Test files

| Test file                                                                       | Tests | What it checks                                                                                                                                                                                                   |
| ------------------------------------------------------------------------------- | ----- | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| [SmartMotorControllerParityTest](smart-motor-controller-parity-test.md)         | 20    | Checks that the C++ SmartMotorController, its config and the sim suppliers match Java for sim stepping, battery sim, wrapping, velocity profiles, feedforward force, conversions, config builders and telemetry. |
| [SmartMotorControllerRegressionTest](smart-motor-controller-regression-test.md) | 21    | Regression tests for SmartMotorController behavior that used to differ from Java, covering telemetry, Live Tuning, linear mode, feedforwards, wrapping, limits, profiles and config validation.                  |
| [SparkWrapperParityTest](spark-wrapper-parity-test.md)                          | 26    | Checks that the C++ SparkWrapper sends the same gains, setpoints, profile limits, soft limits, tolerances, wrapping and encoder values to a SPARK MAX as the Java wrapper, converted to motor rotations and RPM. |
| [NovaWrapperParityTest](nova-wrapper-parity-test.md)                            | 11    | Checks that the C++ NovaWrapper accepts and rejects the same options as Java, reads encoders the same way, and runs the closed loop, gains, soft limits and wrapping on the Nova in the same cases.              |
| [A301WrapperParityTest](a301-wrapper-parity-test.md)                            | 4     | Checks that the C++ A301Wrapper rejects the same options as Java, converges in simulation, reads its absolute encoder in the wrapping range and uses the A301 motor model.                                       |
| [TalonWrapperApiTest](talon-wrapper-api-test.md)                                | 5     | Checks the Talon wrapper API ported from Java (FOC, CANdi PWM selection, update frequency, forced config apply, TalonFXS motor arrangement) and the linear exponential profile setter.                           |

## Run the group

From the `vendordep` folder:

```bash
./gradlew catchTests "-PcatchFilter=[SMCParity],[SMCRegression],[SparkParity],[NovaParity],[A301Parity],[TalonApi]"
```
