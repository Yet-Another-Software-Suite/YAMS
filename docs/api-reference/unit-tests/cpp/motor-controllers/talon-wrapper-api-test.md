# TalonWrapperApiTest

**Group:** [motor-controllers](./)\
**Source:** `vendordep/src/test/native/cpp/TalonWrapperApiTest.cpp`\
**Tags:** `[TalonApi]`

Checks the `TalonFXWrapper` and `TalonFXSWrapper` API ported from Java: FOC, CANdi PWM selection, status signal update frequency, forced config apply, and the TalonFXS motor arrangement chosen from the `DCMotor`. It also checks the config's linear exponential profile setter.

## Setup

* Uses the shared simulated TalonFX, TalonFXS and CANdi from `TalonTestHardware`, reset before each test.
* The config has a 10:1 gearing, a 40 A stator current limit, kP of 1 and closed loop mode.

## Tests

| Test                                          | What it checks                                                                                                                                                                                                           |
| --------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| `TalonApi.FOC`                                | After `DisableFOC()` the TalonFX's control request has FOC off, and after `EnableFOC()` it has FOC on. A TalonFXS using a `TorqueCurrentFOC` request throws on `EnableFOC()`.                                            |
| `TalonApi.CANdiPWMSelection`                  | A vendor config with `SyncCANdiPWM1` and a CANdi external encoder selects PWM1, not PWM2, and the TalonFX is set to `FusedCANdiPWM1`. The external encoder then reads the simulated mechanism position (0.25 rotations). |
| `TalonApi.UpdateFrequencyAndForceConfigApply` | `SetUpdateFrequency(250_Hz)` sets the velocity signal to 250 Hz, and `ForceConfigApply()` succeeds.                                                                                                                      |
| `TalonApi.TalonFXSArrangementFromMotor`       | The TalonFXS motor arrangement comes from the `DCMotor`: Minion gives `Minion_JST` with advanced hall support, NEO Vortex gives `VORTEX_JST`, and a Kraken X60 throws.                                                   |
| `TalonApi.LinearExponentialProfileSetter`     | `WithLinearExponentialProfile` throws without a circumference. With a 0.2 m circumference it turns on the linear closed loop and converts kV and kA to per mechanism rotation (0.2 and 0.02).                            |

## Run it

From the `vendordep` folder:

```bash
./gradlew catchTests "-PcatchFilter=[TalonApi]"
```
