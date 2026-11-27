# LQRControllerTest

**Group:** [other](./)\
**Source:** `vendordep/src/test/native/cpp/LQRControllerTest.cpp`\
**Tags:** `[LQRConfig]`, `[LQRController]`

Checks `LQRConfig` and `LQRController`, the linear quadratic regulator YAMS can use as a closed loop controller. The tests check the config's accessors and that the controller's output points the right way and stays within the voltage limit. No hardware or simulation is used.

## Setup

* Three configs, each on one NEO: a flywheel (MOI 0.00233 kg·m², 1:1), an arm (MOI 0.5 kg·m², 50:1) and an elevator (5 kg, 0.0254 m drum radius, 10:1). Each sets its Q and R elements and its state and measurement standard deviations.

## Tests

| Test                                                        | What it checks                                                                                                         |
| ----------------------------------------------------------- | ---------------------------------------------------------------------------------------------------------------------- |
| `LQRConfig.FlywheelSystemSetsType`                          | `WithFlywheelSystem` makes the type `FLYWHEEL`.                                                                        |
| `LQRConfig.ArmSystemSetsType`                               | `WithArmSystem` makes the type `ARM`.                                                                                  |
| `LQRConfig.ElevatorSystemSetsType`                          | `WithElevatorSystem` makes the type `ELEVATOR`.                                                                        |
| `LQRConfig.DefaultPeriodIs20ms`                             | The default period is 20 ms.                                                                                           |
| `LQRConfig.WithPeriodStored`                                | `WithPeriod(10 ms)` sets the period to 10 ms.                                                                          |
| `LQRConfig.DefaultMaxVoltageIs12V`                          | The default maximum voltage is 12 V.                                                                                   |
| `LQRConfig.WithMaxVoltageStored`                            | `WithMaxVoltage(9 V)` sets the maximum voltage to 9 V.                                                                 |
| `LQRConfig.FlywheelGetLoopIsLoop1`                          | A flywheel config's `GetLoop()` holds a one state loop.                                                                |
| `LQRConfig.ArmGetLoopIsLoop2`                               | An arm config's `GetLoop()` holds a two state loop.                                                                    |
| `LQRConfig.ElevatorGetLoopIsLoop2`                          | An elevator config's `GetLoop()` holds a two state loop.                                                               |
| `LQRController.FlywheelGetType`                             | A controller built from a flywheel config reports `FLYWHEEL`.                                                          |
| `LQRController.ArmGetType`                                  | A controller built from an arm config reports `ARM`.                                                                   |
| `LQRController.ElevatorGetType`                             | A controller built from an elevator config reports `ELEVATOR`.                                                         |
| `LQRController.GetConfigIsPresent`                          | `GetConfig()` returns a config.                                                                                        |
| `LQRController.GetConfigPreservesType`                      | The config `GetConfig()` returns keeps the `ARM` type.                                                                 |
| `LQRController.Flywheel_BelowSetpoint_PositiveOutput`       | At 0 rad/s with a 200 rad/s setpoint, the output is positive.                                                          |
| `LQRController.Flywheel_AboveSetpoint_NegativeOutput`       | After a few loops at 300 rad/s, a 100 rad/s setpoint gives a negative output.                                          |
| `LQRController.Flywheel_ClampedToMaxVoltage`                | A huge setpoint gives at most 12 V.                                                                                    |
| `LQRController.Flywheel_OutputIsFinite`                     | The output for a 100 rad/s setpoint is a finite number.                                                                |
| `LQRController.Flywheel_ResetToSetpoint_ReducesOutput`      | A controller reset to the 100 rad/s setpoint gives a smaller output than one that starts from 0.                       |
| `LQRController.Arm_BelowSetpoint_PositiveOutput`            | At 0 rad with a 1 rad setpoint, the output is positive.                                                                |
| `LQRController.Arm_AboveSetpoint_NegativeOutput`            | At 2 rad with a 1 rad setpoint, the output is negative.                                                                |
| `LQRController.Arm_ClampedToMaxVoltage`                     | A 1000 rad setpoint gives at most 12 V.                                                                                |
| `LQRController.Arm_OutputIsFinite`                          | The output for a 0.5 rad setpoint is a finite number.                                                                  |
| `LQRController.Arm_ResetToSetpoint_ReducesOutput`           | An arm reset to its 1 rad setpoint gives a smaller output than one reset to 0 rad.                                     |
| `LQRController.Elevator_BelowSetpoint_PositiveOutput`       | At 0 m with a 0.5 m setpoint, the output is positive.                                                                  |
| `LQRController.Elevator_AboveSetpoint_NegativeOutput`       | At 1 m with a 0.5 m setpoint, the output is negative.                                                                  |
| `LQRController.Elevator_ClampedToMaxVoltage`                | A 10000 m setpoint gives at most 12 V.                                                                                 |
| `LQRController.Elevator_OutputIsFinite`                     | The output for a 0.5 m setpoint is a finite number.                                                                    |
| `LQRController.Elevator_ResetAffectsDirection`              | On one controller, resetting to 0 m and then to 1 m with a 0.5 m setpoint gives a positive and then a negative output. |
| `LQRController.LinearFlywheel_BelowSetpoint_PositiveOutput` | The flywheel controller given linear speeds, at 0 m/s with a 5 m/s setpoint, gives a positive output.                  |
| `LQRController.LinearFlywheel_ClampedToMaxVoltage`          | A huge linear speed setpoint gives at most 12 V.                                                                       |
| `LQRController.LinearFlywheel_OutputIsFinite`               | The output for a 5 m/s setpoint is a finite number.                                                                    |
| `LQRController.UpdateConfig_ChangesType`                    | `UpdateConfig` with an arm config changes a flywheel controller's type to `ARM`.                                       |
| `LQRController.UpdateConfig_NewMaxVoltageRespected`         | After `UpdateConfig` with a 6 V maximum, a huge setpoint gives at most 6 V.                                            |

## Run it

From the `vendordep` folder:

```bash
./gradlew catchTests "-PcatchFilter=[LQRConfig],[LQRController]"
```
