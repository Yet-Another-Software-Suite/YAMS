# ElevatorTest

**Group:** [elevator](./)\
**Source:** `vendordep/src/test/native/cpp/mechs/ElevatorTest.cpp`\
**Tags:** `[ElevatorTest]`

Checks that an elevator moves in simulation on every supported motor controller, both when you drive the motor controller directly and when you use the `Elevator` mechanism. It mirrors the Java `ElevatorTest`.

## Setup

* Every test runs once for each of SPARK MAX, SPARK Flex, TalonFXS, TalonFX, Nova and A301, each with no motion profile, a trapezoidal profile and an exponential profile. That is 17 cases (`DYNAMIC_SECTION`s), because the Nova has no case without a profile.
* The motor controller has a kP of 8, a 12:1 reduction (3:1 then 4:1), a 5.5 in circumference (a 22 tooth sprocket with 0.25 in pitch), an elevator feedforward with a kS of 1 V, measurement limits of 0 to 5 m and a 40 A current limit. An A301 uses its own 64:1 gearbox and no current limit.
* The `Elevator` tests add an `Elevator` with a 1 lb carriage and heights from 0 to 3 m.
* A test passes if the elevator moved or the motor controller applied a nonzero duty cycle at any point. For the TalonFX and TalonFXS, a duty cycle test that sees no movement prints a warning instead of failing, because Phoenix simulates Talons in real time.

## Tests

| Test                               | What it checks                                                                                                                                                                    |
| ---------------------------------- | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `ElevatorTest.SMCDutyCycle`        | A full (1.0) duty cycle command on the motor controller's subsystem moves the elevator up within 1 s.                                                                             |
| `ElevatorTest.SMCPositionPID`      | A command that calls `SetPosition(2 m)` on the motor controller moves the elevator toward the setpoint. It runs for 20 s, or 1 s on a Talon.                                      |
| `ElevatorTest.ElevatorDutyCycle`   | With an `Elevator` and its `Set(1.0)` command scheduled, a full duty cycle command moves the elevator up.                                                                         |
| `ElevatorTest.ElevatorPositionPID` | `SetMeasurementPositionSetpoint(0.5 m)` makes `GetMeasurementSetpoint()` return 0.5 m. Then, with the `Elevator`'s `RunTo(2 m)` command scheduled, the elevator moves toward 2 m. |

## Run it

From the `vendordep` folder:

```bash
./gradlew catchTests "-PcatchFilter=[ElevatorTest]"
```
