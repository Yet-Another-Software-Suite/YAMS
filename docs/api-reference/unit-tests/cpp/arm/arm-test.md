# ArmTest

**Group:** [arm](./)\
**Source:** `vendordep/src/test/native/cpp/mechs/ArmTest.cpp`\
**Tags:** `[ArmTest]`

Checks that a single jointed arm moves in simulation on every supported motor controller, both when you drive the motor controller directly and when you use the `Arm` mechanism. It mirrors the Java `ArmTest`.

## Setup

* Every test runs once for each of SPARK MAX, SPARK Flex, TalonFXS, TalonFX, Nova and A301, each with no motion profile, a trapezoidal profile and an exponential profile. That is 17 cases (`DYNAMIC_SECTION`s), because the Nova has no case without a profile.
* The motor controller has a kP of 5, a 12:1 reduction (3:1 then 4:1), an arm feedforward with a kV of 1, limits of -100 to 100 degrees and a 40 A current limit. An A301 uses its own 64:1 gearbox and no current limit.
* The `Arm` tests add an `Arm` with a 4 in arm length and angles from -100 to 200 degrees.
* A test passes if the arm moved or the motor controller applied a nonzero duty cycle at any point. For the TalonFX and TalonFXS, a duty cycle test that sees no movement prints a warning instead of failing, because Phoenix simulates Talons in real time.

## Tests

| Test                     | What it checks                                                                                                   |
| ------------------------ | ---------------------------------------------------------------------------------------------------------------- |
| `ArmTest.SMCDutyCycle`   | A 0.5 duty cycle command on the motor controller's subsystem moves the arm within 1.5 s.                         |
| `ArmTest.SMCPositionPID` | A command that calls `SetPosition(80 deg)` on the motor controller moves the arm toward the setpoint within 1 s. |
| `ArmTest.ArmDutyCycle`   | With an `Arm` and its `Set(0.5)` command scheduled, a 0.5 duty cycle command moves the arm.                      |
| `ArmTest.ArmPositionPID` | With an `Arm` and its `RunTo(80 deg)` command scheduled, the arm moves toward 80 degrees.                        |

## Run it

From the `vendordep` folder:

```bash
./gradlew catchTests "-PcatchFilter=[ArmTest]"
```
