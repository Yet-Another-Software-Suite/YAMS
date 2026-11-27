# ShooterTest

**Group:** [shooter](./)\
**Source:** `vendordep/src/test/native/cpp/mechs/ShooterTest.cpp`\
**Tags:** `[ShooterTest]`

Checks that a shooter (a `FlyWheel`) spins up in simulation on every supported motor controller, both when you drive the motor controller directly and when you use the mechanism. It also checks that an idle shooter does not overwrite the shared simulated battery voltage. It mirrors the Java `ShooterTest`.

## Setup

* Most tests run once for each of SPARK MAX, SPARK Flex, TalonFXS, TalonFX, Nova and A301, each with no motion profile, a trapezoidal profile and an exponential profile. That is 17 cases (`DYNAMIC_SECTION`s), because the Nova has no case without a profile.
* The motor controller has a kP of 100, a 12:1 reduction (3:1 then 4:1), coast mode, a simple feedforward with a kV of 1, and a 40 A current limit. An A301 uses the 3:1 launcher gearbox and no current limit.
* The `FlyWheel` tests add a `FlyWheel` with a 4 in roller.
* The movement tests pass if the shooter's speed or position changed, or the motor controller applied a nonzero duty cycle at any point. For the TalonFX and TalonFXS, a duty cycle test that sees no movement prints a warning instead of failing, because Phoenix simulates Talons in real time.

## Tests

| Test                                              | What it checks                                                                                                                                                                                                                                                     |
| ------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| `ShooterTest.SMCDutyCycle`                        | A 0.5 duty cycle command on the motor controller's subsystem spins the shooter within 1 s.                                                                                                                                                                         |
| `ShooterTest.SMCVelocityPID`                      | A command that calls `SetVelocity(12000 deg/s)` (about 2000 RPM) on the motor controller changes the shooter's speed within 2 s, or 1 s on a Talon.                                                                                                                |
| `ShooterTest.ShooterDutyCycle`                    | With a `FlyWheel` and its `Set(0.5)` command scheduled, a 0.5 duty cycle command spins the shooter.                                                                                                                                                                |
| `ShooterTest.SimIterateKeepsSharedBatteryVoltage` | On a SPARK MAX without a profile: when something else draws 100 A from the simulated battery, the voltage drops by more than 1 V, and the idle shooter's next `SimIterate()` leaves the RoboRIO voltage within 0.5 V of that shared value instead of resetting it. |
| `ShooterTest.ShooterVelocityPID`                  | With a `FlyWheel` and its `RunTo(480 deg/s)` command (about 80 RPM) scheduled, the shooter's speed changes.                                                                                                                                                        |

## Run it

From the `vendordep` folder:

```bash
./gradlew catchTests "-PcatchFilter=[ShooterTest]"
```
