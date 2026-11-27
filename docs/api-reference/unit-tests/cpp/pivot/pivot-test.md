# PivotTest

**Group:** [pivot](./)\
**Source:** `vendordep/src/test/native/cpp/mechs/PivotTest.cpp`\
**Tags:** `[PivotTest]`

Checks that a pivot moves in simulation on every supported motor controller, stays on its hard stops, wraps when it is continuous, and drives loosely coupled followers correctly. The first four tests mirror the Java `PivotTest`.

## Setup

* Most tests run once for each of SPARK MAX, SPARK Flex, TalonFXS, TalonFX, Nova and A301, each with no motion profile, a trapezoidal profile and an exponential profile. That is 17 cases (`DYNAMIC_SECTION`s), because the Nova has no case without a profile.
* The motor controller has a kP of 8, a 12:1 reduction (3:1 then 4:1), a simple feedforward with a kS of 1 V, limits of -100 to 100 degrees, a moment of inertia from 12 in and 1 lb, and a 40 A current limit. An A301 uses its own 64:1 gearbox and no current limit.
* The `Pivot` tests add a `Pivot` with angles from -100 to 150 degrees, unless a test says otherwise.
* The movement tests pass if the pivot moved or the motor controller applied a nonzero duty cycle at any point. For the TalonFX and TalonFXS, a duty cycle test that sees no movement prints a warning instead of failing, because Phoenix simulates Talons in real time.

## Tests

| Test                                        | What it checks                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             |
| ------------------------------------------- | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `PivotTest.SMCDutyCycle`                    | A 0.5 duty cycle command on the motor controller's subsystem moves the pivot within 1 s.                                                                                                                                                                                                                                                                                                                                                                                                                                   |
| `PivotTest.SMCPositionPID`                  | A command that calls `SetPosition(80 deg)` on the motor controller moves the pivot toward the setpoint. It runs for 20 s, or 1 s on a Talon.                                                                                                                                                                                                                                                                                                                                                                               |
| `PivotTest.PivotDutyCycle`                  | With a `Pivot` and its `Set(0.5)` command scheduled, a 0.5 duty cycle command moves the pivot.                                                                                                                                                                                                                                                                                                                                                                                                                             |
| `PivotTest.PivotPositionPID`                | With a `Pivot` and its `RunTo(80 deg)` command scheduled, the pivot moves toward 80 degrees.                                                                                                                                                                                                                                                                                                                                                                                                                               |
| `PivotTest.HardStops`                       | A pivot with hard stops at -30 and 60 degrees, driven at 0.5 and then -0.5 duty cycle for 2 s each, never goes more than 1 degree past the stop over the next 0.5 s. Runs on every motor controller, without a profile only (6 cases).                                                                                                                                                                                                                                                                                     |
| `PivotTest.LooselyCoupledFollowers`         | A loosely coupled follower reaches the leader's 45 degree setpoint within 3 degrees. When the leader is stopped halfway back to 0 with a 0 duty cycle command, the follower's duty cycle stays under 0.01 for 1 s. When the leader's closed loop starts again, the follower reaches 45 degrees again. Sections: SPARK MAX following a SPARK Flex; Nova (trapezoidal profile) following a SPARK MAX; SPARK MAX with an exponential profile following a SPARK Flex; A301 following a SPARK MAX; SPARK MAX following an A301. |
| `PivotTest.SimStartsAtStartingPosition`     | A SPARK MAX pivot with a 45 degree starting position has its simulation at 45 degrees as soon as the `Pivot` is built, without calling `SetupSimulation()`.                                                                                                                                                                                                                                                                                                                                                                |
| `PivotTest.ContinuousTurnsPastWhereItWraps` | A SPARK MAX pivot with continuous wrapping over 0 to 1 rotation, starting at 0.9 rotations, driven at 0.5 duty cycle for 1 s, turns past 1.05 rotations instead of stopping where it wraps.                                                                                                                                                                                                                                                                                                                                |

## Run it

From the `vendordep` folder:

```bash
./gradlew catchTests "-PcatchFilter=[PivotTest]"
```
