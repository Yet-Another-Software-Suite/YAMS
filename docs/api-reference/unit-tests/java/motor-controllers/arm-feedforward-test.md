# ArmFeedforwardTest

**Group:** [motor-controllers](./)\
**Source:** `vendordep/src/test/java/yams/core/motorcontrollers/ArmFeedforwardTest.java`

Checks that an `ArmFeedforward` adds to the YAMS closed loop output when no motion profile is set. It uses an LQR, which always runs in YAMS, so the arm's gravity term must come from the feedforward.

## Setup

* Each motor controller gets a 12:1 gearing, a 40 A stator limit, brake mode and closed loop control, starting at 0 degrees.
* The closed loop controller is an LQR for an arm, and the feedforward is `ArmFeedforward(0, 3, 0)` (kG = 3 V).
* Each wrapper overrides `setVoltage` to record the last voltage the closed loop controller asks for.

## Tests

| Test                                       | What it checks                                                                                                                                                                                                                                                                |
| ------------------------------------------ | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `armFeedforwardContributesWithoutAProfile` | Holding a horizontal arm at its 0 degree setpoint, one `iterateClosedLoopController()` call commands about kG (between 0.75 and 1.25 times 3 V). The LQR has no error, so this voltage comes from the feedforward alone. Runs on SPARK MAX, TalonFX, TalonFXS, Nova and A301. |

## Run it

From the `vendordep` folder:

```bash
./gradlew test --tests "yams.core.motorcontrollers.ArmFeedforwardTest"
```
