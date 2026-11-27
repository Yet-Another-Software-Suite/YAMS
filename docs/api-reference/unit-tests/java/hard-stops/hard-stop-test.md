# HardStopTest

**Group:** [hard-stops](./)\
**Source:** `vendordep/src/test/java/yams/mechs/HardStopTest.java`

Checks that a simulated pivot stops at its hard stops as a real one does, and that a simulated Talon keeps an encoder position it is set to. Without these, a simulated mechanism can fly past its limits or report the wrong position after zeroing.

## Setup

* A pivot with hard stops at -30° and 60°, on a 60:1 gearbox with 1 lb at 4 inches, a kP of 4, a 40 A stator limit and brake mode, starting at 0°.
* The Nova gets a trapezoidal profile, as it applies a feedforward only with a motion profile. The A301 runs on its 64:1 gearbox without current limits.

## Tests

| Test                                         | What it checks                                                                                                                                                                                                                                                                           |
| -------------------------------------------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `pivotStaysOnItsHardStops`                   | Driven at a 0.5 duty cycle into the upper stop, then at -0.5 into the lower one, the simulated pivot never goes more than 1° past the stop over 500 ms of pushing, and the encoder reads within 1° of the stop. Runs on each of SPARK MAX, SPARK Flex, TalonFXS, TalonFX, Nova and A301. |
| `talonKeepsAnEncoderPositionItIsSet`         | After the pivot is moved more than 5° and the encoder is set to 0°, both the simulated pivot and the encoder read within 1° of 0°: the move is not counted twice. Runs on the TalonFXS and the TalonFX.                                                                                  |
| `pivotSimulationStartsAtItsStartingPosition` | On a SPARK MAX with a starting position of 45°, a new `Pivot` starts its simulation at 45°, without `setupSimulation()` being called.                                                                                                                                                    |
| `continuousPivotTurnsPastWhereItWraps`       | A SPARK MAX pivot with continuous wrapping over 0 to 1 rotation, starting at 0.9 rotations, driven at a 0.5 duty cycle for 1 s, turns past 1.05 rotations instead of stopping where it wraps.                                                                                            |

## Run it

From the `vendordep` folder:

```bash
./gradlew test --tests "yams.mechs.HardStopTest"
```
