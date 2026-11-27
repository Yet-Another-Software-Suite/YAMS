# BatterySimTest

**Group:** [other](./)\
**Source:** `vendordep/src/test/java/yams/core/motorcontrollers/simulation/BatterySimTest.java`

Checks that `BatterySim` models the shared simulated battery sagging and draining when several motor controllers draw heavy current at once, as on a real robot when several mechanisms run together. It also checks the battery's charge curve, its capacity derating, and that it is safe to use from several threads.

## Setup

* Most tests build six motor controllers in open loop, one per wrapper: SPARK MAX (NEO), SPARK Flex (NEO Vortex), TalonFXS (NEO), TalonFX (Kraken X60), Nova (NEO) and A301. Each has 12:1 gearing, an 80 A stator limit (none on the A301), and a heavy 2 kg m² load so it stays near stall.
* "Heavy load" means full duty cycle. Battery voltage is read as the lowest `RoboRioSim` input voltage over a window, since each mechanism updates it in turn within a loop.
* Discharge simulation is turned off and reset before and after every test.

## Tests

| Test                                                               | What it checks                                                                                                                                                                                                              |
| ------------------------------------------------------------------ | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `testHeavyLoadFromMultipleSmcsSagsBatteryVoltage`                  | The battery voltage drops below idle when one motor runs at full duty cycle, and drops further when all six do.                                                                                                             |
| `testSustainedHeavyLoadDrainsBatteryStateOfCharge`                 | With a 2 Ah battery, the state of charge starts at 1 and falls below 1 after 0.2 s of all six motors at full load.                                                                                                          |
| `testFullyChargedBatteryVoltage`                                   | A fully charged, idle battery reads above 12 V. Under heavy load it sags below that and its state of charge starts to fall.                                                                                                 |
| `testEmptyBatteryVoltage`                                          | A battery drained to a state of charge of 0 reads below 10 V once the motors stop.                                                                                                                                          |
| `testHighCurrentDrainsCapacityFasterThanLowCurrentForSameAmpHours` | Drawing 5.4 Ah at 54 A leaves less charge than drawing 5.4 Ah at 0.9 A (the Peukert effect). Steps 6 hours of simulated time. Disabled on CI (when `CI=true`) because it takes about 45 minutes there.                      |
| `testReplaceCapacityDeratingUsesCustomCurve`                       | After `replaceCapacityDerating` with a flat 0.5 fraction, drawing 30 A for an hour leaves less charge than with the default curve. Steps 2 hours of simulated time. Disabled on CI because it takes about 15 minutes there. |
| `testCalculateVoltageIsThreadSafe`                                 | 8 threads each add, update and remove 20,000 current draws at once without throwing, and afterward no current is left loading the battery (it reads 12 V).                                                                  |
| `testVoltageDoesNotRiseWithMoreSmcsUnderHeavyLoad`                 | From an equally fresh battery each time, all six motors at full load give no higher voltage than one motor alone (within 0.05 V).                                                                                           |
| `testVoltageDoesNotRiseWhenBatteryIsDead`                          | On a drained battery, running the motors at full load again gives no higher voltage than the battery idling (within 0.05 V).                                                                                                |

## Run it

From the `vendordep` folder:

```bash
./gradlew test --tests "yams.core.motorcontrollers.simulation.BatterySimTest"
```
