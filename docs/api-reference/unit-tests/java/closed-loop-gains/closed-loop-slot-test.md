# ClosedLoopSlotTest

**Group:** [closed-loop-gains](./)\
**Source:** `vendordep/src/test/java/yams/core/motorcontrollers/ClosedLoopSlotTest.java`

Checks that every motor controller runs the gains of the selected closed loop slot. Each slot gets different gains, so running another slot's gains gives the wrong voltage.

## Setup

* Uses the shared `ClosedLoopCases` helper: SPARK MAX and SPARK Flex (4 slots), TalonFX and TalonFXS (3 slots), Nova (2 slots) and A301 (4 slots).
* Slot gains grow with the slot number: kP is 2.5, 5, 7.5 and 10, kV is 0.25, 0.5, 0.75 and 1, and kS is 0.05, 0.1, 0.15 and 0.2.
* The mechanism has a 1000 kg m² moment of inertia, so it barely moves while a gain is measured. The simulated battery is reset to 12 V.

## Tests

| Test                                | What it checks                                                                                                                                                                                                                           |
| ----------------------------------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `proportionalGainOfTheSelectedSlot` | After `setClosedLoopSlot`, a 0.1 rotation position error gives that slot's kP x 0.1. Runs on every slot of every motor controller.                                                                                                       |
| `velocityGainOfTheSelectedSlot`     | After `setClosedLoopSlot`, velocity control at 1 rotation per second gives that slot's kV. Runs on every slot of every motor controller, but skips SPARK slots other than 0, because REVLib's simulated SPARK only applies kV in slot 0. |
| `gainsReachTheMatchingSparkSlot`    | On a SPARK MAX, each YAMS slot's gains reach the SPARK slot with the same number, in SPARK units: kP / (12 x 12), kS in volts, and kV per motor RPM. Runs on each of the four slots.                                                     |

## Run it

From the `vendordep` folder:

```bash
./gradlew test --tests "yams.core.motorcontrollers.ClosedLoopSlotTest"
```
