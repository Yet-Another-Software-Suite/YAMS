# closed-loop-gains

Every motor controller given the same gains in YAMS units: the voltage of each feedback and feedforward gain, and the gains of the selected closed loop slot.

## Test classes

| Test class                                     | Tests | What it checks                                                                                                                        |
| ---------------------------------------------- | ----- | ------------------------------------------------------------------------------------------------------------------------------------- |
| [FeedbackTest](feedback-test.md)               | 5     | kP, kI and kD give the same voltage in YAMS units on every motor controller, for position, velocity and linear control.               |
| [FeedforwardTest](feedforward-test.md)         | 8     | kS, kV, kA and kG give the same voltage in YAMS units on every motor controller, with velocity control, profiles, elevators and arms. |
| [ClosedLoopSlotTest](closed-loop-slot-test.md) | 3     | Every motor controller runs the kP and kV of the selected slot, and each YAMS slot's gains reach the matching SPARK slot.             |

## Run the group

From the `vendordep` folder:

```bash
./gradlew test --tests "yams.core.motorcontrollers.FeedbackTest" \
  --tests "yams.core.motorcontrollers.FeedforwardTest" \
  --tests "yams.core.motorcontrollers.ClosedLoopSlotTest"
```
