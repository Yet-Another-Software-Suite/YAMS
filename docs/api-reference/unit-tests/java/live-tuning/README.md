# live-tuning

Live Tuning: every motor controller applying tuned values, the Live Tuning command a mechanism registers, and driving a leader and its follower at the tuning voltage.

## Test classes

| Test class                                               | Tests | What it checks                                                                                                                                                               |
| -------------------------------------------------------- | ----- | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| [LiveTuningTest](live-tuning-test.md)                    | 1     | The tuning table shows the config's values and applies entered gains and profile values exactly to the config and in device units, across 36 controller and mechanism cases. |
| [MechanismLiveTuningTest](mechanism-live-tuning-test.md) | 1     | Arm, Elevator, Pivot and FlyWheel register the motor's Live Tuning command only when the motor telemetry is HIGH.                                                            |
| [VoltageLiveTuningTest](voltage-live-tuning-test.md)     | 2     | The Live Tuning voltage drives a leader and its loosely coupled follower, holds a typed voltage steady, and hands back to the closed loop when turned off or ended.          |

## Run the group

From the `vendordep` folder:

```bash
./gradlew test --tests "yams.core.motorcontrollers.LiveTuningTest" \
  --tests "yams.core.telemetry.MechanismLiveTuningTest" \
  --tests "yams.mechs.VoltageLiveTuningTest"
```
