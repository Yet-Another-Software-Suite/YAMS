# closed-loop-gain-units

The units of the gains each motor controller is given: per rotation for an arm, per meter for an elevator, on the motor controller and on the SystemCore.

## Test classes

| Test class                                                | Tests | What it checks                                                                                                                                |
| --------------------------------------------------------- | ----- | --------------------------------------------------------------------------------------------------------------------------------------------- |
| [ClosedLoopGainUnitsTest](closed-loop-gain-units-test.md) | 8     | Arm gains are per rotation and elevator gains per meter, checked on TalonFX, SPARK MAX, Nova and A301 and on the SystemCore loop.             |
| [NovaConfigTest](nova-config-test.md)                     | 11    | What NovaWrapper sends to the Nova (gains, slots, tolerance, limits, wrapping, feedback sensor) and which SystemCore only options it rejects. |

## Run the group

From the `vendordep` folder:

```bash
./gradlew test --tests "yams.core.motorcontrollers.ClosedLoopGainUnitsTest" \
  --tests "yams.core.motorcontrollers.NovaConfigTest"
```
