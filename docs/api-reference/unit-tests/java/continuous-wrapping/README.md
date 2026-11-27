# continuous-wrapping

Every motor controller, closed loop controller and attached encoder crossing the continuous wrapping point.

## Test classes

| Test class                                            | Tests | What it checks                                                                                                                                                                                                                                        |
| ----------------------------------------------------- | ----- | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| [ContinuousWrappingTest](continuous-wrapping-test.md) | 9     | Checks how continuous wrapping picks the nearest setpoint and how it sets each SPARK's onboard wrapping range, and that every motor controller and encoder under every closed loop controller crosses the wrapping point the short way in simulation. |

## Run the group

From the `vendordep` folder:

```bash
./gradlew test --tests "yams.core.motorcontrollers.ContinuousWrappingTest"
```
