# absolute-encoder-ranges

Every absolute encoder under every closed loop controller, reporting the range continuous wrapping sets.

## Test classes

| Test class                                                 | Tests | What it checks                                                                                                                      |
| ---------------------------------------------------------- | ----- | ----------------------------------------------------------------------------------------------------------------------------------- |
| [AbsoluteEncoderRangeTest](absolute-encoder-range-test.md) | 1     | Checks that continuous wrapping sets the range every absolute encoder reports, under every closed loop controller, across 84 cases. |

## Run the group

From the `vendordep` folder:

```bash
./gradlew test --tests "yams.core.motorcontrollers.AbsoluteEncoderRangeTest"
```
