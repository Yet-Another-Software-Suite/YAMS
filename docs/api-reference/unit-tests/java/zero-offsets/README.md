# zero-offsets

The external encoder zero offset on every absolute encoder, and how it moves the wrapping range.

## Test classes

| Test class                                             | Tests | What it checks                                                                                                                                                                                                                                                                                          |
| ------------------------------------------------------ | ----- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| [ZeroOffsetTest](zero-offset-test.md)                  | 3     | Checks that the external encoder zero offset reaches each absolute encoder and is applied once and the right way round, that a negative offset is stored as the same angle within one rotation, and that a SPARK, Nova or A301 without an absolute encoder rejects the offset while a Talon ignores it. |
| [ZeroOffsetWrappingTest](zero-offset-wrapping-test.md) | 1     | Checks that the wrapping range moves with the zero offset on every absolute encoder, across 40 cases.                                                                                                                                                                                                   |

## Run the group

From the `vendordep` folder:

```bash
./gradlew test --tests "yams.core.motorcontrollers.ZeroOffsetTest" \
  --tests "yams.core.motorcontrollers.ZeroOffsetWrappingTest"
```
