# ChineseRemainderTheoremTest

**Group:** [other](./)\
**Source:** `vendordep/src/test/java/yams/ChineseRemainderTheoremTest.java`

Checks `EasyCRT`, which finds a mechanism's position over more than one rotation from two absolute encoders on different pinions of the same gear (the Chinese remainder theorem). This lets a mechanism know where it is at boot without homing.

## Setup

* Unless a test says otherwise, the shared gear has 100 teeth and the encoder pinions have 19 and 21. The encoder readings are ideal, wrapped into one rotation.

## Tests

| Test                                    | What it checks                                                                                                                                                                             |
| --------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| `uniqueRangeIsLcmOverSharedGear`        | The unique range is the least common multiple of the pinion teeth over the shared gear's teeth: 3.99 rotations for 19 and 21, and 0.6 rotations for 20 and 30, which share a factor of 10. |
| `solvesAcrossTheCenteredRange`          | Across the range centered on 0, `solve` returns the gear's position within 1e-6 rotations at every step.                                                                                   |
| `solvesAcrossAnOffCenterRange`          | Given a range of 0 to 3.5 rotations, `solve` returns the gear's position across that range.                                                                                                |
| `acceptsUnwrappedAndNegativeReadings`   | Readings that are not wrapped into one rotation, and negative ones, still solve to the gear's position (-1.3 rotations).                                                                   |
| `toleratesBacklashOnEitherEncoder`      | With 2° of slop added to encoder A and taken from encoder B, `solve` still finds the position, off by no more than the slop seen through A's gearing.                                      |
| `emptyWhenTheEncodersDisagree`          | Readings halfway between two candidate positions return no solution.                                                                                                                       |
| `emptyForReadingsThatAreNotNumbers`     | A NaN reading returns no solution.                                                                                                                                                         |
| `rejectsTravelLongerThanTheUniqueRange` | A range of 4 rotations (longer than 3.99), or one whose end is below its start, throws `IllegalArgumentException`.                                                                         |
| `rejectsToothCountsThatAreNotPositive`  | A shared gear or pinion with 0 or negative teeth throws `IllegalArgumentException`.                                                                                                        |

## Run it

From the `vendordep` folder:

```bash
./gradlew test --tests "yams.ChineseRemainderTheoremTest"
```
