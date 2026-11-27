# EasyCRTTest

**Group:** [other](./)\
**Source:** `vendordep/src/test/native/cpp/EasyCRTTest.cpp`\
**Tags:** `[EasyCRT]`

Checks the math in `EasyCRT`, which finds a mechanism's position past one rotation from two absolute encoders on different pinions, using the Chinese Remainder Theorem. No hardware or simulation is used.

## Setup

* Most tests use a 100 tooth shared gear with 19 and 21 tooth encoder pinions. Its unique range is lcm(19, 21) / 100 = 3.99 rotations of the shared gear.
* A helper gives the reading an ideal absolute encoder would show for a gear position, wrapped into one rotation.

## Tests

| Test                                                  | What it checks                                                                                                                             |
| ----------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------ |
| `EasyCRT unique range is lcm over the shared gear`    | `GetUniqueRange()` is 3.99 rotations for 100, 19 and 21 teeth, and 0.6 rotations for 100, 20 and 30 teeth, since 20 and 30 share a factor. |
| `EasyCRT solves across the centered range`            | `Solve` returns the gear position, within 1e-6 rotations, at points across the range centered on zero.                                     |
| `EasyCRT solves across an off center range`           | `Solve` with a range of 0 to 3.5 rotations returns the gear position at points across that range.                                          |
| `EasyCRT accepts unwrapped and negative readings`     | Readings that are not wrapped into one rotation, for a gear at -1.3 rotations, still solve to -1.3.                                        |
| `EasyCRT tolerates backlash on either encoder`        | With 2 degrees of slop added to encoder A and taken from encoder B, the result is still within the slop seen through encoder A's gearing.  |
| `EasyCRT is empty when the encoders disagree`         | Readings halfway between two candidates (encoder B off by 1 / 42 of a rotation) return no result.                                          |
| `EasyCRT is empty for readings that are not numbers`  | A NaN reading returns no result.                                                                                                           |
| `EasyCRT rejects travel longer than the unique range` | `Solve` throws `std::invalid_argument` for a 0 to 4 rotation range (longer than 3.99), and for a range whose minimum is above its maximum. |
| `EasyCRT rejects tooth counts that are not positive`  | Building an `EasyCRT` with a zero or negative tooth count on any gear throws `std::invalid_argument`.                                      |

## Run it

From the `vendordep` folder:

```bash
./gradlew catchTests "-PcatchFilter=[EasyCRT]"
```
