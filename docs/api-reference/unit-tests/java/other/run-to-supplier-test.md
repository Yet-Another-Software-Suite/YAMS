# RunToSupplierTest

**Group:** [other](./)\
**Source:** `vendordep/src/test/java/yams/commands2/mechanisms/RunToSupplierTest.java`

Checks the Commands v2 `runTo(Supplier, tolerance)` command. It must follow the supplier while it runs, and end only once the mechanism is near the supplier's current value, not the value it had when the command was built.

## Setup

* A `Pivot` on a SPARK MAX with a NEO, 12:1 gearing, a closed loop of kP 20 and kD 0.5, starting at 0°, with soft limits of ±100°.
* Each test uses a 3° tolerance and steps the pivot's simulation every loop.

## Tests

| Test                                   | What it checks                                                                                                                                      |
| -------------------------------------- | --------------------------------------------------------------------------------------------------------------------------------------------------- |
| `followsATargetThatChangesWhileItRuns` | The target changes from 30° to 60° right after the command starts. Within 4 s the command ends with the pivot within 3° of 60°, not stopped at 30°. |
| `keepsRunningUntilItReachesTheTarget`  | With a 45° target, the command is still running after 0.04 s, before the pivot gets near it.                                                        |

## Run it

From the `vendordep` folder:

```bash
./gradlew test --tests "yams.commands2.mechanisms.RunToSupplierTest"
```
