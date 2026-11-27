# ConfigCopyAndHelpersTest

**Group:** [motor-controllers](./)\
**Source:** `vendordep/src/test/java/yams/core/motorcontrollers/ConfigCopyAndHelpersTest.java`

Covers fixes found while reviewing the docs: copying a `SmartMotorControllerConfig`, the wheel moment of inertia helper, and the elevator exponential profile constraints. None of these tests create a motor controller.

## Tests

| Test                               | What it checks                                                                                                                                                                                     |
| ---------------------------------- | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `cloneDoesNotShareControllers`     | `clone()` gives the copy its own PID controller and `ArmFeedforward`. Changing kP and kG on the original leaves the copy at kP = 4 and kG = 0.5, so live tuning one motor does not change another. |
| `wheelMomentOfInertiaIsASolidDisk` | `withWheelMomentOfInertia(4 in, 1 lb)` gives a moment of inertia of 0.5 x mass x radius², the formula for a solid disk.                                                                            |
| `elevatorConstraintsAreFinite`     | `ExponentialProfilePIDController.createElevatorConstraints` for two NEOs, 16 lb, a 1 in drum and a 12:1 gearing gives finite `A` and `B`, and a maximum velocity above zero.                       |

## Run it

From the `vendordep` folder:

```bash
./gradlew test --tests "yams.core.motorcontrollers.ConfigCopyAndHelpersTest"
```
