# SwerveInputStreamAimTest (Commands v2)

**Group:** [swerve-drive](./)\
**Source:** `vendordep/src/test/java/yams/commands2/swerve/SwerveInputStreamAimTest.java`

Checks the aim options of the Commands v2 `SwerveInputStream`: `withAim`, `withAimLookahead`, `withAimFeedforward`, `withAimHeadingOffset`, and the `aimLock` and `isAimLocked` checks. These decide how fast the robot turns to face a target while the driver moves it. The [Commands v3 version](swerve-input-stream-aim-test-commands3.md) runs the same tests.

## Setup

* A simulated four module drive on SPARK MAX motor controllers, built once for the class, with a rotation controller of kP 1. The angular velocity the stream asks for is the heading error in radians.
* The drive's pose, heading and field relative velocity are set by each test instead of being simulated. Each test starts at the origin, facing 0°, at rest.
* Each stream has centered sticks and aims at a field position while aiming is on.

## Tests

| Test                                             | What it checks                                                                                                                                                                                                                                         |
| ------------------------------------------------ | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| `aimTurnsTowardTheTarget`                        | A target straight to the left asks for π/2 rad/s. A robot already facing left asks for no turn.                                                                                                                                                        |
| `targetVectorIsFromTheRobotToTheTarget`          | From a robot at (1, 1), the target vector to (3, 4) is (2, 3), whatever the robot's heading.                                                                                                                                                           |
| `lookaheadMovesTheRobotAlongItsFieldVelocity`    | At 2 m/s along X with a 0.5 s lookahead, the stream aims from 1 m further along X, so the target at (1, 2) is straight to the left and the stream asks for π/2 rad/s. Without the lookahead it aims from where the robot is now.                       |
| `lookaheadIsFieldRelativeWhateverTheRobotFaces`  | The lookahead follows the field relative velocity, so a robot facing 90° gives the same target vector as one facing 0°.                                                                                                                                |
| `headingOffsetRotatesTheAimHeading`              | A 90° heading offset on a target straight ahead asks for π/2 rad/s. The same offset with its supplier returning false asks for no turn.                                                                                                                |
| `feedforwardIsAddedToTheRotation`                | A feedforward with kV 0.5 adds half of the controller's π/2 rad/s, both in `get()` and in `calculateAngularVelocity`.                                                                                                                                  |
| `aimLockIsTrueOnlyWhileAimingAndFacingTheTarget` | `isAimLocked` is false before the stream is read, then true for a target straight ahead within 1°. For a target 90° away it is false within 1° and true within 91°. Once aiming stops it is false again. `aimLock` gives the same answer as a trigger. |
| `cloneKeepsTheAimOptions`                        | A clone keeps the lookahead, heading offset and feedforward: the lookahead puts the target at 90°, the -90° offset brings that back to no turn, and the feedforward still adds half.                                                                   |

## Run it

From the `vendordep` folder:

```bash
./gradlew test --tests "yams.commands2.swerve.SwerveInputStreamAimTest"
```
