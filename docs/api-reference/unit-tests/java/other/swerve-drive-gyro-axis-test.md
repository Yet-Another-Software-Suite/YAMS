# SwerveDriveGyroAxisTest

**Group:** [other](./)\
**Source:** `vendordep/src/test/java/yams/core/mechanisms/config/SwerveDriveGyroAxisTest.java`

Checks `SwerveDriveConfig.withGyroHeadingAxis(GyroAxis)`, for a gyro mounted on its side. The gyro's attitude is rotated into the robot's frame, so the heading follows the robot turning about whichever gyro axis points up, and the tilt that anti-tipping reads is still the robot's.

## Setup

* A Commands v2 `SwerveDriveConfig` with no modules, whose gyro returns an attitude each test sets.

## Tests

| Test                                                   | What it checks                                                                                                                                                                       |
| ------------------------------------------------------ | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| `yawHeadingFollowsGyroZ`                               | With `YAW`, turning the gyro about its Z axis through 0, 45, 89, 91, 135, 179, 181, 270 and 359°, then back through 181, 90, 0, -90 and -179°, gives that heading, wrapped to ±180°. |
| `rollHeadingFollowsGyroX`                              | With `ROLL`, turning the gyro about its X axis through the same angles gives the same headings.                                                                                      |
| `pitchHeadingFollowsGyroYPastAQuarterRotation`         | With `PITCH`, turning the gyro about its Y axis through the same angles gives the same headings, past a quarter rotation, which a `Rotation3d`'s pitch alone cannot reach.           |
| `inversionAppliesToTheChosenAxis`                      | With `ROLL` and inverted, a 30° gyro roll gives a heading of -30°.                                                                                                                   |
| `inversionChangesOnlyTheHeading`                       | With `YAW` and inverted, a gyro at 5° roll, -8° pitch and 30° yaw gives a heading of -30° and keeps the roll and pitch.                                                              |
| `tiltIsTheRobotsForSideMountedGyros`                   | With `ROLL`, a gyro rotation about its Y axis reads as robot pitch. With `PITCH`, a gyro rotation about its X axis reads as robot roll.                                              |
| `antiTippingCorrectsTheRobotsTiltWithASideMountedGyro` | With `ROLL` and anti-tipping on, the robot tipping forward 15° gives a forward correction of 4 times sin(15°) and no sideways correction.                                            |

## Run it

From the `vendordep` folder:

```bash
./gradlew test --tests "yams.core.mechanisms.config.SwerveDriveGyroAxisTest"
```
