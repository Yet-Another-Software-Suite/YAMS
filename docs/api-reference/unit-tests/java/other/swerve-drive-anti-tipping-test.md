# SwerveDriveAntiTippingTest

**Group:** [other](./)\
**Source:** `vendordep/src/test/java/yams/core/mechanisms/config/SwerveDriveAntiTippingTest.java`

Checks `SwerveDriveConfig.withAntiTipping`. Once the robot's pitch or roll is past the threshold, a correction toward the side it is tipping to is added to every robot relative chassis speed. The correction is `kP` times the sine of the tilt, and no more than the maximum correction speed. With WPILib's convention, positive pitch tips the robot forward and positive roll tips it right.

## Setup

* A Commands v2 `SwerveDriveConfig` with no modules, whose gyro returns an attitude each test sets.
* Anti-tipping is set to a `kP` of 4 m/s, a 10° threshold, and a 1.5 m/s maximum correction.

## Tests

| Test                                            | What it checks                                                                                                                               |
| ----------------------------------------------- | -------------------------------------------------------------------------------------------------------------------------------------------- |
| `doesNotCorrectBelowTheThreshold`               | At 5° roll and -8° pitch, the correction is zero.                                                                                            |
| `drivesForwardUnderAForwardTip`                 | At 15° pitch, the correction drives forward at `kP` times sin(15°).                                                                          |
| `drivesRightUnderARightwardTip`                 | At 15° roll, the correction drives right (negative vy) at `kP` times sin(15°).                                                               |
| `correctsRelativeToTheRobotWhateverItsHeading`  | Tipping forward while facing 90° still gives a forward, robot relative correction.                                                           |
| `correctsNoFasterThanTheMaximumCorrectionSpeed` | At -40° pitch, the backward correction is capped at 1.5 m/s.                                                                                 |
| `addsTheCorrectionToRobotRelativeChassisSpeeds` | `optimizeRobotRelativeChassisSpeeds` adds the correction to the commanded vx and leaves vy and omega unchanged.                              |
| `withoutAntiTippingChassisSpeedsAreUnchanged`   | Without anti-tipping, a 40° tip leaves the chassis speeds unchanged and the correction is zero.                                              |
| `rejectsSettingsThatCannotCorrect`              | A `kP` of 0, a threshold of 0° or 90°, or a maximum speed of 0 throws `IllegalArgumentException`, and the rejected settings are not applied. |

## Run it

From the `vendordep` folder:

```bash
./gradlew test --tests "yams.core.mechanisms.config.SwerveDriveAntiTippingTest"
```
