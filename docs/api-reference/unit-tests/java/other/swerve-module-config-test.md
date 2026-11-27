# SwerveModuleConfigTest

**Group:** [other](./)\
**Source:** `vendordep/src/test/java/yams/core/mechanisms/config/SwerveModuleConfigTest.java`

Checks swerve module optimization in `SwerveModuleConfig.getOptimizedState` and `SwerveModuleVelocityWithAzimuth.optimize`. A wheel turns at most 90° and reverses its drive instead. It keeps the orientation it was last sent to unless the other one is clearly better, so a moving target does not flip it back and forth. A wheel commanded too slowly to steer by holds where it is.

## Setup

* A `SwerveModuleConfig` with no motor controllers, optimization on, and a 0.1 m/s velocity deadband. Each test passes in the wheel's measured angle.

## Tests

| Test                                                | What it checks                                                                                                                                               |
| --------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| `turnsAtMostAQuarterTurnAndReversesTheDrive`        | A target of 150° from a wheel at 0° turns to -30° and drives backward.                                                                                       |
| `reversesTheDriveForATargetBehindTheWheel`          | After a target of 0°, a target of 180° keeps the wheel at 0° and drives backward.                                                                            |
| `turningWheelIsNotReversedBackAndForth`             | With the wheel still near 0° and a target that moves between 68° and 112° from loop to loop, each commanded angle is less than 90° from the one before.      |
| `turnsAroundWhenTheOtherOrientationIsClearlyBetter` | A target of 110° from 0° turns to -70°. Once the wheel has reached 90°, the same target turns to 110° and drives forward.                                    |
| `tooSlowToSteerByHoldsTheWheel`                     | A 0.05 m/s target holds the wheel at 30° and does not drive. The next target, 140°, is optimized from the held 30°, so it turns to -40° and drives backward. |
| `withoutALastAngleOrHysteresisOptimizesLikeWpilib`  | With no last angle, or with zero hysteresis, the result matches WPILib's own `optimize`.                                                                     |

## Run it

From the `vendordep` folder:

```bash
./gradlew test --tests "yams.core.mechanisms.config.SwerveModuleConfigTest"
```
