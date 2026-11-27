# MechanismPoseTest

**Group:** [other](./)\
**Source:** `vendordep/src/test/java/yams/core/mechanisms/MechanismPoseTest.java`

Checks each mechanism's `getPose3d()` against poses worked out by hand: where it is mounted, how its motor's position turns or moves it, in each movement plane, on a turned mount, and on another mechanism. It also checks the stage poses of multi stage elevators.

## Setup

* The first six tests use `MechanismPositionConfig` alone, with no motor controllers.
* The rest build each mechanism on a SPARK MAX with 10:1 gearing, set to a starting position, and read its pose without running the simulation.
* Elevator stage tests mount the elevator 0.1 m up with the carriage at 0.9 m.

## Tests

| Test                                                   | What it checks                                                                                                                                                                                                |
| ------------------------------------------------------ | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `anArmInTheXzPlaneTurnsUpTowardZ`                      | An arm mounted at (0.2, 0, 0.5) points forward at 0°, straight up at 90°, and forward and down at -30°.                                                                                                       |
| `anArmInTheYzPlaneTurnsFromYUpTowardZ`                 | In the YZ plane, an arm points along +Y at 0° and along +Z at 90°.                                                                                                                                            |
| `aTurretInTheXyPlaneTurnsFromXTowardY`                 | In the XY plane, a turret turned 90° points along +Y, and turned 180° points along -X.                                                                                                                        |
| `anElevatorSlidesAlongItsAngle`                        | An upright elevator's carriage at 0.7 m is 0.7 m above the mount and does not turn. An elevator at 60° (leaning 30° forward) moves the carriage forward and up along that angle.                              |
| `aTurnedMountTurnsTheMechanism`                        | An arm on a mount turned 90° to face left points to the left at 0° and up at 90°.                                                                                                                             |
| `aMechanismOnAnotherMovesWithIt`                       | An arm mounted on an elevator's carriage moves up with the carriage as it rises, and its end follows.                                                                                                         |
| `anArmsPoseFollowsItsAngle`                            | An `Arm` at 90° points straight up from its mount.                                                                                                                                                            |
| `aPivotsPoseFollowsItsAngle`                           | A `Pivot` in the XY plane at 90° points along +Y.                                                                                                                                                             |
| `aPivotTurnsInTheXyPlaneByDefault`                     | A `Pivot` with no movement plane set turns in the XY plane.                                                                                                                                                   |
| `anElevatorsPoseFollowsItsHeight`                      | An `Elevator` mounted 0.1 m up with its carriage at 0.7 m puts the carriage 0.8 m up.                                                                                                                         |
| `aCascadeElevatorsStagesEachMoveTheirShareOfItsHeight` | With three equal stages, `getCascadePose3d` puts them at a third, two thirds and all of the carriage's height. The last stage is the carriage's pose.                                                         |
| `aCascadeElevatorsStagesMoveInProportionToTheirTravel` | With stages of 0.6, 0.3 and 0.3 m, the first stage moves half the carriage's height and the others a quarter each.                                                                                            |
| `aContinuousElevatorsCarriageMovesFirst`               | With `getContinuousPose3d(true)` and three 0.5 m stages, the carriage moves its first 0.5 m, the second stage the other 0.4 m, and the first stage does not move.                                             |
| `aContinuousElevatorsCarriageMovesLast`                | With `getContinuousPose3d(false)`, the first stage moves its 0.5 m first, then the second stage the other 0.4 m, and the carriage does not move on the second stage.                                          |
| `anElevatorWithOnlyACarriageHasNoStagePoses`           | An elevator with one stage throws `ElevatorConfigurationException` for cascade and continuous stage poses, but still has a carriage pose.                                                                     |
| `elevatorStagesNeedAPositiveHeightForEachStage`        | `withStages()` with no heights, or with a 0 m height, throws `IllegalArgumentException`. Three heights give three stages. An elevator with no stages throws `ElevatorConfigurationException` for stage poses. |
| `anElevatorsHeightIsItsStagesWithoutHardLimits`        | With no hard limits, three 0.5 m stages give a maximum height of 1.5 m and a minimum of 0, and the carriage pose follows the motor.                                                                           |
| `elevatorStageHeightsAddUpToItsMaximumHeight`          | Stage heights of 0.6, 0.6 and 0.5 m total 1.7 m, and no stages give no total. Stages adding up to 1 m on a 2 m elevator throw `ElevatorConfigurationException` with a message saying so.                      |
| `aFlywheelsPoseFollowsItsPosition`                     | A `FlyWheel` turned 90° turns a point on the wheel from +X to +Z.                                                                                                                                             |
| `aDoubleJointedArmsUpperArmIsAtTheEndOfItsLowerArm`    | On a `DoubleJointedArm` with the lower arm up and the upper arm level, the upper arm's pivot is at the lower arm's end (the elbow) and the upper arm points forward from it.                                  |

## Run it

From the `vendordep` folder:

```bash
./gradlew test --tests "yams.core.mechanisms.MechanismPoseTest"
```
