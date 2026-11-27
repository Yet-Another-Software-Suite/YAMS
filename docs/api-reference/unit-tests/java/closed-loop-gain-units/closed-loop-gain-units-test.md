# ClosedLoopGainUnitsTest

**Group:** [closed-loop-gain-units](./)\
**Source:** `vendordep/src/test/java/yams/core/motorcontrollers/ClosedLoopGainUnitsTest.java`

Checks the units of the gains each motor controller is given. An `ArmFeedforward`'s kV and kA are per rotation, as YAMS documents, both on the motor controller and on the SystemCore (where WPILib's `ArmFeedforward` works in radians). A distance based controller's gains are per meter.

## Setup

* Each motor controller gets a 12:1 gearing, a 40 A stator limit, brake mode and closed loop control.
* Arm tests use kV = 1 V per rotation per second and kA = 0.5 V per rotation per second squared. Elevator tests use 0.2 m per mechanism rotation, kP = 40, kV = 8 and kA = 0.5 per meter, kS = 0.1 V and kG = 0.2 V.
* Gains are read back from the device. Talon values are checked within 0.01, since Phoenix stores them in fixed point.

## Tests

| Test                                         | What it checks                                                                                                                                                                                                                               |
| -------------------------------------------- | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `talonGetsArmGainsAsTheyAre`                 | A TalonFX gets the arm's kV and kA unchanged, since the Talon works per mechanism rotation.                                                                                                                                                  |
| `sparkGetsArmGainsPerRpmOfItsEncoder`        | A SPARK MAX gets the arm's kV and kA divided by the encoder RPM of one mechanism rotation per second (12 x 60).                                                                                                                              |
| `systemCoreUsesArmGainsPerRotation`          | A Nova with a fast velocity profile (run on the SystemCore) and kA = 0.05 commands about 2.5 V to reach 1 rotation per second in one 20 ms loop. Read per radian, the voltage would be 2π times larger.                                      |
| `a301UsesArmGainsPerRotation`                | The same check as `systemCoreUsesArmGainsPerRotation`, on an A301, whose closed loop always runs on the SystemCore.                                                                                                                          |
| `talonGetsElevatorGainsPerMechanismRotation` | A TalonFX gets the elevator's kP, kV and kA divided by mechanism rotations per meter (5), and kS and kG in volts.                                                                                                                            |
| `sparkGetsElevatorGainsPerEncoderRotation`   | A SPARK MAX gets kP / 12 per encoder rotation, kV and kA per encoder RPM, and kS and kG in volts.                                                                                                                                            |
| `novaGetsArmGainsPerMechanismRotation`       | A Nova arm with kP = 8, kI = 2, kD = 0.5 and a 2 rotations per second, 4 rotations per second squared profile gets a sensor to mechanism ratio of 12 and every gain and profile value as given, per mechanism rotation, with cosine gravity. |
| `novaGetsElevatorGainsPerMechanismRotation`  | A Nova elevator with a 1 m/s, 2 m/s² profile gets kP, kV and kA per mechanism rotation, a max velocity of 5 rotations per second, kS and kG in volts, and constant gravity.                                                                  |

## Run it

From the `vendordep` folder:

```bash
./gradlew test --tests "yams.core.motorcontrollers.ClosedLoopGainUnitsTest"
```
