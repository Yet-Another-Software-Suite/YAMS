# NovaConfigTest

**Group:** [closed-loop-gain-units](./)\
**Source:** `vendordep/src/test/java/yams/core/motorcontrollers/NovaConfigTest.java`

Checks what `NovaWrapper` sends to a Thrifty Nova, read back from the Nova, and the configs it rejects. The Nova runs every closed loop except an LQR, an exponential profile and a velocity profile, so options that only the SystemCore's closed loop applies are rejected for the rest.

## Setup

* Each Nova drives a NEO with a 12:1 gearing, a 40 A stator limit, closed loop control and kP = 4.
* A loop on the Nova has a 1 ms closed loop period. A loop on the SystemCore has a 20 ms period.
* Values are read back from the Nova's status, waiting until they arrive.

## Tests

| Test                                            | What it checks                                                                                                                                                                                                                                                                                           |
| ----------------------------------------------- | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `feedforwardWithoutAMotionProfile`              | The loop runs on the Nova for a velocity kS and kV, an arm's kS and kG (which reach the Nova's motion profile slot), a profiled arm feedforward, PID alone, and a feedforward with kA.                                                                                                                   |
| `velocityKvReachesTheVelocity`                  | A velocity loop with kP = 0.1 and a kV of 12 V over the free speed runs on the Nova, and reaches 3 rotations per second (within 0.15) after 150 simulation steps.                                                                                                                                        |
| `systemCoreOptionsNeedTheSystemCoresClosedLoop` | A temperature cutoff or a closed loop maximum voltage throws `SmartMotorControllerConfigurationException`. A temperature cutoff with an exponential profile (run on the SystemCore) does not throw.                                                                                                      |
| `toleranceIsTheSlotsAllowableError`             | A 9 degree closed loop tolerance sets the allowable error of slots 0 and 1 to 0.025 mechanism rotations.                                                                                                                                                                                                 |
| `novaHasTwoSlots`                               | Gains in slot 2 throw `SmartMotorControllerConfigurationException`.                                                                                                                                                                                                                                      |
| `slotOneGainsGoToTheNovasSecondSlot`            | Slot 1's kP of 6 reaches the Nova's slot 1, and slot 0's kP of 4 reaches its slot 0.                                                                                                                                                                                                                     |
| `mechanismLimitsAndWrappingAreTheNovas`         | Soft limits of -90 and 45 degrees are enabled on the Nova as -0.25 and 0.125 mechanism rotations. A CAN encoder closing the loop does not wrap by default, and with continuous wrapping from 0 to 1 it wraps and soft limits are off.                                                                    |
| `wrappingRunsOnTheNovaWhereItCan`               | The loop runs on the Nova when continuously wrapping a CAN encoder from -0.5 to 0.5, and when wrapping the motor's own encoder. It runs on the SystemCore for a CAN encoder read from -0.5 to 0.5 without continuous wrapping, and for continuous wrapping of a CAN encoder geared 2:1 to the mechanism. |
| `motorEncoderWrappingTakesTheShortWay`          | Wrapping the motor's encoder from -0.5 to 0.5, starting at 3.2 rotations with a target of 0.25, the mechanism goes to 3.25 rotations (within 0.02) instead of turning back to 0.25.                                                                                                                      |
| `feedbackSensorAndItsRatio`                     | With no external encoder the Nova uses `INTERNAL` with a ratio of 12. With a CAN encoder geared 2:1 it uses `CAN_ENCODER`, pairs with that encoder's CAN ID, and has a ratio of 2.                                                                                                                       |
| `canEncoderReadsTheSimulatedMechanism`          | With a CAN encoder geared 2:1, setting the mechanism to 0.3 rotations and 1.5 rotations per second makes the CAN encoder read 0.6 rotations and 3 rotations per second, and the wrapper's external encoder reads 0.3 and 1.5 back.                                                                       |

## Run it

From the `vendordep` folder:

```bash
./gradlew test --tests "yams.core.motorcontrollers.NovaConfigTest"
```
