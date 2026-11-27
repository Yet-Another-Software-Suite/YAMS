# ArmTest

**Group:** [arm](./)\
**Source:** `vendordep/src/test/java/yams/mechs/ArmTest.java`

Checks that an `Arm` and its motor controller respond to duty cycle and position commands in simulation, on every motor controller YAMS supports. It catches a motor controller or closed loop setup that leaves an arm sitting still.

## Setup

* Each parameterized test runs on 18 cases: SPARK MAX (NEO), SPARK Flex (Vortex), TalonFXS (NEO), TalonFX (Kraken X60), Nova (NEO) and A301, each with three closed loop setups: PID only, PID with a trapezoidal profile, and PID with an exponential profile.
* The shared config uses a 12:1 gearbox, a kP of 5, an `ArmFeedforward` with a kG of 1, soft limits of -100° to 100°, a 40 A stator limit and brake mode. The arm is 4 inches long with hard limits of -100° to 200°.
* The Nova gets a trapezoidal profile in its PID only case, as a Nova applies its feedforward only with a motion profile. The A301 runs on its 64:1 gearbox without current limits.
* The Talons are paced in real time, as Phoenix simulates them in real time.

## Tests

| Test                                              | What it checks                                                                                                                                                                     |
| ------------------------------------------------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `testSMCDutyCycle`                                | Commanding the motor controller to a 0.5 duty cycle for 1.5 s makes it move: the velocity or position changes, or the reported duty cycle is not zero.                             |
| `testArmDutyCycle`                                | The same check, through the arm's own `set(0.5)` command.                                                                                                                          |
| `testSMCPositionPID`                              | Commanding the motor controller to 80° for 1 s moves the arm more than 0.05°. For all but the Talons, a nonzero duty cycle while it runs also passes.                              |
| `testArmPositionPID`                              | The same check, through the arm's `setAngle(80°)` command.                                                                                                                         |
| `measurementSetpointIsEmptyWithoutACircumference` | On a SPARK MAX, after `setMechanismPositionSetpoint(20°)` the arm has a mechanism setpoint but no measurement setpoint, as an arm has no circumference to turn it into a distance. |

## Run it

From the `vendordep` folder:

```bash
./gradlew test --tests "yams.mechs.ArmTest"
```
