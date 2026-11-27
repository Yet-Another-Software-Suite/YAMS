# PivotTest

**Group:** [pivot](./)\
**Source:** `vendordep/src/test/java/yams/mechs/PivotTest.java`

Checks that a `Pivot` and its motor controller respond to duty cycle and position commands in simulation, on every motor controller YAMS supports. It catches a motor controller or closed loop setup that leaves a pivot sitting still.

## Setup

* Each parameterized test runs on 18 cases: SPARK MAX (NEO), SPARK Flex (Vortex), TalonFXS (NEO), TalonFX (Kraken X60), Nova (NEO) and A301, each with three closed loop setups: PID only, PID with a trapezoidal profile, and PID with an exponential profile.
* The shared config uses a 60:1 gearbox, a kP of 4, a `SimpleMotorFeedforward` with a kS of 1, soft limits of -100° to 100°, a 40 A stator limit, brake mode, and the moment of inertia of 1 lb at 4 inches. The pivot's hard limits are -100° to 150°.
* The Nova gets a trapezoidal profile in its PID only case, as a Nova applies its feedforward only with a motion profile. The A301 runs on its 64:1 gearbox without current limits.
* The Talons are paced in real time, as Phoenix simulates them in real time.

## Tests

| Test                   | What it checks                                                                                                                                                                              |
| ---------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `testSMCDutyCycle`     | Commanding the motor controller to a 0.5 duty cycle for 1 s raises the velocity or the angle, or the reported duty cycle is not zero. A Talon that fails only prints a warning.             |
| `testSMCPositionPID`   | Commanding the motor controller to 80° moves the pivot. A Talon runs for 1 s and must move more than 0.0003°. The others run for 20 s, and a nonzero duty cycle while they run also passes. |
| `testPivotDutyCycle`   | The duty cycle check, through the pivot's own `set(0.5)` command.                                                                                                                           |
| `testPivotPositionPID` | The position check, through the pivot's `setAngle(80°)` command.                                                                                                                            |

## Run it

From the `vendordep` folder:

```bash
./gradlew test --tests "yams.mechs.PivotTest"
```
