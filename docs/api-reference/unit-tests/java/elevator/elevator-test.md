# ElevatorTest

**Group:** [elevator](./)\
**Source:** `vendordep/src/test/java/yams/mechs/ElevatorTest.java`

Checks that an `Elevator` and its motor controller respond to duty cycle and height commands in simulation, on every motor controller YAMS supports. It also checks that the elevator turns a height setpoint into drum rotations correctly.

## Setup

* Each parameterized test runs on 18 cases: SPARK MAX (NEO), SPARK Flex (Vortex), TalonFXS (NEO), TalonFX (Kraken X60), Nova (NEO) and A301, each with three closed loop setups: PID only, PID with a trapezoidal profile, and PID with an exponential profile.
* The shared config uses a drum circumference of 5.5 inches (22 teeth at 0.25 inch), a 12:1 gearbox, a kP of 30 V per meter, soft limits of 0 m to 5 m, a 40 A stator limit and brake mode. The elevator has a 16 lb carriage and hard limits of 0 m to 3 m.
* The Nova gets a trapezoidal profile in its PID only case, as a Nova applies its feedforward only with a motion profile. The A301 runs on its 64:1 gearbox without current limits.
* The Talons are paced in real time, as Phoenix simulates them in real time.

## Tests

| Test                                                   | What it checks                                                                                                                                                                                                           |
| ------------------------------------------------------ | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| `testSMCDutyCycle`                                     | Commanding the motor controller to full duty cycle for 1 s raises the velocity or the height, or the reported duty cycle is not zero.                                                                                    |
| `testSMCPositionPID`                                   | Commanding the motor controller to 2 m for 1 s moves it more than 5 mm. For all but the Talons, a nonzero duty cycle while it runs also passes.                                                                          |
| `testElevatorDutyCycle`                                | The duty cycle check, through the elevator's own `set(1)` command.                                                                                                                                                       |
| `testElevatorPositionPID`                              | The position check, through the elevator's `setHeight(2 m)` command.                                                                                                                                                     |
| `measurementSetpointIsTheMechanismSetpointAsADistance` | On a SPARK MAX, the measurement setpoint is empty until a position is set. After `setMeasurementPositionSetpoint(0.5 m)` it reads 0.5 m, and the mechanism setpoint is 0.5 m divided by the circumference, in rotations. |

## Run it

From the `vendordep` folder:

```bash
./gradlew test --tests "yams.mechs.ElevatorTest"
```
