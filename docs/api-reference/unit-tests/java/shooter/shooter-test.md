# ShooterTest

**Group:** [shooter](./)\
**Source:** `vendordep/src/test/java/yams/mechs/ShooterTest.java`

Checks that a `FlyWheel` shooter and its motor controller respond to duty cycle and velocity commands in simulation, on every motor controller YAMS supports. It catches a motor controller or velocity loop setup that leaves a flywheel still.

## Setup

* Each parameterized test runs on 24 cases: SPARK MAX (NEO), SPARK Flex (Vortex), TalonFXS (two NEOs), TalonFX (Kraken X60), Nova (NEO) and A301, each with four closed loop setups: PID only, PID with a trapezoidal profile, PID with an exponential profile, and feedforward only (all PID gains 0).
* The shared config uses a 12:1 gearbox, a kP of 100, a `SimpleMotorFeedforward` with a kV of 1, a 40 A stator limit and coast mode. The flywheel is 4 inches across.
* The Nova gets a velocity trapezoidal profile in the PID only and feedforward only cases, as a Nova applies its feedforward only with a motion profile. The A301 runs on its 3:1 launcher gearbox without current limits.
* The Talons are paced in real time, as Phoenix simulates them in real time.

## Tests

| Test                     | What it checks                                                                                                                                                                     |
| ------------------------ | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `testSMCDutyCycle`       | Commanding the motor controller to a 0.5 duty cycle for 1 s raises the velocity or the position, or the reported duty cycle is not zero.                                           |
| `testSMCVelocityPID`     | Commanding the motor controller to 2000 RPM changes the velocity by more than 0.05°/s, or the reported duty cycle is not zero while it runs (1 s for a Talon, 2 s for the others). |
| `testShooterDutyCycle`   | The duty cycle check, through the shooter's own `set(0.5)` command.                                                                                                                |
| `testShooterVelocityPID` | The velocity check, through the shooter's `run(80 RPM)` command.                                                                                                                   |

## Run it

From the `vendordep` folder:

```bash
./gradlew test --tests "yams.mechs.ShooterTest"
```
