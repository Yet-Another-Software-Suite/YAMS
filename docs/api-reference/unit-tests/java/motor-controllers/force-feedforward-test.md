# ForceFeedforwardTest

**Group:** [motor-controllers](./)\
**Source:** `vendordep/src/test/java/yams/core/motorcontrollers/ForceFeedforwardTest.java`

Checks the drive wheel feedforward `Force` you can pass to `setVelocity(velocity, force)`, for example from a PathPlanner setpoint generator. It covers the force to voltage and force to current math, the recorded feedforward force used by telemetry, and that the force alone moves the motor.

## Setup

* The config has a 4:1 gearing and a wheel of radius 0.05 m.
* The parameterized tests also use zero PID gains, brake mode, a 40 A stator limit and closed loop control. They run on SPARK MAX, TalonFX, TalonFXS, Nova (with a velocity profile) and A301.

## Tests

| Test                                           | What it checks                                                                                                                                                                                       |
| ---------------------------------------------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `testConvertToVoltageMatchesDCMotorPhysics`    | `convertToVoltage` for 10 N on a NEO equals `DCMotor.getVoltage` of the matching rotor torque at zero speed: only the resistive voltage, with no back EMF.                                           |
| `testConvertToCurrentMatchesDCMotorPhysics`    | `convertToCurrent` for 25 N on a Kraken X60 equals `DCMotor.getCurrent` of the matching rotor torque.                                                                                                |
| `testSetpointFeedforwardForceOptionalTracking` | `setVelocity(500 RPM, 5 N)` records a feedforward force of 5 N. `setVelocity(500 RPM)` clears it, and so does `setVelocity(500 RPM, null)`. Runs on each motor controller.                           |
| `testFeedforwardForceProducesMotorOutput`      | With a zero velocity setpoint and zero PID gains, a 0 N force gives a duty cycle of 0, and an 8 N force gives a duty cycle above 0.01 after one second of simulation. Runs on each motor controller. |

## Run it

From the `vendordep` folder:

```bash
./gradlew test --tests "yams.core.motorcontrollers.ForceFeedforwardTest"
```
