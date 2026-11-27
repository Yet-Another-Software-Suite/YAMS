# LQRConfig

**Package:** `yams.core.math`\
**Javadoc:** [`LQRConfig`](https://javadocs.yassrobotics.com/yams/yams/core/math/LQRConfig.html)

`LQRConfig` is a builder for `LinearSystemLoop` objects used by `LQRController`. It encapsulates the linear system model, Kalman filter, and LQR regulator into a single configurable object.

## Enum: `LQRType`

| Value      | Description                                |
| ---------- | ------------------------------------------ |
| `FLYWHEEL` | Single-state system (angular velocity)     |
| `ARM`      | Two-state system (angle, angular velocity) |
| `ELEVATOR` | Two-state system (position, velocity)      |

## Constructor

```java
LQRConfig(DCMotor motor, MechanismGearing gearing, MomentOfInertia moi)
```

All builder methods return the `LQRConfig` object for chaining.

## Builder methods

| Method                                                                                                                                                                                              | Parameters                 | Description                                                                      |
| --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | -------------------------- | -------------------------------------------------------------------------------- |
| `withRelms(Voltage relms)`                                                                                                                                                                          | `relms`                    | Control effort cost. Higher values penalize voltage usage more.                  |
| `withControlEffort(Voltage effort)`                                                                                                                                                                 | `effort`                   | Alias for `withRelms`.                                                           |
| `withMaxVoltage(Voltage voltage)`                                                                                                                                                                   | `voltage`                  | Saturates the LQR output at this voltage.                                        |
| `withMeasurementDelay(Time delay)`                                                                                                                                                                  | `delay`                    | Compensates for sensor latency.                                                  |
| `withAggressiveness(double aggressiveness)`                                                                                                                                                         | `aggressiveness`: 1 to 100 | Higher values make the controller more responsive (and potentially less stable). |
| `withFlyWheel(AngularVelocity qelms, AngularVelocity modelTrust, AngularVelocity encoderTrust)`                                                                                                     | N/A                        | Configures state cost, model noise, and encoder noise for flywheel systems.      |
| `withElevator(Distance qelmsPosition, LinearVelocity qelmsVelocity, Distance modelPositionTrust, LinearVelocity modelVelocityTrust, Distance encoderPositionTrust, Mass mass, Distance drumRadius)` | N/A                        | Full elevator configuration with state costs and noise parameters.               |
| `withArm(Angle qelmsPosition, AngularVelocity qelmsVelocity, Angle modelPositionTrust, AngularVelocity modelVelocityTrust, Angle encoderPositionTrust)`                                             | N/A                        | Arm configuration with state costs and noise parameters.                         |

### Setting the values on their own

These replace the values `withFlyWheel`, `withArm` or `withElevator` set, so call them after it. They take SI units, as the getters below return them: rad/s for a flywheel, rad and rad/s for an arm, m and m/s for an elevator. Live tuning uses them to change one LQR value at a time.

| Method                                  | Parameters                                                 | Description                                                                                 |
| --------------------------------------- | ---------------------------------------------------------- | ------------------------------------------------------------------------------------------- |
| `withQelms(double... qelms)`            | One per modeled state: velocity, or position then velocity | State error tolerances.                                                                     |
| `withModelStdDevs(double... stdDevs)`   | One per modeled state                                      | The model's standard deviations.                                                            |
| `withEncoderStdDevs(double... stdDevs)` | One                                                        | The encoder's standard deviation: velocity for a flywheel, position for an arm or elevator. |

Each throws an `IllegalArgumentException` if the number of values does not match the mechanism type or a value is not above 0, and a `NoSuchElementException` if no mechanism type has been configured yet.

{% hint style="info" %}
The qelms parameters ("Q elements") define how much you care about state error. Smaller qelms values mean the controller tolerates more error but uses less voltage. The rElms ("R elements") penalize control effort: larger values produce gentler outputs.
{% endhint %}

## Getters

| Method                                                          | Returns                             | Description                                                       |
| --------------------------------------------------------------- | ----------------------------------- | ----------------------------------------------------------------- |
| `getType()`                                                     | `LQRType`                           | The configured mechanism type.                                    |
| `getPeriod()`                                                   | `Time`                              | The control loop period.                                          |
| `getQelms()`                                                    | `double[]`                          | A copy of the state error tolerances, in SI units.                |
| `getModelStdDevs()`                                             | `double[]`                          | A copy of the model's standard deviations, in SI units.           |
| `getEncoderStdDevs()`                                           | `double[]`                          | A copy of the encoder's standard deviation, in SI units.          |
| `getRelms()`                                                    | `Voltage`                           | The control effort tolerance.                                     |
| `getMaxVoltage()`                                               | `Voltage`                           | The most voltage the loop outputs.                                |
| `getSystem()`                                                   | `LinearSystem<?, ?, ?>`             | The computed linear system model.                                 |
| `getKalmanFilter(LinearSystem<?, ?, ?> plant)`                  | `KalmanFilter<?, ?, ?>`             | Constructs and returns the Kalman observer.                       |
| `getRegulator(LinearSystem<?, ?, ?> plant)`                     | `LinearQuadraticRegulator<?, ?, ?>` | Constructs and returns the LQR regulator.                         |
| `getLoop()`                                                     | `LinearSystemLoop<?, ?, ?>`         | Constructs and returns the full system loop (convenience method). |
| `getLoop(LinearSystem, LinearQuadraticRegulator, KalmanFilter)` | `LinearSystemLoop<?, ?, ?>`         | Constructs the loop from individually-provided components.        |

## See also

* [LQRController](lqr-controller.md)
* [C++ LQRConfig](../../c++-reference/math/lqr-config.md)
