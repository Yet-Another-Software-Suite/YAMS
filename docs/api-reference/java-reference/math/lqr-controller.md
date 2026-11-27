# LQRController

**Package:** `yams.core.math`\
**Javadoc:** [`LQRController`](https://javadocs.yassrobotics.com/yams/yams/core/math/LQRController.html)

`LQRController` wraps a `LinearSystemLoop` and provides type-safe `calculate()` overloads for different mechanism types.

## Constructors

```java
LQRController(LQRConfig config)   // Preferred
LQRController(LQRType type, LinearSystemLoop<?, ?, ?> loop, Time period)
```

## Methods

| Method                                              | Returns | Description                                                                                                                                                                                                                           |
| --------------------------------------------------- | ------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `reset(Angle angle, AngularVelocity velocity)`      | `void`  | Resets the observer for arm or flywheel systems. Call on enable.                                                                                                                                                                      |
| `reset(Distance distance, LinearVelocity velocity)` | `void`  | Resets the observer for elevator systems.                                                                                                                                                                                             |
| `updateConfig(LQRConfig config)`                    | `void`  | Replaces the underlying system loop at runtime.                                                                                                                                                                                       |
| `retune(LQRConfig config)`                          | `void`  | Rebuilds the loop from a config whose values changed, keeping the state estimate, so the output does not jump. Safe to call while the closed loop runs. Throws `IllegalArgumentException` if the config is for a different `LQRType`. |

## Calculate overloads

| Method                                                                     | For Mechanism                   | Returns   |
| -------------------------------------------------------------------------- | ------------------------------- | --------- |
| `calculate(Angle measured, Angle position, AngularVelocity velocity)`      | Arm                             | `Voltage` |
| `calculate(Distance measured, Distance position, LinearVelocity velocity)` | Elevator                        | `Voltage` |
| `calculate(AngularVelocity measured, AngularVelocity velocity)`            | Flywheel                        | `Voltage` |
| `calculate(LinearVelocity measured, LinearVelocity velocity)`              | Flywheel (linear surface speed) | `Voltage` |

## Getters

| Method        | Returns               | Description                                                                                     |
| ------------- | --------------------- | ----------------------------------------------------------------------------------------------- |
| `getType()`   | `LQRType`             | The mechanism type this controller was configured for.                                          |
| `getConfig()` | `Optional<LQRConfig>` | The `LQRConfig` used to construct this controller, or empty if constructed from raw components. |

## Usage with SmartMotorControllerConfig

Pass the controller to `SmartMotorControllerConfig.withClosedLoopController(LQRController)` (this clears any configured PID). `withSimClosedLoopController(LQRController)` sets one for simulation only.

```java
LQRConfig lqrCfg = new LQRConfig(DCMotor.getNEO(1), gearing, moi)
    .withArm(Degrees.of(5), DegreesPerSecond.of(10),
             Degrees.of(1), DegreesPerSecond.of(5), Degrees.of(0.5))
    .withControlEffort(Volts.of(12));

SmartMotorControllerConfig motorConfig = new SmartMotorControllerConfig(this)
    .withClosedLoopController(new LQRController(lqrCfg));
```

## Live tuning

A [SmartMotorController](../motor-controllers/smart-motor-controller.md) with an LQR publishes its values at `TelemetryVerbosity.HIGH` in place of kP, kI and kD, see [LQR fields](../motor-controllers/smart-motor-controller-telemetry-config.md#lqr-fields). While the `Live Tuning` command runs, a changed value is set on the controller's `LQRConfig` and passed to `retune`.

## See also

* [LQRConfig](lqr-config.md)
* [C++ LQRController](../../c++-reference/math/lqr-controller.md)
