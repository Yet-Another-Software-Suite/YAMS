# LQRController

**Namespace:** `yams::math`\
**Header:** `#include <yams/math/LQRController.hpp>`\
**Java equivalent:** [Java LQRController](../../java-reference/math/lqr-controller.md)\
**Doxygen:** [`LQRController`](https://doxygen.yassrobotics.com/yams/classyams_1_1math_1_1LQRController.html)

`LQRController` wraps a `LinearSystemLoop` and exposes a unified `Calculate` interface for arm, elevator, and flywheel mechanisms.

***

## Constructor

```cpp
explicit LQRController(LQRConfig config)
```

***

## Methods

### Configuration

| Method         | Signature                                           | Description                                                                                                                                                                                                                                                 |
| -------------- | --------------------------------------------------- | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `UpdateConfig` | `void UpdateConfig(LQRConfig config)`               | Replace the internal system loop at runtime                                                                                                                                                                                                                 |
| `Retune`       | `void Retune(LQRConfig config)`                     | Rebuild the loop from a config whose values changed, keeping the state estimate, so the output does not jump. Safe to call while the closed loop controller's thread runs `Calculate`. Throws `std::invalid_argument` for a config of a different `LQRType` |
| `GetType`      | `LQRConfig::LQRType GetType() const`                | Returns the configured mechanism type                                                                                                                                                                                                                       |
| `GetConfig`    | `const std::optional<LQRConfig>& GetConfig() const` | Returns the config if constructed from `LQRConfig`, otherwise empty                                                                                                                                                                                         |

### Reset

| Method  | Signature                                                                            | Description                                         |
| ------- | ------------------------------------------------------------------------------------ | --------------------------------------------------- |
| `Reset` | `void Reset(wpi::units::radian_t angle, wpi::units::radians_per_second_t velocity)`  | Reset the Kalman observer state for arm or flywheel |
| `Reset` | `void Reset(wpi::units::meter_t distance, wpi::units::meters_per_second_t velocity)` | Reset the Kalman observer state for elevator        |

### Calculate

| Overload           | Signature                                                                                                                               | Description                                                  |
| ------------------ | --------------------------------------------------------------------------------------------------------------------------------------- | ------------------------------------------------------------ |
| Arm                | `wpi::units::volt_t Calculate(wpi::units::radian_t measured, wpi::units::radian_t position, wpi::units::radians_per_second_t velocity)` | Compute output voltage for arm control                       |
| Elevator           | `wpi::units::volt_t Calculate(wpi::units::meter_t measured, wpi::units::meter_t position, wpi::units::meters_per_second_t velocity)`    | Compute output voltage for elevator control                  |
| Flywheel (angular) | `wpi::units::volt_t Calculate(wpi::units::radians_per_second_t measured, wpi::units::radians_per_second_t velocity)`                    | Compute output voltage for flywheel angular velocity control |
| Flywheel (linear)  | `wpi::units::volt_t Calculate(wpi::units::meters_per_second_t measured, wpi::units::meters_per_second_t velocity)`                      | Compute output voltage for flywheel surface speed control    |

***

## Example

```cpp
yams::math::LQRConfig cfg;
cfg.WithType(yams::math::LQRConfig::LQRType::ARM)
   .WithArmSystem(wpi::math::DCMotor::Falcon500(1), 0.5, 50.0)
   .WithQElems({0.01745, 0.08727})
   .WithRElems({12.0})
   .WithStateStdDevs({0.015, 0.17})
   .WithMeasurementStdDevs({0.01})
   .WithMaxVoltage(12_V);

yams::math::LQRController controller{cfg};

// In Periodic():
wpi::units::radian_t measured = GetArmAngle();
wpi::units::volt_t output = controller.Calculate(
    measured,
    targetAngle,
    targetVelocity
);
motor.SetVoltage(output);
```

{% hint style="info" %}
Call `Reset` whenever the mechanism is re-enabled or its state is known (e.g., at initialization) to prevent the Kalman filter from tracking stale state.
{% endhint %}

***

## Live tuning

A [SmartMotorController](../motor-controllers/smart-motor-controller.md) with an LQR publishes its values at `TelemetryVerbosity::HIGH` in place of kP, kI and kD, see [LQR fields](../motor-controllers/smart-motor-controller-telemetry-config.md#lqr-fields). While the `Live Tuning` command runs, a changed value is set on a copy of `GetConfig()` and passed to `Retune`. `SmartMotorController::GetLQRController()` returns the controller, or `nullptr` without one.
