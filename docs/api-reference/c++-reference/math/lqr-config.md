# LQRConfig

**Namespace:** `yams::math`\
**Header:** `#include <yams/math/LQRConfig.hpp>`\
**Java equivalent:** [Java LQRConfig](../../java-reference/math/lqr-config.md)\
**Doxygen:** [`LQRConfig`](https://doxygen.yassrobotics.com/yams/classyams_1_1math_1_1LQRConfig.html)

`LQRConfig` is a builder for `LinearSystemLoop` objects consumed by `LQRController`.

***

## Enum: `LQRType`

Defined inside `LQRConfig`.

| Value      | Description                                |
| ---------- | ------------------------------------------ |
| `FLYWHEEL` | Single-state system (angular velocity)     |
| `ARM`      | Two-state system (angle, angular velocity) |
| `ELEVATOR` | Two-state system (position, velocity)      |

***

## Type aliases

```cpp
using Loop1 = wpi::math::LinearSystemLoop<1, 1, 1>;  // Used for FLYWHEEL
using Loop2 = wpi::math::LinearSystemLoop<2, 1, 1>;  // Used for ARM and ELEVATOR
```

***

## `LoopBundle<States>`

A `LinearSystemLoop` together with everything it points to. WPILib's `LinearSystemLoop` keeps raw pointers to its LQR and Kalman filter, and `KalmanFilter` keeps a raw pointer to its plant, so `GetLoop()` builds all four together on the heap. A `LoopBundle` can be neither copied nor moved, so those pointers never dangle.

```cpp
template <int States>  // 1 for a flywheel, 2 for an arm or elevator
struct LoopBundle {
  wpi::math::LinearSystem<States, 1, 1> plant;
  wpi::math::LinearQuadraticRegulator<States, 1> controller;
  wpi::math::KalmanFilter<States, 1, 1> observer;
  wpi::math::LinearSystemLoop<States, 1, 1> loop;  // The loop to run
};
```

***

## Builder methods

All builder methods return `LQRConfig&` for chaining.

| Method                                                                                                                     | Description                                        |
| -------------------------------------------------------------------------------------------------------------------------- | -------------------------------------------------- |
| `WithType(LQRType type)`                                                                                                   | Set the mechanism type                             |
| `WithFlywheelSystem(const wpi::math::DCMotor& motor, double momentOfInertia, double gearing)`                              | Configure the flywheel linear system               |
| `WithArmSystem(const wpi::math::DCMotor& motor, double momentOfInertia, double gearing)`                                   | Configure the arm linear system                    |
| `WithElevatorSystem(const wpi::math::DCMotor& motor, double mass, double drumRadius, double gearing)`                      | Configure the elevator linear system               |
| `WithQElems(std::initializer_list<double> qElems)`                                                                         | State cost weights (Q matrix diagonal)             |
| `WithRElems(std::initializer_list<double> rElems)`                                                                         | Control effort cost weights (R matrix diagonal)    |
| `WithStateStdDevs(std::initializer_list<double> stateStdDevs)`                                                             | Model noise standard deviations (Kalman Q)         |
| `WithMeasurementStdDevs(std::initializer_list<double> measStdDevs)`                                                        | Sensor noise standard deviations (Kalman R)        |
| `WithQElems(const std::vector<double>& qElems)`, `WithRElems(...)`, `WithStateStdDevs(...)`, `WithMeasurementStdDevs(...)` | The same, from a `std::vector`, checked: see below |
| `WithMaxVoltage(wpi::units::volt_t maxVoltage)`                                                                            | Saturate output at this voltage                    |
| `WithPeriod(wpi::units::second_t period)`                                                                                  | Control loop period (default: 20 ms)               |

The `std::vector` overloads are the ones live tuning uses. They check the values: each throws `std::invalid_argument` if the number of values does not match the mechanism type (one per modeled state for `WithQElems` and `WithStateStdDevs`, one for `WithRElems` and `WithMeasurementStdDevs`) or a value is not above 0. Set the type and system first.

***

## Getters

```cpp
LQRType GetType() const;
wpi::units::second_t GetPeriod() const;
wpi::units::volt_t GetMaxVoltage() const;
std::vector<double> GetQElems() const;
std::vector<double> GetRElems() const;
std::vector<double> GetStateStdDevs() const;
std::vector<double> GetMeasurementStdDevs() const;
std::variant<wpi::math::LinearSystem<1, 1, 1>, wpi::math::LinearSystem<2, 1, 1>> GetSystem() const;
std::variant<std::unique_ptr<LoopBundle<1>>, std::unique_ptr<LoopBundle<2>>> GetLoop() const;
```

`GetQElems()`, `GetStateStdDevs()` and `GetMeasurementStdDevs()` are in SI units: rad/s for a flywheel, rad and rad/s for an arm, m and m/s for an elevator. `GetRElems()` is in volts. A value you never set is the default the loop is built with.

{% hint style="info" %}
`GetSystem()` and `GetLoop()` return `std::variant`. Access the correct alternative using `std::get`, and use the bundle's `loop`:

```cpp
auto flywheel = std::get<std::unique_ptr<LQRConfig::LoopBundle<1>>>(cfg.GetLoop());  // FLYWHEEL
auto arm = std::get<std::unique_ptr<LQRConfig::LoopBundle<2>>>(cfg.GetLoop());       // ARM or ELEVATOR
arm->loop.Reset(...);  // run the bundle's loop
```

Keep the bundle alive as long as you use its `loop`, which points at the bundle's LQR and Kalman filter.
{% endhint %}

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
   .WithMaxVoltage(12_V)
   .WithPeriod(20_ms);
```
