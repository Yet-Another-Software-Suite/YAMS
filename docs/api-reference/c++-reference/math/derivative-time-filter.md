# DerivativeTimeFilter

**Namespace:** `yams::math`\
**Header:** `#include <yams/math/DerivativeTimeFilter.hpp>`\
**Java equivalent:** [Java DerivativeTimeFilter](../../java-reference/math/derivative-time-filter.md)\
**Doxygen:** [`DerivativeTimeFilter`](https://doxygen.yassrobotics.com/yams/classyams_1_1math_1_1DerivativeTimeFilter.html)

`DerivativeTimeFilter` computes a numerical derivative of a signal and applies a debounce filter to suppress transient spikes.

***

## Constructors

```cpp
DerivativeTimeFilter(double initial, wpi::units::second_t debouncerPeriod)
explicit DerivativeTimeFilter(wpi::units::second_t debouncerPeriod)  // initial = 0.0
```

| Parameter         | Description                                                           |
| ----------------- | --------------------------------------------------------------------- |
| `initial`         | Initial value of the signal. Defaults to `0.0` if omitted.            |
| `debouncerPeriod` | Duration the debounce filter holds a spike before passing it through. |

***

## Methods

```cpp
double Derivative(double current, wpi::units::second_t dt)
double Derivative(double current)
```

| Overload                  | Description                                                                                                                                                                                                        |
| ------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| `Derivative(current, dt)` | Computes derivative using an explicit time step `dt`.                                                                                                                                                              |
| `Derivative(current)`     | Computes derivative using the robot timestamp (`wpi::RobotController::GetTime()`); `dt` is measured since the last update. Only recomputes once the debounce period has elapsed, otherwise returns the last value. |

Returns the filtered derivative value.

***

## Example

```cpp
yams::math::DerivativeTimeFilter filter{20_ms};

// In periodic (called every 20 ms):
double velocity = filter.Derivative(encoder.GetPosition());
```

{% hint style="info" %}
Use the explicit `dt` overload in simulation or when calling at non-uniform intervals. Use the single-argument overload in robot periodic methods where the robot timestamp is authoritative.
{% endhint %}
