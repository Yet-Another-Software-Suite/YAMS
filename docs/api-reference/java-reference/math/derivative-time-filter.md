# DerivativeTimeFilter

**Package:** `yams.core.math`\
**Javadoc:** [`DerivativeTimeFilter`](https://javadocs.yassrobotics.com/yams/yams/core/math/DerivativeTimeFilter.html)

`DerivativeTimeFilter` computes a numerical derivative of a signal while debouncing noise below a minimum rate of change. Useful for estimating velocity from a position signal.

## Constructors

```java
DerivativeTimeFilter(double initial, Time debouncerPeriod)
DerivativeTimeFilter(Time debouncerPeriod)   // Initial value = 0
```

| Parameter         | Description                                                                                               |
| ----------------- | --------------------------------------------------------------------------------------------------------- |
| `debouncerPeriod` | Minimum period over which a derivative is considered valid. Changes faster than this rate are suppressed. |

## Methods

| Method                                | Returns  | Description                                                                                                                |
| ------------------------------------- | -------- | -------------------------------------------------------------------------------------------------------------------------- |
| `derivative(double current, Time dt)` | `double` | Computes the derivative (per microsecond) given explicit elapsed time `dt`. Call it periodically at that `dt`.             |
| `derivative(double current)`          | `double` | Computes the derivative (per microsecond) using the robot clock (`RobotController.getTime()`) elapsed since the last call. |

## Example

```java
DerivativeTimeFilter filter = new DerivativeTimeFilter(Seconds.of(0.02));

// In periodic():
double velocity = filter.derivative(encoder.getPosition());
```
