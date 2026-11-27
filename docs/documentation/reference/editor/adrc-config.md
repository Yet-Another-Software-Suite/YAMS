---
icon: sliders
---

# ADRCConfig

An `ADRCConfig` holds the settings for one motor controller's [ADRC](../../understanding/adrc-controllers/). Pass it to [`withADRCController`](config/closed-loop-control.md#withadrccontroller) on the motor config, which runs it in place of a PID and its feedforward. **ADRC in YAMS was written by FRC Team 8331.** To set one up step by step, see [How do I use ADRC instead of PID?](../../how-to-tuning/how-do-i-use-adrc.md).

{% tabs %}
{% tab title="Java" %}
{% code title="ADRCArmSubsystem.java (fragment)" %}
```java
.withADRCController(new ADRCConfig()
    .withSystemOrder(SystemOrder.ARM)
    .withSettlingTime(Seconds.of(0.3)))
```
{% endcode %}

The class is `yams.core.math.ADRCConfig`.
{% endtab %}

{% tab title="C++" %}
{% code title="ADRCArmSubsystem.cpp (fragment)" %}
```cpp
yams::math::ADRCConfig adrc;
adrc.WithSystemOrder(yams::math::ADRCConfig::SystemOrder::ARM).WithSettlingTime(0.3_s);
m_motorConfig.WithADRCController(adrc);
```
{% endcode %}

The class is `yams::math::ADRCConfig`. Its methods start with a capital letter (`WithSettlingTime`), take WPILib units where a value has one, and the motor config keeps its own copy: change it through `GetADRCController()`, not through the one you passed in.
{% endtab %}
{% endtabs %}

Every setting starts empty, and the controller fills an empty one with its default. A `with` method only accepts a value the controller can use; anything else throws (`IllegalArgumentException` in Java, `std::invalid_argument` in C++) and leaves the config as it was.

## `withSystemOrder`

What kind of mechanism the ADRC drives. Required: there is no default, and building the controller throws without one.

| System order                         | Use it for                           | Loop                                                                     |
| ------------------------------------ | ------------------------------------ | ------------------------------------------------------------------------ |
| `FLYWHEEL`                           | Shooters, flywheels, rollers         | First order, on speed                                                    |
| `ARM`, `PIVOT`, `POSITION`           | Arms, pivots, turrets, wrists, hoods | First or second order, chosen when the robot starts                      |
| `ELEVATOR`                           | Elevators                            | Like `POSITION`, in meters. Needs `withLinearClosedLoopController(true)` |
| `FIRST`, `SECOND`, `THIRD`, `FOURTH` | Anything else                        | That order, with your own `withB0(...)`                                  |

`ARM`, `PIVOT`, `POSITION` and `ELEVATOR` choose first or second order from the motor's back-EMF time constant, which needs the motor config's [moment of inertia](config/simulation.md); [ADRC controllers](../../understanding/adrc-controllers/#two-kinds-of-position-loop) explains why. Without one, `ARM`, `PIVOT` and `POSITION` throw, and `ELEVATOR` stays first order. Once a controller is built, a config refuses an order that would build a different controller: use a new `ADRCConfig`.

## `withSettlingTime`

How quickly the mechanism should reach its setpoint. Default 0.2 s; must be finite and positive.

It sets the feedback bandwidth, `2 × (order + 1) ÷ settling time` radians per second, and the observer runs at four times that. It is the main thing to tune: see [Tune the settling time](../../how-to-tuning/how-do-i-use-adrc.md#tune-the-settling-time). Live tunable.

## `withBandwidthScale`

Scales both bandwidths together. Default 1; must be finite and positive. The settling time does the same job in units you can reason about. Live tunable.

## `withB0`

How strongly a volt moves the mechanism. Default: worked out from the motor, the gearing and the moment of inertia. Must be finite and non-zero. Live tunable.

The worked out `b0` is in the loop's units: acceleration per volt for a second order loop, or speed per volt for a first order position loop. The observer copes with one off by up to a factor of two; lower it if the mechanism overshoots.

## `withInputDelay`

How long the mechanism takes to respond to a new voltage, rounded to whole control periods. Default 0 (off); must be finite and not negative. The observer then compares the measurement with the voltage from that many periods ago. For unusual plants only.

## `withCascaded`

Runs a second observer, fed the first one's estimate of the measurement, and cancels its disturbance estimate instead. It is smoother, but slower to react. Default off. For unusual plants only.

## `withBackEmfModel`

Whether a second order position loop's observer models the motor's back-EMF. Default on.

A motor's back-EMF brakes it in proportion to its speed. Modeled, the observer's disturbance only holds what the model misses, such as gravity and friction, and a mechanism following a motion profile stops on its goal instead of running past it: [ADRC and motion profiles](../../understanding/adrc-controllers/adrc-motion-profiles.md) has the measurements. It applies to an `ARM`, `PIVOT`, `POSITION` or `ELEVATOR` loop that chose second order; a first order position loop already includes the back-EMF in its `b0`, and a `FLYWHEEL` still lumps it. Turn it off if a mechanism slowly oscillates while holding.

## `withTrackingDifferentiator`

Smooths a step setpoint with a [tracking differentiator](../../understanding/adrc-controllers/adrc-tracking-differentiators.md). Off by default; `withoutTrackingDifferentiator()` turns it off again.

```java
.withTrackingDifferentiator(TrackingDifferentiator.Method.LD, 0.08)
```

Leave out the parameters for the method's defaults. They are checked when the controller is built, since some depend on the control period. Building throws a `SmartMotorControllerConfigurationException` when the motor config also has a motion profile, since both shape the setpoint.

## Reading the config

| Method                                 | Returns                                                                                                  |
| -------------------------------------- | -------------------------------------------------------------------------------------------------------- |
| `getSystemOrder()`                     | The system order, if set.                                                                                |
| `getSettlingTime()`, `getInputDelay()` | The time, if set.                                                                                        |
| `getBandwidthScale()`, `getB0()`       | The value, if set.                                                                                       |
| `isCascaded()`                         | Whether the cascaded observer is on.                                                                     |
| `isBackEmfModeled()`                   | Whether the back-EMF may be modeled: true unless turned off.                                             |
| `getModeledBackEmfTimeConstant()`      | The back-EMF time constant the built controller models, or empty when it is lumped.                      |
| `getTdMethod()`, `getTdParams()`       | The tracking differentiator method and parameters, if set.                                               |
| `resolveB0(motor, motorConfig)`        | The `b0` the controller will use: the one set, or the worked out one. Throws if it cannot be worked out. |
| `clone()`                              | A deep copy, settings and gains included. Give each motor controller its own config.                     |

## `ADRCController` on its own

The motor controller builds and runs an `ADRCController` for you. You can also run one yourself, like a WPILib `PIDController`:

```java
ADRCController adrc = new ADRCController(new ADRCConfig()
    .withSystemOrder(SystemOrder.FIRST)
    .withSettlingTime(Seconds.of(0.2)));
// b0, used while the config leaves it unset, and the control period.
adrc.initialize(b0, Milliseconds.of(20));

// Every loop: clamp the output, and tell the observer what was applied last loop.
double volts = MathUtil.clamp(adrc.calculate(measurement, setpoint, appliedVoltage), -12, 12);
```

It does not limit or ramp its output: clamp it, and use a WPILib `SlewRateLimiter` if it needs a ramp. Passing the voltage it really applied keeps the observer from mistaking a clamp for a disturbance.

| Method                                             | What it does                                                                                                                                                                                                                                                                                                           |
| -------------------------------------------------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `initialize(b0, period)`                           | Gives the config the `b0` and control period it needs, works out the gains, and starts at zero. Call it first.                                                                                                                                                                                                         |
| `calculate(measurement, setpoint)`                 | One control step. Returns the output.                                                                                                                                                                                                                                                                                  |
| `calculate(measurement, setpoint, appliedOutput)`  | One step, telling the observer what the actuator applied since the last one.                                                                                                                                                                                                                                           |
| `calculate(measurement, now, next, appliedOutput)` | One step on a moving setpoint, such as a motion profile's: `ADRCController.Reference(value, rate)` for the setpoint now and one period ahead. `Reference.still(value)` is a setpoint standing still. See [ADRC and motion profiles](../../understanding/adrc-controllers/adrc-motion-profiles.md#feeding-the-profile). |
| `reset(measurement)`                               | Starts the observer at a measurement.                                                                                                                                                                                                                                                                                  |
| `getEstimatedDisturbance()`                        | The estimated total disturbance. Divide by `b0` to see it in volts.                                                                                                                                                                                                                                                    |
| `getEstimatedStates()`                             | The observer's estimates: the measurement, its derivatives, then the disturbance.                                                                                                                                                                                                                                      |
| `getBandwidth()`, `getObserverBandwidth()`         | The bandwidths the settling time gives, radians per second.                                                                                                                                                                                                                                                            |
| `getSystemOrder()`                                 | The plant order, including the one a position loop chose.                                                                                                                                                                                                                                                              |

## `TrackingDifferentiator` on its own

```java
TrackingDifferentiator td =
    new TrackingDifferentiator(TrackingDifferentiator.Method.LD, 0.02, Optional.of(new double[] {0.08}));
td.update(setpoint);
double smoothSetpoint = td.getY();
```

| Method                                               | What it does                                                                          |
| ---------------------------------------------------- | ------------------------------------------------------------------------------------- |
| `TrackingDifferentiator(method, sampleTime, params)` | A differentiator, with the sample time in seconds. Empty parameters use the defaults. |
| `update(reference)`                                  | Advances one sample.                                                                  |
| `getY()`, `getYd()`, `getYdd()`                      | The smooth setpoint and its first and second derivatives.                             |
| `reset(value)`                                       | Starts at a value, at rest.                                                           |
| `setParameters(params)`, `setSampleTime(sampleTime)` | Change them; an invalid value throws and keeps the old ones.                          |

## Code reference

{% @github-files/github-code-block url="https://github.com/Yet-Another-Software-Suite/YAMS/blob/master/yams/java/yams/core/math/ADRCConfig.java" %}
