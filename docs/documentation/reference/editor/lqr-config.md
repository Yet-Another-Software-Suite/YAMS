---
icon: sliders
---

# LQRConfig

An `LQRConfig` describes a mechanism and your tolerances for an [LQR](../../understanding/lqr-controllers.md). Build an `LQRController` from it and pass that to `withClosedLoopController(...)` on the motor config, in place of PID gains. To set one up step by step, see [How do I use LQR instead of PID?](../../how-to-tuning/how-do-i-use-lqr/).

{% tabs %}
{% tab title="Java" %}
{% code title="LQRArmSubsystem.java (fragment)" %}
```java
LQRController lqr = new LQRController(new LQRConfig(
        DCMotor.getNEO(1), new MechanismGearing(60), KilogramSquareMeters.of(0.25))
    .withArm(Degrees.of(5), DegreesPerSecond.of(60), Degrees.of(3), DegreesPerSecond.of(30), Degrees.of(0.5))
    .withControlEffort(Volts.of(12))
    .withMaxVoltage(Volts.of(12)));

config.withClosedLoopController(lqr);
```
{% endcode %}

The classes are `yams.core.math.LQRConfig` and `yams.core.math.LQRController`.
{% endtab %}

{% tab title="C++" %}
{% code title="LQRArmSubsystem.cpp (fragment)" %}
```cpp
yams::math::LQRConfig lqrConfig;
lqrConfig.WithArmSystem(wpi::math::DCMotor::NEO(1), 0.25, 60.0)
    .WithQElems({0.0873, 1.047})
    .WithRElems({12.0})
    .WithStateStdDevs({0.0524, 0.524})
    .WithMeasurementStdDevs({0.00873})
    .WithMaxVoltage(12_V);

m_motorConfig.WithLQR(lqrConfig);
```
{% endcode %}

The class is `yams::math::LQRConfig`, and the motor config takes it directly with `WithLQR`. Its values are plain numbers in radians, radians per second, meters and meters per second; the method names differ from Java's, as each section below notes.
{% endtab %}
{% endtabs %}

## The constructor

The motor, the gearing, and the moment of inertia. Required.

`new LQRConfig(DCMotor motor, MechanismGearing gearing, MomentOfInertia moi)`. The model is built from these. An elevator does not use the moment of inertia, since its model comes from the carriage mass and drum radius given to `withElevator`, but the constructor still needs one: pass any, such as `KilogramSquareMeters.of(0.01)`.

In C++ the model comes with the type: `WithFlywheelSystem(motor, moi, reduction)`, `WithArmSystem(motor, moi, reduction)` or `WithElevatorSystem(motor, mass, drumRadius, reduction)`.

## `withArm`

An arm: a two-state model, position and velocity, in radians.

```java
.withArm(
    Radians.of(0.1),          // qelms position: the position error you accept
    RadiansPerSecond.of(1.0), // qelms velocity: the velocity error you accept
    Radians.of(0.01),         // model position trust
    RadiansPerSecond.of(0.1), // model velocity trust
    Radians.of(0.01))         // encoder position trust
```

| Parameter                                  | Unit                   | What it does                                                                                |
| ------------------------------------------ | ---------------------- | ------------------------------------------------------------------------------------------- |
| `qelmsPosition`                            | Angle                  | The position error you accept. Smaller closes position error harder.                        |
| `qelmsVelocity`                            | AngularVelocity        | The velocity error you accept. Smaller damps overshoot.                                     |
| `modelPositionTrust`, `modelVelocityTrust` | Angle, AngularVelocity | How far the Kalman filter trusts the model: its standard deviations. Larger trusts it less. |
| `encoderPositionTrust`                     | Angle                  | How noisy the encoder is: its standard deviation. Larger trusts it less.                    |

The model has no gravity: give the motor config an `ArmFeedforward` with a kG. In C++: `WithArmSystem`, then `WithQElems({position, velocity})`, `WithStateStdDevs({position, velocity})` and `WithMeasurementStdDevs({position})`.

## `withElevator`

An elevator: a two-state model, position and velocity, in meters.

```java
.withElevator(
    Meters.of(0.02),         // qelms position
    MetersPerSecond.of(0.5), // qelms velocity
    Meters.of(0.005),        // model position trust
    MetersPerSecond.of(0.1), // model velocity trust
    Meters.of(0.001),        // encoder position trust
    Kilograms.of(5),         // carriage mass
    Inches.of(1))            // drum radius
```

The tolerances and trusts mean the same as an arm's, in meters. `mass` is the carriage and its load; `drumRadius` the radius of the spool or sprocket the rope, belt or chain wraps around. The model has no gravity: give the motor config an `ElevatorFeedforward` with a kG. In C++: `WithElevatorSystem(motor, mass, drumRadius, reduction)`.

## `withFlyWheel`

A flywheel: a one-state model, velocity, in radians per second.

```java
.withFlyWheel(
    RadiansPerSecond.of(1.0),  // qelms: the speed error you accept
    RadiansPerSecond.of(0.01), // model trust
    RadiansPerSecond.of(0.1))  // encoder trust
```

Smaller `qelms` reaches speed faster. In C++: `WithFlywheelSystem(motor, moi, reduction)`, with one value each in `WithQElems`, `WithStateStdDevs` and `WithMeasurementStdDevs`.

## `withControlEffort`

The voltage the LQR treats as acceptable, its "relms". Default 12 V. Lower it for a gentler response that uses less voltage; raise it, up to 12 V, for a faster one. `withRelms(...)` is the same setting. In C++: `WithRElems({12.0})`.

## `withMaxVoltage`

The most voltage the LQR outputs. Default 12 V. Lower it to cap the output, for example to 10 V for headroom. In C++: `WithMaxVoltage(12_V)`.

## The loop period

The LQR's model is worked out for a 20 ms period, the default closed loop control period. Java has no setter, so keep the default period with an LQR: `withClosedLoopControlPeriod` changes how often the loop runs but not the period the LQR was built for, and the Talon wrappers refuse it. In C++, `WithPeriod(...)` sets it.

## `withAggressiveness` and `withMeasurementDelay`

Stored, but not used yet: they change nothing. Use the qelms and the control effort to change how aggressive the LQR is.

## Starting values

Every value is a starting point. Change one at a time, as in [Tune it live](../../how-to-tuning/how-do-i-use-lqr/#tune-it-live).

| Parameter                      | Start at                                                                         | Change it when                                                                   |
| ------------------------------ | -------------------------------------------------------------------------------- | -------------------------------------------------------------------------------- |
| Position qelms (arm, elevator) | `Radians.of(0.1)` (about 6°), `Meters.of(0.05)`                                  | Lower it if the mechanism is slow to reach the setpoint.                         |
| Velocity qelms                 | `RadiansPerSecond.of(1.0)`, `MetersPerSecond.of(0.5)`                            | Lower it if an arm or elevator overshoots, or a flywheel is slow to reach speed. |
| Model trust                    | `Radians.of(0.01)`, `RadiansPerSecond.of(0.1)`, or the same in meters            | Raise it if the estimate lags behind the real mechanism.                         |
| Encoder trust                  | `Radians.of(0.01)`, `RadiansPerSecond.of(0.1)` on a flywheel, `Meters.of(0.001)` | Raise it if the output is noisy.                                                 |
| `withControlEffort`            | `Volts.of(12)`                                                                   | Lower it for a gentler response.                                                 |
| `withMaxVoltage`               | `Volts.of(12)`                                                                   | Lower it to cap the output.                                                      |

The qelms and trusts have no defaults: `withFlyWheel`, `withArm` or `withElevator` sets them.

**The trust values** are standard deviations, and only their ratio matters. Measure the encoder trust as the encoder's noise, and set the model trust against it: [Find the encoder and model trust](../../how-to-tuning/how-do-i-use-lqr/#find-the-encoder-and-model-trust) shows how.

## Simulation only

`withSimClosedLoopController(lqrController)` on the motor config uses the LQR only in simulation, with the real robot keeping its PID. Use it to try LQR, or to compare it with the PID, before deploying.

```java
config.withClosedLoopController(5.0, 0, 0.5)       // the robot runs this PID
    .withSimClosedLoopController(simLqrController); // simulation runs the LQR
```

## Live tuning keys

With `HIGH` telemetry, the LQR's values are tunable under `NT:/Tuning/<Mechanism>/<Motor>/closedloop/lqr/`, in the same units as the tunable setpoints. A field the LQR's type does not use is not published.

| Field              | `LQRConfig` value      | Flywheel | Arm     | Elevator |
| ------------------ | ---------------------- | -------- | ------- | -------- |
| `qelms/position`   | Position qelms         |          | Degrees | Meters   |
| `qelms/velocity`   | Velocity qelms         | RPM      | RPM     | m/s      |
| `relms`            | Control effort         | Volts    | Volts   | Volts    |
| `model/position`   | Model position trust   |          | Degrees | Meters   |
| `model/velocity`   | Model velocity trust   | RPM      | RPM     | m/s      |
| `encoder/position` | Encoder position trust |          | Degrees | Meters   |
| `encoder/velocity` | Encoder trust          | RPM      |         |          |
| `maxVoltage`       | Max voltage            | Volts    | Volts   | Volts    |

A change rebuilds the LQR from every field, starting from the old one's estimate, so the output does not jump. Every value must be above 0; one the LQR cannot use leaves it as it was, with a warning on the Driver Station naming the motor.

## Things to know

* **It always runs on the SystemCore,** on every motor controller, in a `Notifier` the wrapper starts in closed loop mode, with or without a mechanism class.
* **No closed loop tolerance:** `withClosedLoopTolerance` with an `LQRController` throws an `IllegalArgumentException`.
* **The feedforward still applies:** kS, kV, kG and kA are added to the LQR's output.
* **Motion profiles still apply:** with a trapezoidal or exponential profile, the LQR tracks the profile's position and velocity.

## `LQRController`

| Method                                    | What it does                                                                   |
| ----------------------------------------- | ------------------------------------------------------------------------------ |
| `LQRController(LQRConfig)`                | Builds the model, the regulator and the Kalman filter.                         |
| `calculate(measured, position, velocity)` | One step for an arm (angles) or elevator (distances): the voltage to apply.    |
| `calculate(measured, velocity)`           | One step for a flywheel.                                                       |
| `reset(position, velocity)`               | Starts the Kalman filter at a state.                                           |
| `retune(LQRConfig)`                       | Rebuilds the LQR from a new config, keeping its estimate. Live tuning uses it. |
| `getType()`, `getConfig()`                | The LQR's type, and its config.                                                |

## Code reference

{% @github-files/github-code-block url="https://github.com/Yet-Another-Software-Suite/YAMS/blob/master/yams/java/yams/core/math/LQRConfig.java" %}
