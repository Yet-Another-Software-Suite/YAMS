# ADRC Guide

ADRC (Active Disturbance Rejection Control) is a closed loop controller that replaces a PID and
its feedforward. **ADRC in YAMS was written by FRC Team 8331**, who contributed it from their fork
of YAMS.

An ADRC treats everything it is not told about (gravity, friction, back-EMF, a game piece in the
mechanism, a bumped arm) as one "total disturbance". An extended state observer (ESO) estimates
that disturbance every loop, and the control law cancels it. So an ADRC needs very little model:
the plant's order and an approximate input gain `b0`. There is no kS, kV, kA or kG to tune.

ADRC is Java only for now. The C++ port will follow.

## Quick start

```java
SmartMotorControllerConfig config = new SmartMotorControllerConfig(this)
    .withControlMode(ControlMode.CLOSED_LOOP)
    .withGearing(new MechanismGearing(GearBox.fromReductionStages(3, 4)))
    .withMomentOfInertia(Inches.of(18), Pounds.of(8)) // b0 is derived from this
    .withADRCController(new ADRCConfig("Arm")
        .withSystemOrder(2)       // a position loop
        .withSettlingTime(0.6)    // seconds
        .withMaxVoltage(12));
```

`withADRCController` takes the place of `withClosedLoopController(kP, kI, kD)` and
`withFeedforward(...)`. The mechanism itself needs no changes: an `Arm`, `Elevator`, `Pivot` or
`FlyWheel` keeps calling `setPosition(...)` and `setVelocity(...)` as it does with a PID, and the
motor controller runs the ADRC.

## How it runs

- **On the SystemCore, for every motor controller.** ADRC needs the measured state every loop,
  so it runs in YAMS's closed loop thread, like an LQR, on the SPARK MAX, SPARK Flex, TalonFX,
  TalonFXS, Nova and A301. The motor controller is sent a plain voltage.
- **At the closed loop period.** The ADRC's discrete model uses the period from
  `withClosedLoopControlPeriod(...)`, 20 ms unless you set it.
- **In mechanism units.** The ADRC works in mechanism rotations and rotations per second.
  It works in meters and meters per second only when the closed loop is linear: with
  `withLinearClosedLoopController(true)` or a linear motion profile, and a mechanism
  circumference.
- **With the applied voltage.** Each loop, the observer is told the voltage the motor
  controller actually applied, not the one it asked for. So a current limit, a low battery or
  saturation is not mistaken for a disturbance.
- **From where the mechanism is.** When the closed loop starts, and every loop while the robot
  is disabled, the observer is reset to the measurement: the position, or the velocity while a
  velocity setpoint is set. Enabling the robot does not look like a step from zero.

## Choosing the settings

### System order

The order of the plant from the voltage to what you control.

| Mechanism | Setpoint | Order | `b0` |
| --- | --- | --- | --- |
| Flywheel, roller | Velocity | 1 | Derived, or acceleration per volt |
| Arm, pivot, turret | Position | 2 | Derived, or acceleration per volt |
| Elevator, or any mechanism whose speed is set by its back-EMF | Position | 1 | Speed per volt: `Kv / (2 pi G)` |

There is no default system order. `buildController` throws until one is set.

### `b0`, the input gain

`b0` is how much one volt moves the plant's highest derivative: mechanism rotations per second
squared per volt for an order 1 velocity loop or an order 2 position loop.

If `b0` is not set, it is derived from the motor, the gearing and the mechanism's moment of
inertia:

```
b0 = G * Kt / (2 * pi * R * J)    rotations per second squared per volt
```

where `G` is motor rotations per mechanism rotation, `Kt` and `R` come from the `DCMotor`, and `J`
is the moment of inertia. With a linear closed loop it is multiplied by the mechanism
circumference, for meters per second squared per volt.

The derivation only uses an inertia you set with `withMomentOfInertia(...)` or
`withWheelMomentOfInertia(...)`. YAMS's placeholder inertia is never used: without one, and
without `withB0(...)`, building the controller throws.

The derived `b0` ignores back-EMF, which the observer has to estimate as part of the disturbance.
That suits flywheels and arms. An elevator's back-EMF, not its inertia, sets how fast it moves,
so for an elevator use order 1 on its position, with its speed per volt as `b0`:

```java
DCMotor motor = DCMotor.getNEO(1);
double gearing = 12; // motor rotations per drum rotation
config.withADRCController(new ADRCConfig("Elevator")
    .withSystemOrder(1)
    .withSettlingTime(0.5)
    .withB0(motor.Kv / (2 * Math.PI * gearing))); // drum rotations per second per volt
```

`b0` does not have to be exact: the observer absorbs a `b0` off by a factor of two. If the
mechanism overshoots or oscillates, lower `b0`; if it is sluggish, raise it.

### Settling time and bandwidth scale

The settling time is how quickly you want the mechanism to reach its setpoint. It sets the
controller bandwidth `wc` and the observer bandwidth `wo`:

```
wc = 2 * (order + 1) * bandwidthScale / settlingTime    radians per second
wo = 4 * wc
```

A shorter settling time is more aggressive. The observer has to be faster than the dynamics it
estimates, including the motor's back-EMF. For a flywheel's velocity loop that means a short
settling time: a Kraken X60 flywheel crawls to its setpoint at 0.3 s and holds it well at 0.15 s.

The bandwidth scale multiplies both bandwidths without changing the settling time you think in.

### Output limits

The output is clamped to `withMaxVoltage(max)` (from `-max` to `max`) or
`withVoltageLimits(min, max)`. Without either, the motor config's
`withClosedLoopControllerMaximumVoltage(...)` is used, and without that the motor's nominal
voltage, with a warning.

## Examples

These are the settings YAMS's tests use, which hold each mechanism at its setpoint on every motor
controller.

```java
// Flywheel: order 1 on velocity, b0 derived from the wheel's inertia.
config.withMomentOfInertia(KilogramSquareMeters.of(0.01))
      .withADRCController(new ADRCConfig("Shooter").withSystemOrder(1).withSettlingTime(0.15).withMaxVoltage(12));
flyWheel.setMechanismVelocitySetpoint(RPM.of(3000));

// Arm under gravity, with no kG: order 2 on position.
config.withMomentOfInertia(KilogramSquareMeters.of(0.1))
      .withADRCController(new ADRCConfig("Arm").withSystemOrder(2).withSettlingTime(0.6).withMaxVoltage(12));
arm.setMechanismPositionSetpoint(Degrees.of(45));

// Pivot or turret: order 2 on position.
config.withMomentOfInertia(KilogramSquareMeters.of(0.1))
      .withADRCController(new ADRCConfig("Turret").withSystemOrder(2).withSettlingTime(0.6).withMaxVoltage(12));

// Elevator under gravity, with no kG: order 1 on position, b0 is its speed per volt.
config.withMechanismCircumference(Meters.of(2 * Math.PI * 0.02))
      .withADRCController(new ADRCConfig("Elevator").withSystemOrder(1).withSettlingTime(0.5)
          .withB0(motor.Kv / (2 * Math.PI * gearing)).withMaxVoltage(12));
elevator.setMeasurementPositionSetpoint(Meters.of(0.5));
```

## Telemetry and live tuning

With an ADRC, a motor controller's telemetry hides kP, kI and kD and publishes these under
`closedloop/adrc/`:

| Topic | Tunable | What it is |
| --- | --- | --- |
| `disturbance` | No | The estimated total disturbance, in the plant's highest derivative. |
| `bandwidth` | No | Controller bandwidth `wc`, radians per second. |
| `observerBandwidth` | No | Observer bandwidth `wo`, radians per second. |
| `systemOrder` | No | Plant order. |
| `output` | No | The last output, volts. |
| `b0` | Yes | Input gain. |
| `settlingTime` | Yes | Settling time, seconds. |
| `bandwidthScale` | Yes | Bandwidth scale. |

The tunable values are applied while the Live Tuning command runs. Retuning keeps the estimated
disturbance, so the output does not jump. A value the ADRC cannot use, such as a `b0` of zero,
is reported as a warning and leaves the controller as it was.

## API reference

### `SmartMotorControllerConfig`

| Method | What it does |
| --- | --- |
| `withADRCController(ADRCConfig)` | Use an ADRC as the closed loop controller in slot 0. |
| `withADRCController(ADRCConfig, ClosedLoopControllerSlot)` | Use an ADRC in a slot. `null` removes the slot's ADRC. |
| `getADRCController()` | The ADRC config of slot 0, if any. |
| `getADRCController(ClosedLoopControllerSlot)` | The ADRC config of a slot, if any. |
| `isMomentOfInertiaConfigured()` | Whether `withMomentOfInertia(...)` or `withWheelMomentOfInertia(...)` was called, so `b0` can be derived. |

`withADRCController` removes the slot's PID (and sim PID), the LQR, and every feedforward of the
slot (simple, arm and elevator, and their sim versions): the observer estimates what a
feedforward models, so keeping both would count the plant twice. While a slot has an ADRC,
`getSimpleFeedforward`, `getArmFeedforward` and `getElevatorFeedforward` return empty for it.
Setting a PID (`withClosedLoopController(kP, kI, kD)`) or an LQR afterwards removes the ADRC.

With an ADRC, the SPARKs and Talons accept `withClosedLoopControlPeriod(...)`,
`withTemperatureCutoff(...)` and `withFeedbackSynchronizationThreshold(...)`, as they do with an
LQR, and the Talons accept `withClosedLoopTolerance(...)`.

### `ADRCConfig`

`yams.core.motorcontrollers.config.ADRCConfig`. The settings for one motor controller's ADRC.
Every `with` method returns the config for chaining.

| Method | What it does |
| --- | --- |
| `new ADRCConfig(name)` | A config named `name` (letters, digits, `_` and `-`). |
| `withSystemOrder(int)` | Plant order, 1 to 4. Required. |
| `withSettlingTime(double)` | Settling time, seconds. Default 0.2. |
| `withBandwidthScale(double)` | Bandwidth scale. Default 1. |
| `withB0(double)` | Input gain. Default: derived, see above. |
| `withMaxVoltage(double)` | Output limits of `-max` to `max`, volts. |
| `withVoltageLimits(double min, double max)` | Asymmetric output limits, volts. |
| `withSlewRateLimit(double)` | Largest change in output per second, volts per second. 0, the default, turns it off. |
| `withInputDelay(double)` | Delay between the output and the plant responding, seconds, rounded to whole loops. 0, the default, turns it off. |
| `withCascaded(boolean)` | Use the cascaded observer: a second ESO, fed the first one's estimate of the measurement, supplies the disturbance the control law cancels. Default off. |
| `withTrackingDifferentiator(String method, double... params)` | Smooth the setpoint with a tracking differentiator, see below. `"none"`, the default, turns it off. |
| `getName()` | The config's name. |
| `getSystemOrder()` | `OptionalInt`, empty until set. |
| `getSettlingTime()`, `getBandwidthScale()`, `getSlewRateLimit()`, `getInputDelay()` | The value in use, or its default. |
| `getB0()`, `getMinVoltage()`, `getMaxVoltage()` | `OptionalDouble`, empty when it was not set. |
| `isCascaded()`, `getTdMethod()`, `getTdParams()` | The cascaded and tracking differentiator settings. |
| `resolveB0(DCMotor, SmartMotorControllerConfig)` | The `b0` the controller will use: the one set, or the derived one. Throws if it cannot be derived. |
| `buildController(DCMotor, SmartMotorControllerConfig, Time period)` | Build an initialized `ADRCController`. The motor controller calls this; you only need it to use the controller yourself. |
| `clone()` | A copy. `SmartMotorControllerConfig.clone()` copies each slot's ADRC config. |

Validation: each `with` method throws `IllegalArgumentException` for a value it cannot use, such
as a settling time that is not positive, or a minimum voltage at or above the maximum.
`buildController` throws `SmartMotorControllerConfigurationException` when the order is missing
or `b0` cannot be derived.

`ADRCConfig.DEFAULT_SETTLING_TIME`, `DEFAULT_BANDWIDTH_SCALE`, `DEFAULT_SLEW_RATE_LIMIT`,
`DEFAULT_INPUT_DELAY` and `DEFAULT_TD_METHOD` hold the defaults.

### `SmartMotorController`

| Method | What it does |
| --- | --- |
| `getADRCController()` | The `ADRCController` running the closed loop, if the config selected one. Use it to read the estimated disturbance or states. |
| `usesSystemCoreController()` | Protected. Whether an LQR or ADRC runs the closed loop on the SystemCore. Wrappers use it to send a plain voltage instead of an onboard closed loop request. |
| `applyADRCConfig(SmartMotorControllerConfig)` | Protected. Builds the slot's ADRC from the config, or clears it. Each wrapper calls it when a config is applied. |

### `ADRCController`

`yams.core.math.ADRCController`. The controller itself. The motor controller builds and runs it,
but it can also be used on its own:

```java
ADRCController adrc = new ADRCController(1);
adrc.initialize(
    0.2,            // settling time, seconds
    1.0,            // bandwidth scale
    b0,             // input gain
    -12.0, 12.0,    // output limits
    0.02,           // control period, seconds
    null,           // initial observer state (null: zero)
    0.0,            // initial output
    1.0,            // input gain scale: b0 is divided by it
    0.0,            // slew rate limit (0: off)
    0.0,            // input delay, seconds (0: off)
    "none", null,   // tracking differentiator and its parameters
    false);         // cascaded observer

// Every loop:
double volts = adrc.calculate(measurement, setpoint, appliedVoltage);
```

| Method | What it does |
| --- | --- |
| `ADRCController(int systemOrder)` | A controller for a plant of order 1 to 4. |
| `initialize(...)` | Set every parameter and compute the gains. Must be called before anything else. |
| `calculate(measurement, setpoint)` | One control step. Returns the output. |
| `calculate(measurement, setpoint, appliedOutput)` | One control step, telling the observer what the actuator applied since the last step. Use this when the actuator can apply less than asked. |
| `step(reference, output, refDerivatives)` | One control step with the setpoint's derivatives, for feeding a planned trajectory's velocity and acceleration. `calculate` calls this with none. |
| `reset(measurement)` | Start the observer at a measurement, with the other states and the output at zero. |
| `reset(double[] xhatInit, double uInit)` | Start the observer at a full state and output. |
| `updateTuning(settlingTime, bandwidthScale)` | Retune, keeping the estimated disturbance. |
| `setB0(b0)` | Change the input gain, keeping the estimated disturbance. |
| `setSaturation(min, max)` | Change the output limits. |
| `setTd(method, params)` | Change the tracking differentiator. |
| `getEstimatedDisturbance()` | The estimated total disturbance. Divide by `b0` to see it in volts. |
| `getEstimatedStates()` | The observer's state: the measurement, its derivatives, then the disturbance. |
| `getB0()`, `getBandwidth()`, `getObserverBandwidth()`, `getSettlingTime()`, `getBandwidthScale()` | The tuning in use. |
| `getMinOutput()`, `getMaxOutput()`, `getLastOutput()` | Output limits and the last output. |
| `getSystemOrder()`, `getControllerOrder()` | Plant order, and the observer's order (one more). |
| `isInitialized()` | Whether `initialize` has run. |

### `TrackingDifferentiator`

`yams.core.math.TrackingDifferentiator`. Turns a stepped setpoint into a smooth one with its first
and second derivatives, so a step does not kick the controller. `ADRCConfig.withTrackingDifferentiator`
puts one in front of the ADRC; it can also be used on its own.

```java
TrackingDifferentiator td = new TrackingDifferentiator("ld", 0.02, new double[] {0.2});
td.update(setpoint);
double smoothSetpoint = td.getY();
double setpointVelocity = td.getYd();
```

| Method | Parameters (defaults) | Behavior |
| --- | --- | --- |
| `euler` | `alpha` (0.9), from 0 to 1 | First order smoothing. Larger is smoother and slower. |
| `tod` | `r` (1), positive | Time optimal: reaches the setpoint as fast as a bounded acceleration `r` allows. |
| `ld` | `lambda` (1), more than half the sample time | Linear, critically damped, with time constant `lambda`. |
| `red` | `lambda1`, `lambda2` (1, 1), positive | Nonlinear, fast to converge near the setpoint. |
| `intd` | `alpha` (0.5) from 0 to below 1, `beta`, `gamma`, `r` (1, 1, 1) positive | Integral form with a smooth saturation. |

| Method | What it does |
| --- | --- |
| `TrackingDifferentiator(method, dt, params)` | A differentiator with a sample time, seconds. `params` may be `null` for the defaults. |
| `update(reference)` | Advance one sample. |
| `getY()`, `getYd()`, `getYdd()` | The smooth setpoint and its first and second derivatives. |
| `reset(value)` | Start at a value, at rest. |
| `setParameters(params)` | Change the parameters; invalid ones are rejected and the old ones kept. |
| `setSampleTime(dt)` | Change the sample time. |
| `getMethod()` | The method's name. |

## Things to know

- **One controller per slot serves position and velocity.** Its observer is in the units of the
  setpoint it was last reset for. Switching between a position and a velocity setpoint while
  enabled does not reset it yet, so pick one per mechanism.
- **Simulation needs an inertia**, as with a PID, unless the mechanism builds its own simulation.
- **Motion profiles still apply.** With a trapezoidal or exponential profile, the ADRC follows the
  profile's next position, as a PID does.
