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
    .withADRCController(new ADRCConfig()
        .withSystemOrder(SystemOrder.SECOND)  // a position loop
        .withSettlingTime(Seconds.of(0.6))
        .withMaxVoltage(Volts.of(12)));
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
- **From its config.** The `ADRCController` keeps only its running state: the observer's
  estimate, its last outputs and the tracking differentiator. Everything else it reads from its
  `ADRCConfig` every loop, including the observer and control gains. The config works the gains
  out when the controller is built, and again whenever its settling time, bandwidth scale or `b0`
  changes. So a change to any of those, or to the output limits, slew-rate limit or cascaded
  observer, takes effect on the next loop. The system order, input delay and tracking
  differentiator are fixed when the controller is built.

## Choosing the settings

### System order

The order of the plant from the voltage to what you control, set with an
`ADRCConfig.SystemOrder`: `FIRST` to `FOURTH`, with `FLYWHEEL` the same as `FIRST` and `ARM` and
`ELEVATOR` the same as `SECOND`.

| Mechanism | Setpoint | Order | `b0` |
| --- | --- | --- | --- |
| Flywheel, roller | Velocity | `FIRST` | Derived, or acceleration per volt |
| Arm, pivot, turret | Position | `SECOND` | Derived, or acceleration per volt |
| Elevator, or any mechanism whose speed is set by its back-EMF | Position | `FIRST` | Speed per volt: `Kv / (2 pi G)` |

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
so for an elevator use order 1 (`SystemOrder.FIRST`, not `ELEVATOR`) on its position, with its
speed per volt as `b0`:

```java
DCMotor motor = DCMotor.getNEO(1);
double gearing = 12; // motor rotations per drum rotation
config.withADRCController(new ADRCConfig()
    .withSystemOrder(SystemOrder.FIRST)
    .withSettlingTime(Seconds.of(0.5))
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
voltage, with an alert.

## Examples

A whole robot with an ADRC arm, elevator and shooter is in the `adrc_mechanisms` example, for
[Commands v2](examples/commands2/adrc_mechanisms/README.md) and
[Commands v3](examples/commands3/adrc_mechanisms/README.md).

These are the settings YAMS's tests use, which hold each mechanism at its setpoint on every motor
controller.

```java
// Flywheel: order 1 on velocity, b0 derived from the wheel's inertia.
config.withMomentOfInertia(KilogramSquareMeters.of(0.01))
      .withADRCController(new ADRCConfig()
          .withSystemOrder(SystemOrder.FIRST)
          .withSettlingTime(Seconds.of(0.15))
          .withMaxVoltage(Volts.of(12)));
flyWheel.setMechanismVelocitySetpoint(RPM.of(3000));

// Arm under gravity, with no kG: order 2 on position.
config.withMomentOfInertia(KilogramSquareMeters.of(0.1))
      .withADRCController(new ADRCConfig()
          .withSystemOrder(SystemOrder.SECOND)
          .withSettlingTime(Seconds.of(0.6))
          .withMaxVoltage(Volts.of(12)));
arm.setMechanismPositionSetpoint(Degrees.of(45));

// Pivot or turret: order 2 on position.
config.withMomentOfInertia(KilogramSquareMeters.of(0.1))
      .withADRCController(new ADRCConfig()
          .withSystemOrder(SystemOrder.SECOND)
          .withSettlingTime(Seconds.of(0.6))
          .withMaxVoltage(Volts.of(12)));

// Elevator under gravity, with no kG: order 1 on position, b0 is its speed per volt.
config.withMechanismCircumference(Meters.of(2 * Math.PI * 0.02))
      .withADRCController(new ADRCConfig()
          .withSystemOrder(SystemOrder.FIRST)
          .withSettlingTime(Seconds.of(0.5))
          .withB0(motor.Kv / (2 * Math.PI * gearing))
          .withMaxVoltage(Volts.of(12)));
elevator.setMeasurementPositionSetpoint(Meters.of(0.5));
```

## Telemetry and live tuning

With an ADRC, a motor controller's telemetry hides kP, kI and kD and, at `HIGH` verbosity,
publishes these under `closedloop/adrc/`. A motor controller without an ADRC never shows them.

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
| `maxVoltage` | Yes | Maximum output, volts. The minimum is its negative unless the ADRC config sets one. |
| `slewRateLimit` | Yes | Largest change in output per second, volts per second. 0 turns it off. |

The tunable values are applied while the Live Tuning command runs. Like a tuned PID, they are kept
in the motor controller's `ADRCConfig`, so applying the motor config again keeps them. Loosely
coupled followers are tuned the same way.

The ADRC reads its config every loop (see [How it runs](#how-it-runs)), so a tuned value takes
effect on the next loop. The estimated disturbance carries over, so the output does not jump.

Live tuning first tries the new values on a copy of the config. If the config refuses one, such as
a `b0` of zero, every value stays as it was and a YAMS alert names the value until it is changed.

Robot code can retune the same way, for example to soften an arm while it carries a game piece:

```java
smc.getConfig().getADRCController().ifPresent(adrc -> adrc.withSettlingTime(Seconds.of(0.8)));
```

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

`yams.core.math.ADRCConfig`. The settings for one motor controller's ADRC. Every `with` method
returns the config for chaining.

Every setting starts empty. The getters return that `Optional` as it is, and the controller
fills each empty one with its default. The `with` methods only accept values the controller can
use: anything else throws `IllegalArgumentException` and leaves the config as it was. The
controller reads the config directly, so it never sees a value it cannot run.

Once a controller is built from it, the config also holds what the controller needs that the
settings do not give: the control period, the `b0` used while `b0` is unset (the derived one), the
maximum output used while it is unset (the motor config's limit or the nominal voltage), and the
gains worked out from them. Building again, as applying the motor config again does, replaces
them.

| Method | What it does |
| --- | --- |
| `new ADRCConfig()` | A config for one motor controller. Its errors and alerts name that motor controller by its telemetry name. |
| `withSystemOrder(SystemOrder)` | Plant order. Required. Once a controller is built, a different order throws `IllegalStateException`: the observer's size depends on it, so a new order needs a new `ADRCConfig`. |
| `withSettlingTime(Time)` | Settling time. Default 0.2 s. Must be finite and positive. Works the gains out again. |
| `withBandwidthScale(double)` | Bandwidth scale. Default 1. Must be finite and positive. Works the gains out again. |
| `withB0(double)` | Input gain. Default: derived, see above. Must be finite and non-zero. Works the gains out again. |
| `withMaxVoltage(Voltage)` | Output limits of `-max` to `max`, or up to `max` from a minimum that is set. Must not be NaN or below the minimum. |
| `withVoltageLimits(Voltage min, Voltage max)` | Asymmetric output limits. Must not be NaN, with the minimum not above the maximum. |
| `withSlewRateLimit(Velocity<VoltageUnit>)` | Largest change in output per second, such as `Volts.per(Second).of(24)`. 0, the default, turns it off. Must not be negative. |
| `withInputDelay(Time)` | Delay between the output and the plant responding, rounded to whole loops. 0, the default, turns it off. Must be finite and not negative. |
| `withCascaded(boolean)` | Use the cascaded observer: a second ESO, fed the first one's estimate of the measurement, supplies the disturbance the control law cancels. Default off. |
| `withTrackingDifferentiator(TrackingDifferentiator.Method, double... params)` | Smooth the setpoint with a tracking differentiator, see below. Leave out `params` for the method's defaults. Off by default. Its parameters are checked when the controller is built, as some depend on the period. |
| `withoutTrackingDifferentiator()` | Turn the tracking differentiator off. |
| `getSystemOrder()` | `Optional<SystemOrder>`. |
| `getSettlingTime()`, `getInputDelay()` | `Optional<Time>`. |
| `getBandwidthScale()`, `getB0()` | `OptionalDouble`. |
| `getMinVoltage()`, `getMaxVoltage()` | `Optional<Voltage>`. |
| `getSlewRateLimit()` | `Optional<Velocity<VoltageUnit>>`. |
| `getTdMethod()` | `Optional<TrackingDifferentiator.Method>`. |
| `getTdParams()` | `Optional<double[]>`: a copy of the tracking differentiator parameters, or empty for the defaults. |
| `isCascaded()` | Whether the cascaded observer is on. |
| `resolveB0(DCMotor, SmartMotorControllerConfig)` | The `b0` the controller will use: the one set, or the derived one. Throws if it cannot be derived. |
| `buildController(DCMotor, SmartMotorControllerConfig, Time period)` | Build an initialized `ADRCController` that reads this config, giving the config the period, `b0` and maximum output it needs. The motor controller calls this; you only need it to use the controller yourself. |
| `reportRejected(Optional<String> reason)` | Raise the YAMS alert for a refused value, or clear it with an empty reason. Live tuning calls it. |
| `clone()` | A deep copy: the settings, the values given when a controller was built, and the gains, with matrices of its own. Changing the copy leaves the original alone. The copy raises its own alerts. `SmartMotorControllerConfig.clone()` copies each slot's ADRC config this way. |

`buildController` throws `SmartMotorControllerConfigurationException` when the order is missing
or `b0` cannot be derived. When neither the ADRC config nor the motor config limits the output, it
uses the motor's nominal voltage and raises a YAMS alert saying so.

`ADRCConfig.DEFAULT_SETTLING_TIME`, `DEFAULT_BANDWIDTH_SCALE`, `DEFAULT_SLEW_RATE_LIMIT` and
`DEFAULT_INPUT_DELAY` hold the defaults.

### `SmartMotorController`

| Method | What it does |
| --- | --- |
| `getADRCController()` | The `ADRCController` running the closed loop, if the config selected one. Use it to read the estimated disturbance or states. |
| `usesSystemCoreController()` | Protected. Whether an LQR or ADRC runs the closed loop on the SystemCore. Wrappers use it to send a plain voltage instead of an onboard closed loop request. |
| `applyADRCConfig(SmartMotorControllerConfig)` | Protected. Builds the slot's ADRC from the config, or clears it. Each wrapper calls it when a config is applied. |

### `ADRCController`

`yams.core.math.ADRCController`. The controller itself. The motor controller builds and runs it,
but it can also be used on its own. Every setting comes from the controller's `ADRCConfig`;
`initialize` only takes what the config can leave unset and the controller still needs: `b0`, the
maximum output and the control period.

```java
ADRCController adrc = new ADRCController(new ADRCConfig()
    .withSystemOrder(SystemOrder.FIRST)
    .withSettlingTime(Seconds.of(0.2)));
// b0 and the maximum output, used while the config leaves them unset, and the control period.
adrc.initialize(b0, Volts.of(12), Milliseconds.of(20));

// Every loop:
double volts = adrc.calculate(measurement, setpoint, appliedVoltage);
```

| Method | What it does |
| --- | --- |
| `ADRCController(ADRCConfig)` | A controller that reads its settings from a config, with the config's system order. Throws if the config has none. |
| `initialize(double b0, Voltage maxVoltage, Time period)` | Give the config the values it cannot hold itself, which makes it work out the gains, and start the observer and output at zero. `b0` and `maxVoltage` are used while the config leaves them unset; `buildController` passes the derived `b0` and the motor config's limit. Must be called before anything else. |
| `calculate(measurement, setpoint)` | One control step. Returns the output. |
| `calculate(measurement, setpoint, appliedOutput)` | One control step, telling the observer what the actuator applied since the last step. Use this when the actuator can apply less than asked. |
| `step(reference, output, refDerivatives)` | One control step with the setpoint's derivatives, for feeding a planned trajectory's velocity and acceleration. `calculate` calls this with none. |
| `reset(measurement)` | Start the observer at a measurement, with the other states and the output at zero. |
| `reset(double[] xhatInit, double uInit)` | Start the observer at a full state and output. |
| `getEstimatedDisturbance()` | The estimated total disturbance. Divide by `b0` to see it in volts. |
| `getEstimatedStates()` | The observer's state: the measurement, its derivatives, then the disturbance. |
| `getB0()`, `getSettlingTime()`, `getBandwidthScale()` | The config's tuning, with its defaults and the `b0` given to `initialize` filling in what it leaves unset. `getSettlingTime()` returns a `Time`. |
| `getBandwidth()`, `getObserverBandwidth()` | The bandwidths that tuning gives. |
| `getMinOutput()`, `getMaxOutput()`, `getLastOutput()` | The config's output limits, with the maximum given to `initialize` when it leaves it unset, and the last output. |
| `getSlewRateLimit()` | The config's largest change in output per second, or 0 when it is off. |
| `getTrackingDifferentiator()` | The tracking differentiator, if the config turned one on. |
| `getConfig()` | The `ADRCConfig` the controller reads. |
| `getSystemOrder()`, `getControllerOrder()` | Plant order, and the observer's order (one more). |
| `isInitialized()` | Whether `initialize` has run. |

### `TrackingDifferentiator`

`yams.core.math.TrackingDifferentiator`. Turns a stepped setpoint into a smooth one with its first
and second derivatives, so a step does not kick the controller. `ADRCConfig.withTrackingDifferentiator`
puts one in front of the ADRC; it can also be used on its own.

```java
TrackingDifferentiator td =
    new TrackingDifferentiator(TrackingDifferentiator.Method.LD, 0.02, Optional.of(new double[] {0.2}));
td.update(setpoint);
double smoothSetpoint = td.getY();
double setpointVelocity = td.getYd();
```

| Method | Parameters (defaults) | Behavior |
| --- | --- | --- |
| `EULER` | `alpha` (0.9), from 0 to 1 | First order smoothing. Larger is smoother and slower. |
| `TOD` | `r` (1), positive | Time optimal: reaches the setpoint as fast as a bounded acceleration `r` allows. |
| `LD` | `lambda` (1), more than half the sample time | Linear, critically damped, with time constant `lambda`. |
| `RED` | `lambda1`, `lambda2` (1, 1), positive | Nonlinear, fast to converge near the setpoint. |
| `INTD` | `alpha` (0.5) from 0 to below 1, `beta`, `gamma`, `r` (1, 1, 1) positive | Integral form with a smooth saturation. |

| Method | What it does |
| --- | --- |
| `TrackingDifferentiator(Method, dt, Optional<double[]> params)` | A differentiator with a sample time, seconds. Empty `params` use the defaults. |
| `update(reference)` | Advance one sample. |
| `getY()`, `getYd()`, `getYdd()` | The smooth setpoint and its first and second derivatives. |
| `reset(value)` | Start at a value, at rest. |
| `setParameters(params)` | Change the parameters; invalid ones are rejected and the old ones kept. |
| `setSampleTime(dt)` | Change the sample time. |
| `getMethod()` | The `Method`. |

## Things to know

- **One controller per slot serves position and velocity.** Its observer is in the units of the
  setpoint it was last reset for. Switching between a position and a velocity setpoint while
  enabled does not reset it yet, so pick one per mechanism.
- **Simulation needs an inertia**, as with a PID, unless the mechanism builds its own simulation.
- **Motion profiles still apply.** With a trapezoidal or exponential profile, the ADRC follows the
  profile's next position, as a PID does.
- **One `ADRCConfig` per motor controller.** A config holds the period, `b0` and maximum output of
  the controller built from it, so two motor controllers sharing one config instance share those
  too: the last one built sets them for both. Give each motor controller its own config, or a
  `clone()` of one.
- **Tuning is safe while the loop runs.** The closed loop thread reads the gains once per loop,
  and the config replaces them whole when it works them out again, so a loop never runs with a mix
  of old and new gains.
