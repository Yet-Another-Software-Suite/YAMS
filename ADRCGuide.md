# ADRC Guide

ADRC (Active Disturbance Rejection Control) is a closed loop controller that replaces a PID and
its feedforward. **ADRC in YAMS was written by FRC Team 8331**, who contributed it from their fork
of YAMS.

An ADRC treats everything it is not told about (gravity, friction, back-EMF, a game piece in the
mechanism, a bumped arm) as one "total disturbance". An extended state observer (ESO) estimates
that disturbance every loop, and the control law cancels it. So an ADRC needs very little model:
the plant's order and an approximate input gain `b0`. There is no kS, kV, kA or kG to tune.

ADRC works the same in Java and C++. This guide uses Java; [C++](#c) lists what differs.

## Quick start

```java
SmartMotorControllerConfig config = new SmartMotorControllerConfig(this)
    .withControlMode(ControlMode.CLOSED_LOOP)
    .withGearing(new MechanismGearing(GearBox.fromReductionStages(3, 4)))
    .withMomentOfInertia(Inches.of(18), Pounds.of(8)) // b0 is derived from this
    .withClosedLoopControllerMaximumVoltage(Volts.of(12))
    .withADRCController(new ADRCConfig()
        .withSystemOrder(SystemOrder.ARM)  // a position loop; first or second order is chosen for you
        .withSettlingTime(Seconds.of(0.6)));
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
- **Limited and ramped by the motor config.** The motor controller limits the output to
  `withClosedLoopControllerMaximumVoltage(...)` and ramps it with `withClosedLoopRampRate(...)`,
  as for every closed loop. See [Output limits and ramp](#output-limits-and-ramp).
- **From where the mechanism is.** When the closed loop starts, and every loop while the robot
  is disabled, the observer is reset to the measurement: the position, or the velocity while a
  velocity setpoint is set. Enabling the robot does not look like a step from zero.
- **From its config.** The `ADRCController` keeps only its running state: the observer's
  estimate, its last outputs and the tracking differentiator. Everything else it reads from its
  `ADRCConfig` every loop, including the observer and control gains. The config works the gains out when the controller is built, and again whenever
  its settling time, bandwidth scale or `b0` changes. So a change to any of those, or to the
  cascaded observer, takes effect on the next loop. The system order, input delay and tracking
  differentiator are fixed when the controller is built.

## Choosing the settings

### System order

The order of the plant from the voltage to what you control, set with an
`ADRCConfig.SystemOrder`.

| Mechanism | Setpoint | Order | `b0` |
| --- | --- | --- | --- |
| Flywheel, roller | Velocity | `FIRST` (or `FLYWHEEL`) | Acceleration per volt, derived from the inertia |
| Arm, pivot, turret | Position | `ARM`, `PIVOT` or `POSITION` | Chosen when built, see below |
| Elevator | Position, linear | `ELEVATOR` | Chosen when built, see below; in meters |

`FIRST`, `SECOND`, `THIRD` and `FOURTH` are fixed orders, for when you know the plant.

There is no default system order. `buildController` throws until one is set.

#### Position loops

A position loop is one of two plants, depending on what sets how fast the mechanism moves:

- **Its inertia**, for a lightly geared mechanism. A voltage accelerates it, and the acceleration
  integrates twice into its position: second order, with its acceleration per volt as `b0`.
- **Its motor's back-EMF**, for a heavily geared one. Each volt moves it at about the same speed,
  reached sooner than the loop can act: first order, with its speed per volt as `b0`.

The wrong form works badly. A 60:1 NEO pivot run second order overshoots a 135° step by 77°; first
order, it settles in half a second with no overshoot. Geared 12:1, the same pivot is the other way
round.

`POSITION` (and `ARM`, `PIVOT` and `ELEVATOR`, which are the same) chooses for you when the
controller is built, from the motor's back-EMF time constant at the mechanism:

```
tau = J * R / (G^2 * Kt * Ke)    seconds
```

where `J` is the moment of inertia, `G` motor rotations per mechanism rotation, and `R`, `Kt` and
`Ke` (the inverse of `Kv`) come from the `DCMotor`. When `tau` is under two control periods (40 ms
at the default 20 ms, `ADRCConfig.SPEED_PER_VOLT_TIME_CONSTANT_PERIODS`), the position loop is first
order with its speed per volt as `b0`; otherwise it is second order with its acceleration per volt.

| Mechanism | `tau` | Chosen |
| --- | --- | --- |
| NEO, 60:1, 0.1 kg·m² pivot | 6 ms | First order, speed per volt |
| NEO, 60:1, 3 kg 0.5 m arm | 16 ms | First order, speed per volt |
| NEO, 12:1, 0.1 kg·m² pivot | 160 ms | Second order, acceleration per volt |

The choice needs the mechanism's moment of inertia, from `withMomentOfInertia(...)`. Without it,
building the controller throws, except for `ELEVATOR`.

#### Elevators

`ELEVATOR` is `POSITION` in a linear closed loop: it runs in meters, so its `b0` is too. It needs a
linear closed loop: without one, building the controller throws, even with `withB0(...)` set. With
an ADRC there is no `ElevatorFeedforward` to turn the linear closed loop on, so turn it on yourself.
Without a moment of inertia, an elevator is first order with its speed per volt, since an
elevator's speed is nearly always set by its motor's back-EMF.

```java
config.withGearing(new MechanismGearing(GearBox.fromReductionStages(3, 4)))
    .withMechanismCircumference(Inches.of(0.25).times(22)) // 22T sprocket, #25 chain
    .withLinearClosedLoopController(true)
    .withADRCController(new ADRCConfig()
        .withSystemOrder(SystemOrder.ELEVATOR)
        .withSettlingTime(Seconds.of(0.5)));
```

### `b0`, the input gain

`b0` is how much one volt moves the plant's highest derivative, in the loop's units: mechanism
rotations, or meters for a linear closed loop. If it is not set, it is derived:

```
acceleration per volt:  b0 = G * Kt / (2 * pi * R * J)    rotations per second squared per volt
speed per volt:         b0 = Kv / (2 * pi * G)            rotations per second per volt
```

With a linear closed loop both are multiplied by the mechanism circumference. A velocity loop and
a second order position loop use the acceleration per volt, a first order position loop the speed
per volt (see [Position loops](#position-loops)).

The acceleration per volt only uses an inertia you set with `withMomentOfInertia(...)` or
`withWheelMomentOfInertia(...)`. YAMS's placeholder inertia is never used: without one, and
without `withB0(...)`, building the controller throws.

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

### Output limits and ramp

The motor config limits and ramps the ADRC's output, as it does for every closed loop:

- **`withClosedLoopControllerMaximumVoltage(...)`** limits the output, from `-max` to `max`.
  The ADRC has no limit of its own; without this, the motor controller applies what it can.
- **`withClosedLoopRampRate(...)`** limits how quickly the output changes: the ramp rate is the
  time it takes to change by 12 V. A motor controller treats the voltage a SystemCore closed loop
  sends it as open loop, so its own closed loop ramp never applied to one; YAMS ramps the output
  of an ADRC or an LQR itself, on every motor controller, the A301 too.

The observer is told the voltage the motor controller applied, so a limit or ramp that holds the
output back is not taken for a disturbance.

### Smoothing the setpoint: the tracking differentiator

A tracking differentiator (TD) is a filter in front of the ADRC. Each loop it takes the raw
setpoint and gives the ADRC a smooth one, `y`, which glides to the setpoint instead of jumping,
and its rate of change, `yd`. The ADRC then follows `y` and uses `yd` as the setpoint's speed (for
a position loop) or acceleration (for a velocity loop), so a step does not kick it and it does not
lag a moving setpoint. It is off by default.

```java
new ADRCConfig()
    .withSystemOrder(SystemOrder.ARM)
    .withSettlingTime(Seconds.of(0.3))
    // A linear TD with a 0.08 s time constant: a step is followed in about 0.46 s.
    .withTrackingDifferentiator(TrackingDifferentiator.Method.LD, 0.08)
```

```cpp
adrc.WithTrackingDifferentiator(yams::math::TrackingDifferentiator::Method::LD, {0.08});
```

**When to use one.** A step setpoint on a mechanism that should not be slammed, when you do not
use a motion profile. A motion profile on the motor config does the same job with real speed and
acceleration limits; do not use both, since the TD only adds lag to a setpoint the profile has
already smoothed. The TD is set when the controller is built: its method and parameters are not
live tunable.

**Units.** The TD works in the loop's units: mechanism rotations for an arm or pivot, meters for
an elevator, and rotations per second for a flywheel. Parameters in "loop units" below change
meaning with them; the ones in seconds do not.

#### The methods and their parameters

Parameters are passed in order, and any left off take the default. The defaults are slow for most
mechanisms (`LD`'s is a 1 s time constant), so set them.

| Method | Parameters (default) | How it moves to a new setpoint |
| --- | --- | --- |
| `LD`, linear | `lambda` (1), seconds, more than half the control period | A critically damped glide, with no overshoot. Reaches 98% in about 5.8 `lambda`, at a peak speed of about step / (e × `lambda`). |
| `INTD`, saturating | `alpha` (0.5), from 0 to below 1; `beta` (1), loop units; `gamma` (1), loop units; `r` (1), per second | Far from the setpoint, it cruises at `r·beta/(1 - alpha)`. Within about `gamma/(1 - alpha)` of it, it lands like a linear TD with natural frequency `r·sqrt(beta/gamma)` and damping `0.5·sqrt(gamma/beta)`. |
| `EULER`, exponential | `alpha` (0.9), from 0 to 1, per loop | A first order lag with a time constant of about one period / (1 - `alpha`). Its `yd` jumps on a step, to (1 - `alpha`) × step / period. |
| `TOD`, time optimal | `r` (1), loop units per s² | Bang-bang: full acceleration `r` toward the setpoint, then full braking. |
| `RED`, nonlinear | `lambda1` (1), sqrt(loop units) per s; `lambda2` (1), loop units per s² | A super-twisting filter: `lambda2` bounds how quickly `yd` changes, `lambda1` how hard the error pulls. |

Every method also works out `ydd`, the second derivative, but the ADRC does not use it.

#### Choosing and tuning one

These are measured on the TD alone at the 20 ms default period, stepping an arm 90° (0.25
rotations), an elevator 1 m and a flywheel 50 rotations per second. Each cell is the time to stay
within 2% of the step, and the overshoot:

| Method and parameters | Arm | Elevator | Flywheel | Notes |
| --- | --- | --- | --- | --- |
| `LD` 1 (default) | 5.8 s, 0% | 5.8 s, 0% | 5.8 s, 0% | Far too slow. |
| `LD` 0.1 | 0.52 s, 0% | 0.52 s, 0% | 0.52 s, 0% | |
| `LD` 0.05 | 0.22 s, 0% | 0.22 s, 0% | 0.22 s, 0% | |
| `INTD` 0.5, 1, 1, 1 (default) | 8.1 s, 17% | 8.1 s, 16% | Not within 12 s | Overshoots, and crawls on big steps. |
| `INTD` 0.5, 1, 4, 10 | 1.1 s, 0% | 1.1 s, 0% | 2.9 s, 0% | Cruises at 20 loop units/s. |
| `INTD` 0.5, 1, 4, 20 | 0.52 s, 0% | 0.52 s, 0% | 1.4 s, 0% | Cruises at 40 loop units/s. |
| `EULER` 0.9 (default) | 0.74 s, 0% | 0.74 s, 0% | 0.74 s, 0% | `yd` jumps to 1.25 rot/s on the arm's step. |
| `TOD` 4 | Never holds 2%, 8% | 1.4 s, 8% | 6.2 s, 1% | `yd` chatters at rest. |
| `RED` 3, 4 | 0.40 s, 13% | 0.76 s, 10% | 5.3 s, 8% | Overshoots; `yd` chatters at rest. |

- **Start with `LD`.** One parameter, in seconds, that means the same thing on every mechanism,
  and it never overshoots. Pick `lambda` so 5.8 × `lambda` is about the time a move should take,
  then check that the peak speed, step / (e × `lambda`), is one the mechanism can reach: a
  `lambda` of 0.08 s moves an arm 90° in about 0.46 s, at up to 1.15 rotations per second, which
  a NEO on 60:1 (top speed about 1.5) can follow. Lengthen
  it if the output saturates during a move; shorten it if the mechanism waits on the TD.
- **Use `INTD` when step sizes vary a lot**, so big moves cruise at a speed the mechanism can hold
  while small ones still land quickly. Leave `alpha` at 0.5, then set `beta` and `r` so the cruise
  speed `r·beta/(1 - alpha)` is a little under the mechanism's top speed, and `gamma` to 4 ×
  `beta` for no overshoot (critical damping). Raise `r` for a faster landing. A smaller `gamma`
  lands faster but overshoots: at `gamma` = `beta`, by about 16%.
- **Avoid `EULER` on a position loop.** It smooths `y`, but its `yd` kicks on every step, which is
  what a TD is meant to prevent.
- **Avoid `TOD` and `RED` at the default period.** Their sign switching makes `yd` chatter every
  loop at rest, which the ADRC passes on to the motor, and a large `TOD` `r` never settles. They
  need a much shorter period than 20 ms to work as intended.

## Tuning an ADRC

An ADRC has few knobs, and the defaults get a mechanism moving. Tuning means picking the system
order, giving YAMS what it needs to derive `b0`, and then shortening the settling time until the
mechanism stops getting better.

### 1. Describe the plant

- **Pick the system order.** `FLYWHEEL` (or `FIRST`) for a velocity loop, and `ARM`, `PIVOT`,
  `POSITION` or `ELEVATOR` for a position loop, which choose first or second order for you (see
  [Position loops](#position-loops)).
- **Set the moment of inertia** with `withMomentOfInertia(...)`, so YAMS can derive `b0` and choose
  a position loop's form. Without it, set `withB0(...)` and a fixed order yourself.
- **Set the output limit and ramp on the motor config:** `withClosedLoopControllerMaximumVoltage(...)`,
  and `withClosedLoopRampRate(...)` only if the mechanism needs its output ramped. A ramp adds lag,
  which makes a position loop overshoot more.

### 2. Graph it

Run the robot, in simulation first, with `HIGH` telemetry, and graph these in AdvantageScope:

| Topic | What to look for |
| --- | --- |
| `closedloop/setpoint/position` or `velocity` | The step you command. |
| `mechanism/position` or `velocity` (`measurement/position` for a linear loop) | How the mechanism follows it. |
| `motor/outputVoltage` | Whether the output saturates, and whether it chatters. |
| `closedloop/adrc/disturbance` | What the observer estimates: steady under gravity, smooth while moving. |

Command steps of the size the robot uses, both ways. The example robots' demo steps an arm 45°,
90°, 0° and -20°, a second apart.

### 3. Shorten the settling time

Start at about 0.5 s for a position loop and 0.3 s for a velocity loop, and shorten the settling
time while each step gets faster and stays clean. Stop when shortening it no longer helps:

- **The output saturates for most of a move.** The mechanism is going as fast as it can, and a
  shorter settling time only makes it overshoot. A big elevator move is like this.
- **It overshoots or rings.** Lengthen the settling time again.
- **The output or the disturbance chatters.** The observer is fast enough to amplify sensor noise.

These are the measurements behind the example robots' settings, in simulation:

| Mechanism | Settling time | Worst overshoot | Settles in |
| --- | --- | --- | --- |
| Arm, NEO 60:1, 3 kg, 0.5 m (first order) | 0.6 s | 0° | 0.60 s |
| | **0.3 s** | 0° | 0.34 s |
| | 0.2 s | 0° | 0.28 s |
| Elevator, NEO 12:1, 16 lb, 22T #25 sprocket | 0.5 s | 0 m | 1.28 s |
| | **0.3 s** | 0 m | 1.16 s |
| Shooter, NEO direct, 4 in 1 lb wheel | 0.3 s | 2.7 RPM | 0.48 s |
| | **0.2 s** | 2.8 RPM | 0.44 s |
| | 0.15 s | 46 RPM | 0.44 s |
| | 0.1 s | 96 RPM | 0.42 s |

The elevator barely speeds up: its motor's top speed limits a 1 m move, whatever the settling time.
The examples leave some margin rather than take the fastest clean value, since a real mechanism has
friction, backlash and sensor noise the simulation does not.

### 4. Adjust `b0` if it still misbehaves

The derived `b0` is a starting point, and the observer absorbs one off by a factor of two. If the
mechanism still overshoots or oscillates, lower `b0`; if it is sluggish, raise it. On a 12:1 arm,
half the derived `b0` cut the overshoot from 2.7° to 1.2°, and twice it raised it to 13.6°.

A position loop that overshoots badly whatever the settling time is likely the wrong form: check
which order it chose (`closedloop/adrc/systemOrder`), and the moment of inertia it was chosen from.

### 5. Tune live, then copy the values back

With `HIGH` telemetry, the Live Tuning command retunes `b0`, the settling time and the bandwidth
scale while the robot runs (see [Telemetry and live tuning](#telemetry-and-live-tuning)). Tuned
values last until the robot restarts: copy the ones you keep into the code.

### Leave these alone at first

- **Bandwidth scale** scales both bandwidths together. The settling time does the same job in units
  you can reason about.
- **Input delay** and the **cascaded observer** are for unusual plants.
- **The tracking differentiator** only matters when step setpoints slam the mechanism and it has
  no motion profile; see [Smoothing the setpoint](#smoothing-the-setpoint-the-tracking-differentiator).

## Examples

A whole robot with an ADRC arm, elevator and shooter is in the `adrc_mechanisms` example, for
[Commands v2](examples/commands2/adrc_mechanisms/README.md) and
[Commands v3](examples/commands3/adrc_mechanisms/README.md). The C++ example robot has the same three as
`ADRCArmSubsystem`, `ADRCElevatorSubsystem` and `ADRCShooterSubsystem` in
[examples/cpptest](examples/cpptest/README.md#adrc-subsystems).

These are the settings YAMS's tests use, which hold each mechanism at its setpoint on every motor
controller. The example robots use the tuned settling times from [Tuning an ADRC](#tuning-an-adrc). Each motor config also has `withClosedLoopControllerMaximumVoltage(Volts.of(12))`.

```java
// Flywheel: order 1 on velocity, b0 derived from the wheel's inertia.
config.withMomentOfInertia(KilogramSquareMeters.of(0.01))
      .withADRCController(new ADRCConfig()
          .withSystemOrder(SystemOrder.FIRST)
          .withSettlingTime(Seconds.of(0.15)));
flyWheel.setMechanismVelocitySetpoint(RPM.of(3000));

// Arm under gravity, with no kG: its position loop's form is chosen from the motor, gearing and inertia.
config.withMomentOfInertia(KilogramSquareMeters.of(0.1))
      .withADRCController(new ADRCConfig()
          .withSystemOrder(SystemOrder.ARM)
          .withSettlingTime(Seconds.of(0.6)));
arm.setMechanismPositionSetpoint(Degrees.of(45));

// Pivot or turret: the same.
config.withMomentOfInertia(KilogramSquareMeters.of(0.1))
      .withADRCController(new ADRCConfig()
          .withSystemOrder(SystemOrder.PIVOT)
          .withSettlingTime(Seconds.of(0.6)));

// Elevator under gravity, with no kG: first order on position in meters, b0 its derived speed per volt.
config.withMechanismCircumference(Meters.of(2 * Math.PI * 0.02))
      .withLinearClosedLoopController(true)
      .withADRCController(new ADRCConfig()
          .withSystemOrder(SystemOrder.ELEVATOR)
          .withSettlingTime(Seconds.of(0.5)));
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
| `b0` | Yes | Input gain. |
| `settlingTime` | Yes | Settling time, seconds. |
| `bandwidthScale` | Yes | Bandwidth scale. |

The output limit and ramp are the motor config's, tuned like any closed loop's ramp rate.

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
settings do not give: the control period, the `b0` used while `b0` is unset (the derived one), and
the gains worked out from them. Building again, as applying the motor config again does, replaces
them.

| Method | What it does |
| --- | --- |
| `new ADRCConfig()` | A config for one motor controller. Its errors and alerts name that motor controller by its telemetry name. |
| `withSystemOrder(SystemOrder)` | Plant order. Required. Once a controller is built, an order that would build a different controller (not just an alias) throws `IllegalStateException`: the observer's size and the kind of `b0` depend on it, so a new order needs a new `ADRCConfig`. |
| `withSettlingTime(Time)` | Settling time. Default 0.2 s. Must be finite and positive. Works the gains out again. |
| `withBandwidthScale(double)` | Bandwidth scale. Default 1. Must be finite and positive. Works the gains out again. |
| `withB0(double)` | Input gain. Default: derived, see above. Must be finite and non-zero. Works the gains out again. |
| `withInputDelay(Time)` | Delay between the output and the plant responding, rounded to whole loops. 0, the default, turns it off. Must be finite and not negative. |
| `withCascaded(boolean)` | Use the cascaded observer: a second ESO, fed the first one's estimate of the measurement, supplies the disturbance the control law cancels. Default off. |
| `withTrackingDifferentiator(TrackingDifferentiator.Method, double... params)` | Smooth the setpoint with a tracking differentiator, see below. Leave out `params` for the method's defaults. Off by default. Its parameters are checked when the controller is built, as some depend on the period. |
| `withoutTrackingDifferentiator()` | Turn the tracking differentiator off. |
| `getSystemOrder()` | `Optional<SystemOrder>`. `SystemOrder.getOrder()` is 0 for a position loop chosen when built; `isChosenWhenBuilt()` and `requiresLinearClosedLoop()` tell them apart. The controller's `getSystemOrder()` gives the chosen order. |
| `getSettlingTime()`, `getInputDelay()` | `Optional<Time>`. |
| `getBandwidthScale()`, `getB0()` | `OptionalDouble`. |
| `getTdMethod()` | `Optional<TrackingDifferentiator.Method>`. |
| `getTdParams()` | `Optional<double[]>`: a copy of the tracking differentiator parameters, or empty for the defaults. |
| `isCascaded()` | Whether the cascaded observer is on. |
| `resolveB0(DCMotor, SmartMotorControllerConfig)` | The `b0` the controller will use: the one set, or the derived one, a speed per volt for a first order position loop. Throws if it cannot be derived, if a position loop's form cannot be chosen, or if the order is `ELEVATOR` and the closed loop is not linear. |
| `buildController(DCMotor, SmartMotorControllerConfig, Time period)` | Build an initialized `ADRCController` that reads this config, giving the config the period and `b0` it needs. The controller does not limit or ramp its output; the motor controller does. The motor controller calls this; you only need it to use the controller yourself. |
| `reportRejected(Optional<String> reason)` | Raise the YAMS alert for a refused value, or clear it with an empty reason. Live tuning calls it. |
| `clone()` | A deep copy: the settings, the values given when a controller was built, and the gains, with matrices of its own. Changing the copy leaves the original alone. The copy raises its own alerts. `SmartMotorControllerConfig.clone()` copies each slot's ADRC config this way. |

`buildController` throws `SmartMotorControllerConfigurationException` when the order is missing
or `b0` cannot be derived.

`ADRCConfig.DEFAULT_SETTLING_TIME`, `DEFAULT_BANDWIDTH_SCALE` and `DEFAULT_INPUT_DELAY` hold the
defaults.

### `SmartMotorController`

| Method | What it does |
| --- | --- |
| `getADRCController()` | The `ADRCController` running the closed loop, if the config selected one. Use it to read the estimated disturbance or states. |
| `usesSystemCoreController()` | Protected. Whether an LQR or ADRC runs the closed loop on the SystemCore. Wrappers use it to send a plain voltage instead of an onboard closed loop request. |
| `applyADRCConfig(SmartMotorControllerConfig)` | Protected. Builds the slot's ADRC from the config, or clears it. Each wrapper calls it when a config is applied. |

### `ADRCController`

`yams.core.math.ADRCController`. The controller itself. The motor controller builds and runs it,
but it can also be used on its own. Every setting comes from the controller's `ADRCConfig`;
`initialize` takes what the config can leave unset and the controller still needs: `b0` and the
control period.

The controller does not limit or ramp its output. Used on its own, clamp the output to what the
actuator can apply, ramp it with WPILib's `SlewRateLimiter` if it needs one, and tell the observer
what was applied with `calculate(measurement, setpoint, appliedOutput)`, so a clamped output is not
taken for a disturbance.

```java
ADRCController adrc = new ADRCController(new ADRCConfig()
    .withSystemOrder(SystemOrder.FIRST)
    .withSettlingTime(Seconds.of(0.2)));
// b0, used while the config leaves it unset, and the control period.
adrc.initialize(b0, Milliseconds.of(20));

// Every loop: clamp the output, and tell the observer what was applied last loop.
double volts = MathUtil.clamp(adrc.calculate(measurement, setpoint, appliedVoltage), -12, 12);
```

| Method | What it does |
| --- | --- |
| `ADRCController(ADRCConfig)` | A controller that reads its settings from a config, with the config's system order. Throws if the config has none. |
| `initialize(double b0, Time period)` | Give the config the values it cannot hold itself, which makes it work out the gains, and start the observer and output at zero. `b0` is used while the config leaves it unset; `buildController` passes the derived one. Must be called before anything else. |
| `calculate(measurement, setpoint)` | One control step. Returns the output. |
| `calculate(measurement, setpoint, appliedOutput)` | One control step, telling the observer what the actuator applied since the last step. Use this when the actuator can apply less than asked. |
| `step(reference, output, refDerivatives)` | One control step with the setpoint's derivatives, for feeding a planned trajectory's velocity and acceleration. `calculate` calls this with none. |
| `reset(measurement)` | Start the observer at a measurement, with the other states and the output at zero. |
| `reset(double[] xhatInit, double uInit)` | Start the observer at a full state and output. |
| `getEstimatedDisturbance()` | The estimated total disturbance. Divide by `b0` to see it in volts. |
| `getEstimatedStates()` | The observer's state: the measurement, its derivatives, then the disturbance. |
| `getB0()`, `getSettlingTime()`, `getBandwidthScale()` | The config's tuning, with its defaults and the `b0` given to `initialize` filling in what it leaves unset. `getSettlingTime()` returns a `Time`. |
| `getBandwidth()`, `getObserverBandwidth()` | The bandwidths that tuning gives. |
| `getLastOutput()` | The last output, volts. |
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
    new TrackingDifferentiator(TrackingDifferentiator.Method.LD, 0.02, Optional.of(new double[] {0.08}));
td.update(setpoint);
double smoothSetpoint = td.getY();
double setpointVelocity = td.getYd();
```

| Method | Parameters (defaults) | Behavior |
| --- | --- | --- |
| `EULER` | `alpha` (0.9), from 0 to 1 | First order smoothing. Larger is smoother and slower. |
| `TOD` | `r` (1), positive, loop units per s² | Bang-bang with an acceleration bound `r`. |
| `LD` | `lambda` (1), seconds, more than half the sample time | Linear, critically damped, with time constant `lambda`. |
| `RED` | `lambda1`, `lambda2` (1, 1), positive | Super-twisting: `lambda2` bounds the change of `yd`, `lambda1` scales the pull of the error. |
| `INTD` | `alpha` (0.5) from 0 to below 1, `beta` and `gamma` (1, 1) in loop units, `r` (1) per second | Saturating: cruises at `r·beta/(1 - alpha)`, then lands like `LD`. |

See [Smoothing the setpoint](#smoothing-the-setpoint-the-tracking-differentiator) for what each
parameter does and how to choose them.

| Method | What it does |
| --- | --- |
| `TrackingDifferentiator(Method, dt, Optional<double[]> params)` | A differentiator with a sample time, seconds. Empty `params` use the defaults. |
| `update(reference)` | Advance one sample. |
| `getY()`, `getYd()`, `getYdd()` | The smooth setpoint and its first and second derivatives. |
| `reset(value)` | Start at a value, at rest. |
| `setParameters(params)` | Change the parameters; invalid ones are rejected and the old ones kept. |
| `setSampleTime(dt)` | Change the sample time. |
| `getMethod()` | The `Method`. |

## C++

The C++ port has the same classes in `yams::math` (`ADRCConfig`, `ADRCController` and
`TrackingDifferentiator`), with the same behavior: the controller reads its config every loop,
the config keeps the gains and works them out again when the settling time, bandwidth scale or
`b0` changes, and the `With` methods refuse a value the controller cannot use. What differs:

```cpp
config.WithMotorGearing(12.0)
    .WithMOI(0.1_kg_sq_m)  // b0 is derived from this
    .WithClosedLoopMaxVoltage(12_V)
    .WithADRCController(yams::math::ADRCConfig{}
        .WithSystemOrder(yams::math::ADRCConfig::SystemOrder::ARM)
        .WithSettlingTime(600_ms));

// Retune, as live tuning does: the motor config's ADRC config is shared with the controller.
config.GetADRCController()->WithSettlingTime(800_ms);
```

- **Errors are exceptions.** A refused value throws `std::invalid_argument`; a built config given a
  different system order throws `std::logic_error`; a missing order or a `b0` that cannot be
  derived throws `exceptions::SmartMotorControllerConfigurationException`.
- **Empty values are `std::optional`.** The tracking differentiator parameters are a
  `std::vector<double>`, empty for the method's defaults.
- **The motor config owns a copy.** `WithADRCController` copies the `ADRCConfig` it is given, and
  `GetADRCController(slot)` returns that copy as a `std::shared_ptr`, shared with the controller.
  Change the config through it, not through the one you passed in. Copying the motor config copies
  each slot's ADRC config deeply. `WithFeedback` or `WithLQR` on the slot removes the ADRC, and
  `WithoutADRCController(slot)` removes it too.
- **Building a controller yourself** takes the config as a `std::shared_ptr`:
  `ADRCConfig::BuildController(config, motor, motorConfig, period)`, or
  `ADRCController{config}` followed by `Initialize(b0, period)`.
- **Names.** The motor config's output limit is `WithClosedLoopMaxVoltage`.
- **System orders.** `SystemOrder` is an `enum class`, and aliases share a value: `ARM` and
  `PIVOT` equal `POSITION`. `ADRCConfig::GetOrder(order)` gives its order, 0 for one chosen when
  built, and `IsChosenWhenBuilt(order)` and `RequiresLinearClosedLoop(order)` tell them apart.
- **Units where the value has one.** `Calculate`, `Step` and `GetLastOutput` use `volt_t`, and
  `GetBandwidth` and `GetObserverBandwidth` use `radians_per_second_t`. The
  measurement, setpoint and `b0` are plain numbers in the loop's units, as in Java.
- **Thread safety.** The config guards its values with a mutex, so live tuning on the robot thread
  and the closed loop thread never see a half-changed config.
- **Moment of inertia.** `WithMOI(...)`, `WithWheelMOI(...)` and an exponential profile with an
  inertia count as configured; `IsMomentOfInertiaConfigured()` tells.

## Things to know

- **One controller per slot serves position and velocity.** Its observer is in the units of the
  setpoint it was last reset for. Switching between a position and a velocity setpoint while
  enabled does not reset it yet, so pick one per mechanism.
- **Simulation needs an inertia**, as with a PID, unless the mechanism builds its own simulation.
- **Motion profiles still apply.** With a trapezoidal or exponential profile, the ADRC follows the
  profile's next position, as a PID does.
- **One `ADRCConfig` per motor controller.** A config holds the period and `b0` of the controller
  built from it, so two motor controllers sharing one config instance share those
  too: the last one built sets them for both. Give each motor controller its own config, or a
  `clone()` of one.
- **Tuning is safe while the loop runs.** The closed loop thread reads the gains once per loop,
  and the config replaces them whole when it works them out again, so a loop never runs with a mix
  of old and new gains.
