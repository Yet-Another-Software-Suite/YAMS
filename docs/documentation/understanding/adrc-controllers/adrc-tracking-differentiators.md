---
icon: wave-square
---

# Tracking differentiators

A **tracking differentiator** (TD) is an optional filter in front of an [ADRC](./). It turns a step setpoint into a glide, so the ADRC is not asked to be somewhere else at once. **ADRC in YAMS, including its tracking differentiators, was written by FRC Team 8331.**

## What it does

Each loop, the TD takes the raw setpoint and gives the ADRC two things:

* **`y`**, a smooth setpoint that moves toward the raw one instead of jumping.
* **`yd`**, how fast `y` is moving. The ADRC uses it as the setpoint's speed, or on a flywheel, its acceleration, so it neither gets kicked by a step nor trails a setpoint that is moving.

It also works out `ydd`, the second derivative, which the ADRC does not use. It is off by default.

{% tabs %}
{% tab title="Java" %}
{% code title="ADRCArmSubsystem.java (fragment)" %}
```java
new ADRCConfig()
    .withSystemOrder(SystemOrder.ARM)
    .withSettlingTime(Seconds.of(0.3))
    // A 0.08 s time constant: a 90° step is followed in about 0.46 s.
    .withTrackingDifferentiator(TrackingDifferentiator.Method.LD, 0.08)
```
{% endcode %}
{% endtab %}

{% tab title="C++" %}
{% code title="ADRCArmSubsystem.cpp (fragment)" %}
```cpp
adrc.WithSystemOrder(yams::math::ADRCConfig::SystemOrder::ARM)
    .WithSettlingTime(0.3_s)
    // A 0.08 s time constant: a 90° step is followed in about 0.46 s.
    .WithTrackingDifferentiator(yams::math::TrackingDifferentiator::Method::LD, {0.08});
```
{% endcode %}
{% endtab %}
{% endtabs %}

### A TD or a motion profile, not both

A [motion profile](../../reference/editor/config/motion-profiles.md) on the motor config also shapes the setpoint, with real speed and acceleration limits. A TD on top of it would reshape a setpoint the profile already shaped, so YAMS throws a `SmartMotorControllerConfigurationException` when the robot starts with both. Use a TD when the mechanism should not be slammed by a step and has no motion profile.

The TD is set when the robot starts: its method and parameters are not live tunable.

## Units

The TD works in the loop's units: mechanism rotations for an arm or pivot, meters for an elevator, and rotations per second for a flywheel. A parameter in seconds means the same thing on every mechanism. A parameter in loop units changes meaning with them: a `beta` of 1 is one rotation on an arm, but one meter on an elevator.

## The methods

Parameters are passed in order, and any you leave off take their default. The defaults are slow for most mechanisms, so set them.

| Method               | Parameters (default)                                                                                            | How it moves to a new setpoint                                                                                                                                                       |
| -------------------- | --------------------------------------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| `LD`, linear         | `lambda` (1): a time constant in seconds, more than half the control period                                     | Glides there with no overshoot. Reaches 98% in about 5.8 × `lambda`, at a peak speed of about step ÷ (2.7 × `lambda`).                                                               |
| `INTD`, saturating   | `alpha` (0.5): 0 to below 1. `beta` (1) and `gamma` (1): loop units. `r` (1): per second                        | Far from the setpoint it cruises at `r × beta ÷ (1 - alpha)`. Near it, it lands like `LD`, with a natural frequency of `r × √(beta ÷ gamma)` and damping of `0.5 × √(gamma ÷ beta)`. |
| `EULER`, exponential | `alpha` (0.9): 0 to 1, per loop                                                                                 | A first order lag, with a time constant of about one period ÷ (1 - `alpha`). Its `yd` jumps on a step, to (1 - `alpha`) × step ÷ period.                                             |
| `TOD`, time optimal  | `r` (1): an acceleration, loop units per second squared                                                         | Full acceleration toward the setpoint, then full braking.                                                                                                                            |
| `RED`, nonlinear     | `lambda1` (1): how hard the error pulls. `lambda2` (1): how fast `yd` may change, loop units per second squared | A super-twisting filter.                                                                                                                                                             |

## How they compare

Measured on the TD alone at the 20 ms default period, stepping an arm 90° (a quarter rotation), an elevator 1 m and a flywheel 50 rotations per second. Each cell is how long it took to stay within 2% of the step, and how far it overshot.

| Method and parameters         | Arm                | Elevator    | Flywheel        | Notes                                                     |
| ----------------------------- | ------------------ | ----------- | --------------- | --------------------------------------------------------- |
| `LD` 1 (default)              | 5.8 s, 0%          | 5.8 s, 0%   | 5.8 s, 0%       | Far too slow                                              |
| `LD` 0.1                      | 0.52 s, 0%         | 0.52 s, 0%  | 0.52 s, 0%      |                                                           |
| `LD` 0.05                     | 0.22 s, 0%         | 0.22 s, 0%  | 0.22 s, 0%      |                                                           |
| `INTD` 0.5, 1, 1, 1 (default) | 8.1 s, 17%         | 8.1 s, 16%  | Not within 12 s | Overshoots, and crawls on big steps                       |
| `INTD` 0.5, 1, 4, 10          | 1.1 s, 0%          | 1.1 s, 0%   | 2.9 s, 0%       | Cruises at 20 loop units per second                       |
| `INTD` 0.5, 1, 4, 20          | 0.52 s, 0%         | 0.52 s, 0%  | 1.4 s, 0%       | Cruises at 40 loop units per second                       |
| `EULER` 0.9 (default)         | 0.74 s, 0%         | 0.74 s, 0%  | 0.74 s, 0%      | `yd` jumps to 1.25 rotations per second on the arm's step |
| `TOD` 4                       | Never holds 2%, 8% | 1.4 s, 8%   | 6.2 s, 1%       | `yd` chatters at rest                                     |
| `RED` 3, 4                    | 0.40 s, 13%        | 0.76 s, 10% | 5.3 s, 8%       | Overshoots, and `yd` chatters at rest                     |

## Choosing and tuning one

* **Start with `LD`.** It has one parameter, in seconds, that means the same thing on every mechanism, and it never overshoots. Pick `lambda` so 5.8 × `lambda` is about how long a move should take. Then check the mechanism can reach the peak speed, step ÷ (2.7 × `lambda`). A `lambda` of 0.08 s moves an arm 90° in about 0.46 s at up to 1.15 rotations per second, which a NEO on 60:1 (about 1.5 at most) can follow. Lengthen `lambda` if the motor voltage sits at its limit during a move; shorten it if the mechanism waits on the TD.
* **Use `INTD` when step sizes vary a lot,** so big moves cruise at a speed the mechanism can hold while small ones still land quickly. Leave `alpha` at 0.5. Choose `beta` and `r` so the cruise speed, `r × beta ÷ (1 - alpha)`, is a little under the mechanism's top speed, and set `gamma` to 4 × `beta`, which lands without overshoot. Raise `r` to land faster. A smaller `gamma` lands faster but overshoots: at `gamma` = `beta`, by about 16%.
* **Avoid `EULER` on a position loop.** It smooths `y`, but its `yd` still jumps on every step, which is what a TD is meant to prevent.
* **Avoid `TOD` and `RED` at the default period.** They switch direction every loop near the setpoint, so their `yd` chatters at rest and the ADRC passes the chatter to the motor. A large `TOD` `r` never settles.

## Using one on its own

`TrackingDifferentiator` also works without an ADRC, to smooth any setpoint. Give it the method, the sample time in seconds and the parameters, then update it each loop. [ADRCConfig](../../reference/editor/adrc-config.md#trackingdifferentiator-on-its-own) lists its methods.

```java
TrackingDifferentiator td =
    new TrackingDifferentiator(TrackingDifferentiator.Method.LD, 0.02, Optional.of(new double[] {0.08}));
td.update(setpoint);
double smoothSetpoint = td.getY();
double setpointVelocity = td.getYd();
```
