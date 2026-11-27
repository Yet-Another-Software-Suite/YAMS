---
icon: chart-line
---

# ADRC and motion profiles

A [motion profile](../../reference/editor/config/motion-profiles.md) turns a step into a trajectory: a setpoint that moves at a limited speed and acceleration. An [ADRC](./) follows one well only if it is fed the profile the right way and its observer knows about the motor's back-EMF. This page explains both. You do not need to do anything: YAMS does this for you when a motor config has a motion profile and an ADRC. **ADRC in YAMS was written by FRC Team 8331.**

Every number here was measured in simulation with one pivot: a NEO on a 12:1 reduction, a 0.1 kg·m² moment of inertia, `SystemOrder.PIVOT` with a 0.3 s settling time, and a trapezoidal profile that moves it 90° in about 0.4 s. At 12:1 this pivot runs a [second order position loop](./#two-kinds-of-position-loop).

## Feeding the profile

### Only the position: the ADRC trails behind

Told only where the profile is, the ADRC treats each setpoint as standing still: it tries to bring the mechanism's speed to zero while the profile is moving. So it trails the profile, by 15° to 32° depending on the motor controller, and catches up after the profile ends. It still arrives within 3° of the goal, so a step response hides the lag, but an arm that has to be somewhere at a given time, such as in step with an intake, is late.

### The next state: the ADRC runs a loop ahead

A closed loop on the SystemCore is given the profile's state one control period ahead, which suits a PID. An ADRC's feedback is much stiffer: here it asks for 400 rotations per second squared of acceleration for each rotation of error. Aiming a whole period ahead while the profile moves asks for about 10 rotations per second squared of extra acceleration at top speed, more than the profile itself uses. Fed that way, the pivot ran ahead of the profile and past the goal, by 3° to 11°.

### Now and next: what YAMS does

YAMS gives the ADRC the profile's state **now**, at the start of the loop, and its state **next**, one period later, as WPILib's feedforwards take them. The ADRC follows the setpoint now, and feeds forward how it moves over the period:

* **A second order loop** uses the profile's speed now, and its acceleration over the period.
* **A first order loop** uses the average speed over the period, which carries it exactly from now to next.

Without a motion profile, both states are the setpoint standing still, so a plain step works as before.

<figure><img src="../../.gitbook/assets/adrc-profile-feed.gif" alt="Animation of three 12:1 pivots following the same trapezoidal profile to 90 degrees, drawn side by side above a graph of their angles. Fed only the next position, the orange pivot lags well behind the dashed profile, by up to 28 degrees, and creeps up to 90 late. Fed the next position and speed, the red pivot runs ahead of the profile and swings past 90 to about 95 degrees before settling. Fed the profile now and next, as YAMS does, the green pivot stays on the dashed profile line the whole way and stops on 90."><figcaption><p>Three ways to feed the same profile. Only now and next follows it and stops on the goal. Made with YAMS's ADRC on a simulated pivot, with the back-EMF modeled in all three.</p></figcaption></figure>

## Modeling the back-EMF

### Why lumping it overshoots

The ADRC's [observer](./#what-is-adrc) estimates the total disturbance: everything that moves the mechanism other than its voltage. For a DC motor that includes its **back-EMF**: as the motor spins faster it generates a voltage against its supply, which brakes it in proportion to its speed.

Lumped into the disturbance, the back-EMF changes as fast as the mechanism's speed does, and the observer can only follow a change so fast. While the pivot speeds up, the observer underestimates the braking; while it slows down at the end of the profile, it overestimates it and keeps pushing. The pivot runs past the goal. On the SPARK MAX and SPARK Flex the push was enough to make it oscillate around the goal after the profile ended.

A kV feedforward does not help. The observer has already lumped the back-EMF into the disturbance and cancels it, so a feedforward cancels it a second time: in testing, that made the overshoot worse, up to 15°.

### What YAMS does

A second order position loop's observer includes the back-EMF in its model. It works out how strong the back-EMF is from the same motor, gearing and moment of inertia that choose the loop's form, and the control law supplies the voltage to overcome it. The disturbance then only holds what the model misses, such as gravity and friction.

`withBackEmfModel(false)` on the `ADRCConfig` turns it off. A first order position loop does not need it, since its input gain already includes the back-EMF.

<figure><img src="../../.gitbook/assets/adrc-backemf.gif" alt="Animation of two 12:1 pivots following the same profile to 90 degrees, one with the back-EMF lumped into the disturbance and one with it modeled. Top right, near the goal, the orange lumped pivot runs past 90 to about 91.7 degrees and drifts back, while the green modeled pivot stops on 90 with the dashed profile. Bottom right, a dashed line shows the motor&#x27;s back-EMF in volts, rising to about 1.9 volts as the pivot speeds up and falling back to zero as it stops. The orange lumped estimate follows it late: below it while the pivot speeds up, above it while the pivot slows down, which keeps pushing the pivot past the goal. The green modeled estimate stays at zero, since the model already accounts for the back-EMF."><figcaption><p>Lumped, the observer's estimate trails the back-EMF, and the pivot runs past the goal. Modeled, there is nothing left to estimate. Made with YAMS's ADRC on a simulated pivot.</p></figcaption></figure>

## What each change does

How far the pivot ran past 90°:

| Fed to the ADRC                           | SPARK MAX | SPARK Flex | Thrifty Nova | A301      |
| ----------------------------------------- | --------- | ---------- | ------------ | --------- |
| The next position only                    | +3.1°     | +1.6°      | +0.2°        | 0.0°      |
| The next position and speed               | +8.0°     | +11.4°     | +6.5°        | +2.8°     |
| Now and next, back-EMF lumped             | +6.7°     | +8.3°      | +1.7°        | +0.4°     |
| **Now and next, back-EMF modeled (YAMS)** | **+0.8°** | **+1.1°**  | **+0.3°**    | **+0.1°** |

With what YAMS does, the pivot stayed within 0.1° to 1.5° of the profile while it moved, against 15° to 25° given only the next position, and every motor controller ended within 0.01° of the goal.

## On a Talon in simulation

A Talon is simulated on CTRE's own real-time thread, whose timing shifts by up to a loop from run to run. Following a profile there, the modeled back-EMF stayed within 2° to 5° of it once that shift is allowed for. But holding a setpoint in a `Pivot` mechanism, a TalonFXS with the back-EMF modeled sometimes settled into a slow oscillation, about 3 Hz, which the lumped observer only rarely did, and far less. No fixed delay, jitter, sensor resolution, misreported voltage or inertia error up to twice the real one reproduced it in a deterministic simulation, where the modeled back-EMF did as well or better, so it is put down to that thread.

{% hint style="info" %}
If a mechanism slowly oscillates around its setpoint while holding, with the back-EMF modeled, try `withBackEmfModel(false)`.
{% endhint %}

## Flywheels

A flywheel's speed loop lumps the back-EMF too, and the back-EMF is the whole voltage a spinning flywheel needs. Modeling it let a flywheel settle up to twice as fast at a settling time of 0.3 s or more, but at 0.15 s on a Talon it overshot by 6% to 7% instead of under 1%. So a `FLYWHEEL` still lumps the back-EMF, and keeps a short settling time.

## Tracking differentiators

A [tracking differentiator](adrc-tracking-differentiators.md) smooths a step setpoint without a motion profile. It hands the ADRC its smoothed setpoint and that setpoint's speed from the same moment, so it has no timing lead, and a second order loop models the back-EMF with one too. YAMS refuses a tracking differentiator together with a motion profile.
