---
icon: car-battery
---

# Battery simulation

YAMS models a single, shared robot battery in simulation instead of letting each mechanism compute its own voltage sag in isolation. Every simulated `SmartMotorController`, whether driven by an `ArmSimSupplier`, `ElevatorSimSupplier`, `DCMotorSimSupplier`, or a hardware wrapper (`SparkWrapper`, `TalonFXWrapper`, `TalonFXSWrapper`, `NovaWrapper`, `A301Wrapper`), reports its current draw into [`BatterySim`](https://javadocs.yassrobotics.com/yams/yams/core/motorcontrollers/simulation/BatterySim.html), which combines every mechanism's draw into one realistic loaded-voltage calculation and sets the simulated battery voltage that every motor then runs from.

{% hint style="info" %}
This is automatic. You do not need to call anything for basic voltage sag under combined load, it happens as soon as a mechanism has a sim supplier attached and `simIterate()`/`SimIterate()` is called each loop, same as [Simulation without a mechanism](../reference/editor/#simulation-without-a-mechanism).
{% endhint %}

{% hint style="info" %}
FIRST does not publish the A301's currents, so its motor model, `YMotors.getA301(1)`, uses the NEO 550's for now (100 A stalled, 1.4 A free). An A301's simulated current draw is an estimate until FIRST publishes them.
{% endhint %}

<figure><img src="../.gitbook/assets/battery-sim-shared.png" alt="An arm, an elevator, a shooter flywheel, an intake roller and a swerve drive with eight motors each send their current draw to BatterySim, which adds them to 150 amps and computes 12.0 volts minus 150 amps times 0.020 ohms equals 9.0 volts, the voltage every motor then runs from"><figcaption><p>Every simulated motor reports into one battery, and every motor runs from the result.</p></figcaption></figure>

## Why this matters

Before, if two mechanisms each set the simulated battery voltage from only their own current draw, whichever mechanism updated last would overwrite the other's contribution, so the simulated battery would never actually see the _combined_ draw of the whole robot. With a shared `BatterySim`, a flywheel spinning up while an elevator is climbing will sag the simulated bus voltage the same way a real battery would under both loads at once, which is exactly the scenario that trips brownouts on a real robot. See [Limiting Power Consumption](../how-to-motor-controllers-and-sensors/limiting-power-consumption.md) for how to guard against that on top of realistic simulation.

{% hint style="warning" %}
**Power consumption is not the only thing that causes brownouts.** The battery simulation only models the current your mechanisms draw. Electrical issues can also cause brownouts and excessive power draw on a real robot, and the simulation cannot see any of them. They include, and are not limited to:

* **Shorts**, such as a pinched wire or a frame short.
* **Incorrect wiring**, such as loose or badly crimped connections, loose battery terminals, or the wrong breaker.
* **Wire limitations**, such as wire that is too thin or too long for the current it carries.

A robot that never browns out in simulation can still brown out for these reasons. If the real robot browns out when the simulation says it should not, check the electrical system before changing current limits.
{% endhint %}

## Battery sag vs battery drain

The battery voltage drops for two different reasons, and the simulation models them separately.

|                  | Battery sag                                                                                                            | Battery drain                                                                                                                                         |
| ---------------- | ---------------------------------------------------------------------------------------------------------------------- | ----------------------------------------------------------------------------------------------------------------------------------------------------- |
| What it is       | The voltage dropping **while current is being drawn**, because of the battery's internal resistance.                   | The battery **emptying** as amp-hours are used up over a match.                                                                                       |
| How fast         | Instant. It follows the current loop by loop.                                                                          | Slow. It builds up over minutes.                                                                                                                      |
| Does it recover? | Yes. The voltage comes back as soon as the load stops.                                                                 | No. It stays until the battery is recharged.                                                                                                          |
| What it changes  | The loaded voltage only.                                                                                               | The resting voltage goes down and the internal resistance goes up.                                                                                    |
| In YAMS          | Always on. See [How each mechanism affects the battery](battery-simulation.md#how-each-mechanism-affects-the-battery). | Off until you call `BatterySim.enableDischarge(...)`. See [Realistic discharge over a match](battery-simulation.md#realistic-discharge-over-a-match). |

The two add up. A drained battery starts from a lower resting voltage and has more internal resistance, so the same current sags it further. That is why a move that is fine at the start of a match can brown out at the end of it.

## How each mechanism affects the battery

The loaded voltage is the resting voltage minus the total current times the battery's internal resistance:

`battery voltage = resting voltage - (sum of every motor's supply current) x internal resistance`

With the default 12 V and 20 milliohm battery, every 50 A of total draw costs 1 V.

<figure><img src="../.gitbook/assets/battery-sim-load.gif" alt="Animation of four current bars for a swerve drive, shooter, elevator and arm and a graph of battery voltage. The voltage dips when the swerve drive launches, dips again when the shooter spins up, and drops lowest, to about 8.8 volts, when the elevator and arm move while the drive is pushing"><figcaption><p>A few seconds of a match: the voltage follows the total, and it is lowest when the peaks line up.</p></figcaption></figure>

Mechanisms do not load the battery evenly. Each one has moments when it draws a lot and long stretches when it draws very little.

<figure><img src="../.gitbook/assets/battery-sim-mechanisms.png" alt="Six sketches of current over time. Arm: spikes when it starts and stops, then a steady draw to hold against gravity. Elevator: largest going up, steady to hold a height. Flywheel, shooter, rollers: very large while spinning up, small at speed, a bump on every shot. Pivot, turret, wrist: short bursts when starting and stopping. Swerve drive: eight motors, usually the largest draw. All together: the draws add up and overlapping peaks cause the deepest sag"><figcaption><p>When each kind of mechanism draws current.</p></figcaption></figure>

| Mechanism                               | Motors that report             | Draws the most when                                          | What changes it                                                  |
| --------------------------------------- | ------------------------------ | ------------------------------------------------------------ | ---------------------------------------------------------------- |
| `Arm`                                   | 1 or 2                         | Starting and stopping a move, and holding away from vertical | Arm length and mass (MOI), gearing, motion profile acceleration  |
| `Elevator`                              | 1 to 4                         | Accelerating upward, then a steady draw to hold a height     | Carriage mass, drum radius, gearing, motion profile acceleration |
| `FlyWheel` (shooters, rollers, intakes) | 1 to 4                         | Spinning up from rest, and recovering after each shot        | Wheel MOI, target speed, stator current limit                    |
| `Pivot` (turrets, wrists)               | 1                              | Short bursts when starting and stopping                      | MOI, motion profile acceleration                                 |
| `SwerveDrive`                           | 8: four drive and four azimuth | Accelerating, changing direction, and pushing                | Robot mass, drive gearing, drive stator current limit            |

### Swerve drives

A swerve drive is not one load, it is eight. Each module's drive motor and azimuth motor is its own `SmartMotorController`, and each one reports its own supply current to `BatterySim` when `SwerveDrive.simIterate()` runs. Nothing extra is needed to include the drivetrain.

* The **drive motors** dominate. Four of them accelerating a full weight robot is normally the largest draw on the robot.
* The **azimuth motors** draw short bursts when the modules change heading, and little while driving straight.
* Because the drive is the largest load, it also sets the voltage every other mechanism gets. A shooter that reaches its speed with the robot parked can fall short while the robot is accelerating.

{% hint style="success" %}
To keep the peaks from lining up, set stator and supply current limits on the biggest loads and stagger the moves that can wait. See [Limiting Power Consumption](../how-to-motor-controllers-and-sensors/limiting-power-consumption.md).
{% endhint %}

## Accurate current draw starts with an accurate MOI

`BatterySim` can only sag voltage as realistically as the current draw it's fed, and that current draw comes straight out of each mechanism's physics simulation (`SingleJointedArmSim`/`DCMotorSim`/`ElevatorSim`). Those models derive current from the torque needed to produce a given acceleration, so if the moment of inertia doesn't match your real mechanism, the simulated current, and therefore the simulated voltage sag, won't either. A default/guessed MOI tends to make mechanisms look artificially light, understating both acceleration current spikes and how much they actually sag the bus.

Set a real MOI on the `SmartMotorControllerConfig` with `.withMomentOfInertia(Distance, Mass)` (estimated from a simple rod/arm model) or `.withMomentOfInertia(MomentOfInertia)` (a known/measured value), see [MOI](turrets-wrists.md#moment-of-inertia) for the full explanation and an example.

{% hint style="info" %}
This is the same MOI used for PID/feedforward tuning in simulation, so getting it right pays off twice: more realistic control-loop behavior _and_ more realistic battery sag under load.
{% endhint %}

## Realistic discharge over a match

By default the simulated battery holds a constant nominal voltage and resistance (12V / 20 mΩ), enough to model voltage sag under instantaneous load, but not a battery getting weaker over the course of a match. Call `BatterySim.enableDischarge(...)` to layer state-of-charge modeling on top: as amp-hours are drawn from the battery, its open-circuit voltage droops along a discharge curve and its internal resistance rises, both becoming more pronounced as the battery empties, the same way a real lead-acid FRC battery behaves late in a match. The amp-hours drawn are also derated by discharge rate, so heavy sustained current drains the modeled capacity faster than the same amp-hours drawn slowly, see [Capacity derating](battery-simulation.md#capacity-derating) below.

<figure><img src="../.gitbook/assets/battery-sim-discharge.png" alt="Two graphs of the default discharge model. Resting voltage against state of charge falls slowly from 12.9 volts at full to 12.0 volts at 20 percent, then steeply to 9 volts when empty. Usable capacity against sustained current falls from 100 percent at 0.9 amps to 60 percent at 54 amps. Internal resistance rises from 20 milliohms when full to 60 milliohms when empty"><figcaption><p>The default discharge curves.</p></figcaption></figure>

To turn it on, reset it between runs or read the state of charge, see [How do I enable battery discharge simulation?](../how-to-simulation-and-logging/how-do-i-enable-battery-discharge-simulation.md).

{% hint style="warning" %}
Discharge only affects simulation. On a real robot `BatterySim` is never consulted, the SystemCore reports the actual battery voltage from hardware.
{% endhint %}

## Custom discharge curves

`enableDischarge(...)` sags voltage along a curve tuned for a typical FRC sealed lead-acid battery, flat through most of the charge, then dropping off quickly near depletion. Not every sealed lead-acid battery you want to model behaves exactly that way, so call `BatterySim.replaceSOCInterpolation(...)` **before** `enableDischarge(...)` to swap in your own curve.

Reach for this when:

* **You're simulating a well-used competition battery**, which sags earlier and harder than a fresh one, a flatter, lower curve reproduces "that one tired battery" instead of assuming every match starts fresh.
* **You have measured data**, e.g. from putting a real battery on a load tester, feeding in the actual voltage-vs-state-of-charge points gives the most accurate brownout predictions for that specific battery.

The code is in [How do I enable battery discharge simulation?](../how-to-simulation-and-logging/how-do-i-enable-battery-discharge-simulation.md#replace-the-state-of-charge-curve-optional).

{% hint style="info" %}
Keys and values should span the full `[0, 1]` state-of-charge range. Querying outside the range you defined just returns the nearest endpoint's voltage instead of extrapolating, so a table missing the low or high end will not sag realistically there.
{% endhint %}

## Capacity derating

Sealed lead-acid batteries are far less [coulombically efficient](https://en.wikipedia.org/wiki/Coulombic_efficiency) than lithium chemistries at high discharge rates. Coulombic efficiency is how much of the charge stored in a battery you actually get back out of it. This is known as the [Peukert effect](https://en.wikipedia.org/wiki/Peukert's_law): the faster you discharge a lead-acid battery, the fewer total amp-hours it actually delivers before it's empty. A battery rated for 18 Ah at a light 0.9A discharge (the 20-hour rate its label is based on) might only deliver \~11 Ah at a sustained 54A draw, well within normal FRC match currents.

`enableDischarge(...)` accounts for this by derating the amp-hours consumed against a discharge-current → capacity-fraction curve, so a match spent at high sustained current drains the modeled state of charge faster than the same amp-hours drawn gently would. The default curve is averaged from discharge testing across five FRC battery manufacturers, see [Detailed FRC Battery Comparison for 2026](https://www.chiefdelphi.com/t/detailed-frc-battery-comparison-for-2026/508077). Call `BatterySim.replaceCapacityDerating(...)` **before** `enableDischarge(...)` if you have measured discharge-rate-vs-capacity data for your own battery.

The code is in [How do I enable battery discharge simulation?](../how-to-simulation-and-logging/how-do-i-enable-battery-discharge-simulation.md#replace-the-capacity-derating-curve-optional).

## Related pages

* [How do I enable battery discharge simulation?](../how-to-simulation-and-logging/how-do-i-enable-battery-discharge-simulation.md)
* [Limiting Power Consumption](../how-to-motor-controllers-and-sensors/limiting-power-consumption.md)
* [Simulation without a mechanism](../reference/editor/#simulation-without-a-mechanism)
* [Simulation Only PID + FeedForward](simulation-only-pid-+-feedforward.md)
* [MOI](turrets-wrists.md#moment-of-inertia)
