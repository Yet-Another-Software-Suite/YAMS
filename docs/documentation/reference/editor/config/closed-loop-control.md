---
icon: arrows-spin
---

# Closed loop control

Closed loop control is what takes a mechanism to a setpoint and holds it there. These options need [`withControlMode(ControlMode.CLOSED_LOOP)`](motor-basics.md#withcontrolmode), which is the default.

{% hint style="warning" %}
Not every option on this page is supported on every motor controller. Each option has a table showing where it works, and an option a controller cannot do throws a `SmartMotorControllerConfigurationException` or an `IllegalArgumentException` at startup. Each option says which. The [config overview](./#not-every-option-works-on-every-motor-controller) lists them all together.
{% endhint %}

For how to find good values, see [live tuning](../../../how-to-tuning/live-tuning/).

## `withClosedLoopController`

Sets the PID gains.

<figure><img src="../../../.gitbook/assets/smc-pid.png" alt="A graph of position against time moving to a setpoint with three sets of gains: a low kP that is slow, a high kP that overshoots, and kP with kD that is quick and settled. kP pushes harder the further away, kI pushes harder the longer it is off, kD eases off as it gets close. The input is rotations, or meters for a distance mechanism, and the output is volts."><figcaption><p>What the gains do to a move.</p></figcaption></figure>

```java
.withClosedLoopController(4, 0, 0.2)                 // kP, kI, kD
.withClosedLoopController(new PIDController(4, 0, 0.2))
```

Write gains in volts per mechanism rotation, or volts per meter for a [distance based controller](closed-loop-control.md#withlinearclosedloopcontroller). YAMS converts them for each motor controller, so the same numbers behave the same on every vendor: see [Under the hood](closed-loop-control.md#under-the-hood) if you want the details.

Passing an `LQRController` instead replaces the PID with an LQR: see [LQR controllers](../../../understanding/lqr-controllers.md). To use an ADRC, see [`withADRCController`](closed-loop-control.md#withadrccontroller).

**Supported on every motor controller.** Every controller accepts PID gains. An [LQR controller](../../../understanding/lqr-controllers.md) or an [ADRC](../../../understanding/adrc-controllers/) always runs on the SystemCore, on every controller, and so does every loop of an A301, which has no closed loop gains of its own.

| TalonFX | TalonFXS | SPARK MAX / Flex | Thrifty Nova           | A301                         |
| ------- | -------- | ---------------- | ---------------------- | ---------------------------- |
| ✅       | ✅        | ✅                | ✅ PID runs on the Nova | ✅ PID runs on the SystemCore |

## `withADRCController`

Replaces the slot's PID, LQR and feedforward with an ADRC, written by FRC Team 8331. You give it the kind of mechanism and a settling time; it works out the rest from the motor, the gearing and the moment of inertia. See [ADRC controllers](../../../understanding/adrc-controllers/) for how it works, [ADRCConfig](../adrc-config.md) for every setting, and [How do I use ADRC instead of PID?](../../../how-to-tuning/how-do-i-use-adrc.md) to set one up.

```java
.withADRCController(new ADRCConfig()
    .withSystemOrder(SystemOrder.ARM)
    .withSettlingTime(Seconds.of(0.3)))
```

**Supported on every motor controller.** It always runs on the SystemCore, so `withClosedLoopControllerMaximumVoltage` and `withClosedLoopRampRate` apply to it on every controller.

| TalonFX | TalonFXS | SPARK MAX / Flex | Thrifty Nova | A301 |
| ------- | -------- | ---------------- | ------------ | ---- |
| ✅       | ✅        | ✅                | ✅            | ✅    |

### Starting values

These are only places to start, so the first move is gentle and you can see which way to go. Every mechanism ends up somewhere else. Tune the [feedforward](feedforward.md#withfeedforward) first, with these PID gains at 0.

| Mechanism          | Start `kP` at | Which means                                                | Start `kI`, `kD` at |
| ------------------ | ------------- | ---------------------------------------------------------- | ------------------- |
| Arm, pivot, turret | `10`          | 10 V per rotation of error, about 0.3 V at 10° off         | `0`, `0`            |
| Elevator           | `10`          | 10 V per meter of error, 1 V at 10 cm off                  | `0`, `0`            |
| Flywheel, roller   | `0.1`         | 0.1 V per rotation per second of error, 1 V at 600 RPM off | `0`, `0`            |

Graph the position (or velocity) against the setpoint and make small moves:

* **It stops short, or creeps up slowly:** `kP` is too low. Double it and try again.
* **It overshoots and rings:** `kP` is too high. Go back about a third, then add `kD`, starting around a tenth of `kP`, until the overshoot goes.
* **It stops a little short every time, even with a good `kP`:** check `kG` and `kS` in the [feedforward](feedforward.md) before reaching for `kI`.

The Arm tutorial starts its light, simulated arm at a `kP` of 50; either is a fine place to start because you double or cut it from there. [Step 5 of the Arm tutorial](../../../tutorials/arm/tune.md) walks through this, and [Tuning order](../../../how-to-tuning/live-tuning/#tuning-order) has the order for each mechanism.

### Slots

Each call can name a slot, so one motor can hold several sets of gains and switch between them.

<figure><img src="../../../.gitbook/assets/smc-slots.png" alt="Four slots, SLOT_0 to SLOT_3, each holding its own PID gains and feedforward, for example one for holding nothing, one for holding a game piece and one for climbing. SLOT_0 is in use. SLOT_3 is not available on TalonFX or TalonFXS. motor.setClosedLoopSlot switches slot at once."><figcaption><p>Four slots, each with its own gains and feedforward.</p></figcaption></figure>

```java
.withClosedLoopController(4, 0, 0, ClosedLoopControllerSlot.SLOT_0)
.withClosedLoopController(6, 0, 0, ClosedLoopControllerSlot.SLOT_1)
// later, while running
motor.setClosedLoopSlot(ClosedLoopControllerSlot.SLOT_1);
```

**Not supported everywhere.** Selecting `SLOT_3` on a TalonFX or TalonFXS throws an `IllegalArgumentException`, and a feedforward configured for `SLOT_3` on a TalonFX throws a `SmartMotorControllerConfigurationException`. A Thrifty Nova running the closed loop itself has two slots: gains or a feedforward in `SLOT_2` or `SLOT_3` throw a `SmartMotorControllerConfigurationException`.

| TalonFX                 | TalonFXS                | SPARK MAX / Flex | Thrifty Nova                                                                                                   | A301 |
| ----------------------- | ----------------------- | ---------------- | -------------------------------------------------------------------------------------------------------------- | ---- |
| ⚠️ `SLOT_0` to `SLOT_2` | ⚠️ `SLOT_0` to `SLOT_2` | ✅                | ⚠️ `SLOT_0` and `SLOT_1`, unless the loop [runs on the SystemCore](closed-loop-control.md#where-the-loop-runs) | ✅    |

## `withClosedLoopTolerance`

How close to the setpoint counts as being there.

<figure><img src="../../../.gitbook/assets/smc-tolerance.png" alt="A graph of position moving to a setpoint with a shaded band of plus and minus the tolerance around it. Once the position stays inside the band the mechanism counts as at the setpoint."><figcaption><p>The tolerance is a band around the setpoint.</p></figcaption></figure>

```java
.withClosedLoopController(4, 0, 0)
.withClosedLoopTolerance(Degrees.of(1))
```

Call it after `withClosedLoopController`: it is set on the PID controllers that exist at that moment, and throws a `SmartMotorControllerConfigurationException` if there are none.

**Not supported everywhere.** It throws a `SmartMotorControllerConfigurationException` on a TalonFX or TalonFXS.

| TalonFX | TalonFXS | SPARK MAX / Flex                                                           | Thrifty Nova                                                               | A301                                                                       |
| ------- | -------- | -------------------------------------------------------------------------- | -------------------------------------------------------------------------- | -------------------------------------------------------------------------- |
| ❌       | ❌        | ⚠️ not with an [LQR controller](../../../understanding/lqr-controllers.md) | ⚠️ not with an [LQR controller](../../../understanding/lqr-controllers.md) | ⚠️ not with an [LQR controller](../../../understanding/lqr-controllers.md) |

## `withContinuousWrapping`

Tells the controller that the two ends of the range are the same place, so it takes the short way round.

<figure><img src="../../../.gitbook/assets/smc-continuous-wrapping.png" alt="Two dials going from 170 degrees to minus 170 degrees. Without continuous wrapping the mechanism goes 340 degrees the long way round. With continuous wrapping from minus 180 to 180 degrees it goes 20 degrees across the seam, the short way."><figcaption><p>From 170° to -170°, without and with continuous wrapping.</p></figcaption></figure>

```java
.withClosedLoopController(50, 0, 0.5)
.withContinuousWrapping(Degrees.of(-180), Degrees.of(180))
```

It needs a closed loop controller first, and cannot be combined with [soft limits](limits-and-protection.md#withsoftlimits) or a distance based controller, because a mechanism with ends cannot wrap. Setting both throws a `SmartMotorControllerConfigurationException`, in either order.

It also sets the range an absolute external encoder reads, as [`withWrapping`](encoders.md#withwrapping) does. A mechanism with ends whose absolute encoder must read across zero, such as an arm, uses `withWrapping` instead: the encoder reads the same range, but the closed loop never takes the mechanism the other way round, and soft limits still work.

**Not the same everywhere.** A Thrifty Nova wraps continuously itself over any one rotation range, such as -180° to 180°, of an absolute encoder closing its loop and mounted 1:1 on the mechanism. It also wraps its motor's own encoder or a quadrature encoder: YAMS sends each target as the equivalent nearest the mechanism, so the Nova takes the short way. A Nova swerve azimuth seeded from a `CANcoder` runs entirely on the Nova, like a SPARK MAX with one. Only an absolute encoder geared to the mechanism runs the closed loop [on the SystemCore](closed-loop-control.md#where-the-loop-runs): it wraps once per encoder rotation, inside the mechanism's range. An encoder's own range has its own limits: see [`withWrapping`](encoders.md#withwrapping).

| TalonFX | TalonFXS | SPARK MAX / Flex | Thrifty Nova                                                                                                               | A301 |
| ------- | -------- | ---------------- | -------------------------------------------------------------------------------------------------------------------------- | ---- |
| ✅       | ✅        | ✅                | ✅ on the Nova; an absolute encoder geared to the mechanism [on the SystemCore](closed-loop-control.md#where-the-loop-runs) | ✅    |

## `withClosedLoopControllerMaximumVoltage`

Caps the voltage the closed loop controller may ask for.

<figure><img src="../../../.gitbook/assets/smc-max-voltage.png" alt="A graph of closed loop output against time. The controller asks for nearly 12 volts at the start of a move, but the motor only gets the 6 volt maximum until the request drops below it."><figcaption><p>A 6 V maximum on a move that asked for more.</p></figcaption></figure>

```java
.withClosedLoopControllerMaximumVoltage(Volts.of(6))
```

**Not supported everywhere.** On a TalonFX or TalonFXS it is applied as the peak forward and reverse voltage. On a SPARK or Thrifty Nova it throws a `SmartMotorControllerConfigurationException` unless the loop runs on the SystemCore. An A301's loop always runs on the SystemCore, so it always works there.

| TalonFX | TalonFXS | SPARK MAX / Flex                                                                                                                                           | Thrifty Nova                                                                                                                                                                                                                             | A301 |
| ------- | -------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ---- |
| ✅       | ✅        | ⚠️ only with an exponential profile, an [LQR controller](../../../understanding/lqr-controllers.md) or an [ADRC](../../../understanding/adrc-controllers/) | ⚠️ only when the loop runs on the SystemCore, such as with an exponential profile, a velocity trapezoidal profile, an [LQR controller](../../../understanding/lqr-controllers.md) or an [ADRC](../../../understanding/adrc-controllers/) | ✅    |

## Where the loop runs

Some combinations run on the motor controller, at 1 kHz, and some on the SystemCore, every 20 ms by default. It matters for a few options, and YAMS raises an alert when the loop is on the SystemCore. `getClosedLoopControllerPeriod()` also tells you: 1 ms on the motor controller, the closed loop control period on the SystemCore. The figure does not show the A301: it has no closed loop of its own, so PID, both profiles and LQR all run on the SystemCore. An ADRC runs on the SystemCore on every controller.

A Thrifty Nova also runs the loop on the SystemCore when the Nova can't run it: a feedforward without a trapezoidal profile (the Nova only applies a feedforward in a motion profile), an absolute encoder closing the loop over a range other than 0° to 360° without continuous wrapping, or continuous wrapping of an absolute encoder geared to the mechanism (see [`withContinuousWrapping`](closed-loop-control.md#withcontinuouswrapping)).

<figure><img src="../../../.gitbook/assets/smc-where-it-runs.png" alt="A table of where the closed loop runs. TalonFX and TalonFXS run PID and profiles on the controller and LQR on the SystemCore. SPARK MAX and SPARK Flex run PID and trapezoidal profiles on the controller, and exponential profiles and LQR on the SystemCore. The Thrifty Nova runs PID and trapezoidal position profiles on the controller, and exponential profiles, velocity profiles, LQR and a feedforward without a profile on the SystemCore. Below it, three timelines over 240 milliseconds show how often each loop runs: on the controller every 1 millisecond, which is 1 kilohertz and 240 runs; on the SystemCore every 20 milliseconds by default, which is 12 runs; and on the SystemCore at 10 milliseconds, which is 24 runs. withClosedLoopControlPeriod only changes the SystemCore loop. The period of the loop on the controller cannot be changed."><figcaption><p>Where the closed loop runs, and how often.</p></figcaption></figure>

### PID gains do not carry over between the two

PID gains tuned with the loop on the motor controller will never be the same as gains tuned with the loop on the SystemCore, even on the same mechanism. The reason is the closed loop control period:

* **On the motor controller** the loop runs at 1 kHz, once every millisecond, right next to the motor and its encoder. That is 20 runs for every one run of the SystemCore loop at its default period. This period is fixed by the motor controller and cannot be changed.
* **On the SystemCore** the loop runs once every control period, 20 ms by default, and each measurement and each output also has to cross the CAN bus.

A slower loop reacts later to every error, so it cannot use gains as high as a fast one before it starts to oscillate. `kD` and `kI` are affected the most, because they depend directly on the time between two runs of the loop.

What this means in practice:

* **Tune again whenever the loop moves.** On a SPARK or Thrifty Nova that happens when you switch between a trapezoidal profile, which runs on the controller, and an exponential profile or an [LQR controller](../../../understanding/lqr-controllers.md), which run on the SystemCore. Moving a mechanism to a different brand of motor controller can move the loop too.
* **Tune again after changing** [**`withClosedLoopControlPeriod`**](closed-loop-control.md#withclosedloopcontrolperiod)**.** A different period is a different loop.
* **Do not copy gains from another team or another robot** without checking where their loop ran and how fast.

### `withClosedLoopControlPeriod`

Changes how often the SystemCore loop runs. The default is 20 ms. It does not change a loop that runs on the motor controller: that period is fixed at 1 ms (1 kHz).

```java
.withClosedLoopControlPeriod(Milliseconds.of(10))
```

**Not supported everywhere.** Where it is not supported it throws an `IllegalArgumentException` on a TalonFX or TalonFXS, and a `SmartMotorControllerConfigurationException` on a SPARK or Thrifty Nova. Outside closed loop mode it throws an `IllegalArgumentException` on a Nova or an A301.

| TalonFX | TalonFXS | SPARK MAX / Flex                                                                                      | Thrifty Nova                                                                                                                                                                        | A301                        |
| ------- | -------- | ----------------------------------------------------------------------------------------------------- | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | --------------------------- |
| ❌       | ❌        | ⚠️ only with an exponential profile or an [LQR controller](../../../understanding/lqr-controllers.md) | ⚠️ only when the loop runs on the SystemCore, such as with an exponential profile, a velocity trapezoidal profile or an [LQR controller](../../../understanding/lqr-controllers.md) | ⚠️ only in closed loop mode |

## `withLinearClosedLoopController`

Chooses whether the controller works in rotations or in meters.

<figure><img src="../../../.gitbook/assets/smc-linear-units.png" alt="Two panels. Angle based, the default: error is measured in rotations, kP is in volts per rotation and profile limits are in rotations per second, for arms, pivots, turrets and flywheels. Distance based: error in meters, kP in volts per meter and profile limits in meters per second, for elevators and other linear mechanisms."><figcaption><p>The units the gains and limits are in.</p></figcaption></figure>

```java
.withLinearClosedLoopController(true)  // gains and limits in meters
.withLinearClosedLoopController(false) // gains and limits in rotations
```

It turns on by itself when you give a distance profile, a distance exponential profile or an `ElevatorFeedforward`, and it only takes effect when a [circumference](gearing-and-units.md#withmechanismcircumference) is set. [How do I turn off linear closed loop control?](../../../how-to-motor-controllers-and-sensors/how-to-disable-linear-closed-loop-control.md) explains when to turn it off.

**Supported on:** every supported motor controller.

## Under the hood

You do not need this section to use YAMS. It explains what YAMS does with your gains, for when you compare them with a vendor's documentation or another team's numbers.

### How your gains reach the motor controller

You write every gain once, in mechanism units: volts per mechanism rotation, or per meter for a [distance based controller](closed-loop-control.md#withlinearclosedloopcontroller). Every [feedforward](feedforward.md) uses the same units, including an `ArmFeedforward`. Each motor controller wants its gains in its own units, so YAMS converts them when it applies the config, and again every time you change a gain while [live tuning](../../../how-to-tuning/live-tuning/).

<figure><img src="../../../.gitbook/assets/smc-gain-overview.png" alt="Your gains, in YAMS units of volts per mechanism rotation or meter, go to three motor controllers. TalonFX and TalonFXS: the sensor to mechanism ratio is the gear ratio, so the Talon already works in mechanism rotations; a linear mechanism&#x27;s gains are divided by the rotations per meter; kS and kG are volts and do not change; Motion Magic limits go from meters to rotations; gravity is Arm_Cosine or Elevator_Static. SPARK MAX and Flex: gains are per rotation of its encoder and output a duty cycle, so they are divided by 12 V and the gear ratio, velocity gains by 60 more for RPM, kI is multiplied by 1 ms and kD divided by 1 ms, kV and kA are volts per RPM of its encoder, and an arm&#x27;s kG becomes kCos with a kCosRatio of 1 over the ratio. Thrifty Nova: the sensor to mechanism ratio is the gear ratio, so it already works in mechanism rotations; a linear mechanism&#x27;s gains are divided by the rotations per meter; the feedforward goes in its motion profile, with cosine gravity for an arm; exponential profiles and LQR run on the SystemCore."><figcaption><p>Where your gains go.</p></figcaption></figure>

#### Dimensional analysis

The conversion is dimensional analysis. A gain is a number with units, like 4 V per mechanism rotation. To change its units, multiply it by fractions that are each worth exactly 1, such as 1 mechanism rotation over 12 motor rotations on a 12:1 gearbox. A unit that appears on both the top and the bottom cancels. Once only the units the motor controller wants are left, the number in front is its gain.

{% tabs %}
{% tab title="SPARK MAX / Flex" %}
A SPARK's closed loop works on its own encoder, not on the mechanism. Its PID output is a duty cycle from -1 to 1, not volts. So every PID gain loses the gear ratio and the 12 V, and a velocity gain also changes from rotations per second to RPM.

<figure><img src="../../../.gitbook/assets/smc-gain-units-spark.gif" alt="An animation of five unit conversions for a 12 to 1 SPARK. Position kP: 4 volts per mechanism rotation, times 1 mechanism rotation over 12 motor rotations, times 1 duty over 12 volts. Mechanism rotations cancel, then volts cancel, leaving 0.0278 duty cycle per motor rotation. Elevator kP: 40 volts per meter, times 0.2 meters per mechanism rotation, then the same gear ratio and voltage, leaving 0.0556 duty cycle per motor rotation. Velocity kP: 0.5 volts per mechanism rotation per second, times 1 mechanism rotation per second over 12 motor rotations per second, times 1 motor rotation per second over 60 RPM, times 1 duty over 12 volts, leaving 5.79e-05 duty cycle per RPM. kD: 0.2 volt seconds per mechanism rotation, through the same gear ratio and voltage, times 1 loop over 0.001 seconds, so seconds cancel and it leaves 1.39 duty cycle loops per motor rotation. kV: 1.0 volt per mechanism rotation per second, through the gear ratio and 60 RPM, leaving 0.00139 volts per RPM, because feedforwards stay in volts."><figcaption><p>Converting a 12:1 mechanism's gains for a SPARK, one unit at a time.</p></figcaption></figure>

| Gain                 | YAMS units                                      | SPARK units                     | YAMS does               |
| -------------------- | ----------------------------------------------- | ------------------------------- | ----------------------- |
| `kP`, position       | V per mechanism rotation (or meter)             | duty cycle per encoder rotation | ÷ (12 V × ratio)        |
| `kP`, velocity       | V per mechanism rotation (or meter) per second  | duty cycle per encoder RPM      | ÷ (12 V × ratio × 60)   |
| `kI`                 | `kP`'s units per second                         | `kP`'s units per 1 ms loop      | × 0.001 s, then as `kP` |
| `kD`                 | `kP`'s units × seconds                          | `kP`'s units × 1 ms loops       | ÷ 0.001 s, then as `kP` |
| `kS`, `kG`           | V                                               | V                               | nothing                 |
| `kV`                 | V per mechanism rotation (or meter) per second  | V per encoder RPM               | ÷ (ratio × 60)          |
| `kA`                 | V per mechanism rotation (or meter) per second² | V per encoder RPM per second    | ÷ (ratio × 60)          |
| Profile velocity     | mechanism rotations (or meters) per second      | encoder RPM                     | × ratio × 60            |
| Profile acceleration | mechanism rotations (or meters) per second²     | encoder RPM per second          | × ratio × 60            |

* **The ratio** is the gearing between the mechanism and the encoder the SPARK uses: your [gearing](gearing-and-units.md), or the [external encoder's gearing](encoders.md#withexternalencodergearing) when the SPARK uses an external encoder for feedback.
* **The 12 V** is your [voltage compensation](limits-and-protection.md#withvoltagecompensation) if you set one, and 12 V if you do not. The SPARK's PID output is a fraction of full output, from -1 to 1, in every closed loop control type YAMS sends: position, velocity and both MAXMotion types. With voltage compensation, full output is the compensation voltage, so the gains are exact. Without it, full output is the battery's voltage, so at 11 V the PID pushes about 8% less than you tuned for.
* **The feedforward is in volts**, not a fraction of full output. The SPARK's `kS`, `kV`, `kA`, `kG` and `kCos` are in volts, and YAMS sends its own feedforward with each setpoint in volts (`ArbFFUnits.kVoltage`), so the feedforward does not change with the battery's voltage.
* **An elevator, or any distance based controller,** has every gain per meter, as on a Talon: `kP`, `kI`, `kD`, `kV` and `kA`, and the profile's limits. `kS` and `kG` are volts either way. YAMS first divides a gain per meter by the mechanism rotations per meter (multiplies, for a profile limit), then converts the result as in the table. An `ElevatorFeedforward` or [`withLinearClosedLoopController`](closed-loop-control.md#withlinearclosedloopcontroller) makes the controller distance based.
* **Position and velocity need different gains.** A SPARK measures position error in rotations and velocity error in RPM, so YAMS writes the position or velocity version of your gains whenever the type of setpoint changes. You still tune only one set.
* **An arm's `kG`** goes to the SPARK's `kCos`, with a `kCosRatio` of 1 over the ratio so the SPARK can turn its encoder's rotations into the arm's angle.
* **In simulation**, REVLib's SPARK applies `kI` and `kD` per second instead of per loop, so YAMS skips the 0.001 s step there.
{% endtab %}

{% tab title="TalonFX / TalonFXS" %}
YAMS sets the Talon's `SensorToMechanismRatio` to your gear ratio, so the Talon already measures in mechanism rotations and works in volts. An angle based gain goes to the Talon unchanged. Only a [distance based controller](closed-loop-control.md#withlinearclosedloopcontroller) is converted, from meters to mechanism rotations.

<figure><img src="../../../.gitbook/assets/smc-gain-units-talon.gif" alt="An animation of three unit conversions for an elevator on a Talon whose drum moves 0.2 meters per mechanism rotation. kP: 40 volts per meter times 0.2 meters per mechanism rotation; meters cancel, leaving 8 volts per mechanism rotation. kV: 8 volts per meter per second times 0.2 meters per mechanism rotation, leaving 1.6 volts per mechanism rotation per second. Motion Magic cruise velocity: 1.5 meters per second times 1 mechanism rotation over 0.2 meters, leaving 7.5 mechanism rotations per second."><figcaption><p>Converting an elevator's gains for a Talon, one unit at a time.</p></figcaption></figure>

| Gain                                | YAMS units                                                                     | Talon units                             | YAMS does                                                                                       |
| ----------------------------------- | ------------------------------------------------------------------------------ | --------------------------------------- | ----------------------------------------------------------------------------------------------- |
| `kP`, `kI`, `kD`                    | V per mechanism rotation (or meter)                                            | V per mechanism rotation                | ÷ mechanism rotations per meter when distance based                                             |
| `kS`, `kG`                          | V                                                                              | V                                       | nothing, and sets `GravityType` to `Arm_Cosine` for an arm or `Elevator_Static` for an elevator |
| `kV`, `kA`                          | V per mechanism rotation (or meter) per second (²)                             | V per mechanism rotation per second (²) | ÷ mechanism rotations per meter when distance based                                             |
| Motion Magic velocity, acceleration | mechanism rotations (or meters) per second (²)                                 | mechanism rotations per second (²)      | × mechanism rotations per meter when distance based                                             |
| Motion Magic Expo `kV`, `kA`        | from the [exponential profile](../../../understanding/exponential-profiles.md) | V per mechanism rotation per second (²) | ÷ mechanism rotations per meter when distance based                                             |

With an [external encoder](encoders.md#withexternalencoder), the `SensorToMechanismRatio` is the encoder's gearing and the `RotorToSensorRatio` is the rest of the gearing, so the gains are still per mechanism rotation.
{% endtab %}

{% tab title="Thrifty Nova" %}
The Nova's `sensorToMechanismRatio` is set to the gear ratio, or to the external encoder's gearing when that encoder closes the loop, so it already works in mechanism rotations, like a Talon. Each closed loop slot goes to one of the Nova's two slots:

| YAMS                               | On the Nova                                                                                               | Conversion                                          |
| ---------------------------------- | --------------------------------------------------------------------------------------------------------- | --------------------------------------------------- |
| `kP`, `kI`, `kD`                   | the slot's PID (`Pid0` or `Pid1`), outputting volts                                                       | ÷ mechanism rotations per meter when distance based |
| `withClosedLoopTolerance`          | the slot's allowable error                                                                                | in mechanism rotations                              |
| Trapezoidal profile                | the slot's motion profile (`Mp0` or `Mp1`) maximum velocity and acceleration                              | ÷ mechanism rotations per meter when distance based |
| Feedforward `kS`, `kV`, `kA`, `kG` | the slot's motion profile, with cosine gravity for an arm; the Nova only applies them in a motion profile | elevator `kV`, `kA` ÷ mechanism rotations per meter |

The Nova only applies a feedforward in a motion profile. So a feedforward without a trapezoidal profile runs on the SystemCore, together with the PID, and YAMS raises an alert saying why. Add [`withTrapezoidalProfile`](motion-profiles.md) to run both on the Nova. A feedforward whose gains are all 0 counts, as live tuning can change them, and so does one in `SLOT_1`, which can be switched to later. That puts a Nova swerve drive motor or flywheel with `kS` and `kV`, and an arm with `kG` but no profile, on the SystemCore, so tune their PID gains there. A velocity trapezoidal profile, an exponential profile or an [LQR controller](../../../understanding/lqr-controllers.md) runs on the SystemCore, in YAMS units, and the Nova then only receives the voltage the loop asks for.
{% endtab %}

{% tab title="A301" %}
The A301 has no closed loop gains, so nothing is converted. YAMS runs the PID, profiles and feedforward on the SystemCore in YAMS units, per mechanism rotation (or meter), and sends the A301 the voltage the loop asks for.
{% endtab %}
{% endtabs %}

The units are the same on every controller, but [where the loop runs](closed-loop-control.md#where-the-loop-runs) still matters. A loop on the motor controller runs every millisecond, so gains tuned there are not the best gains for a loop on the SystemCore.

## Code reference

* [`SmartMotorControllerConfig`](https://javadocs.yassrobotics.com/yams/yams/core/motorcontrollers/SmartMotorControllerConfig.html)
