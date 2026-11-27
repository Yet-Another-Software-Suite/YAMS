---
icon: sliders
---

# YAMS Live Tuner

The [YAMS Live Tuner](https://tuning.yassrobotics.com/) is a web page that tunes an elevator, arm, pivot or flywheel through [live tuning](./). It connects to the robot or simulation over NetworkTables, finds every YAMS motor controller with live tuning, and walks you through checking it before anything moves: a safety checklist, the motor inversion, and the range of motion. Then you send setpoints, drive with voltage and change gains while watching a graph.

It writes the same `NT:/Tuning` keys you would edit in AdvantageScope or Elastic, so everything on the [Live tuning](./) page still applies.

{% hint style="warning" %}
The Live Tuner drives the motor at a voltage while it checks the inversion and the range of motion. That needs YAMS v2026.10.22 or newer, which adds `voltage/enabled` and `voltage/setpoint` to live tuning, and registers the `Live Tuning` command for mechanisms such as `Elevator` and `Arm`.
{% endhint %}

The screenshots below come from the YAMS Commands v3 examples running in simulation: the BroncBotz 2025 elevator and coral arm, the BroncBotz 2026 turret and kicker, and the `double_flywheel` example. Each motor is set up with `withTelemetry(name, TelemetryVerbosity.HIGH)` and its mechanism with `withTelemetry(...)` too, which is all live tuning needs.

{% hint style="danger" %}
Like all tuning, this can be **dangerous** on a real robot. Practice in simulation first. Press **Space** anywhere on the page, or the red **STOP** button at the top, to stop straight away, and keep a hand on the Driver Station's disable.
{% endhint %}

## Connect

Open [tuning.yassrobotics.com](https://tuning.yassrobotics.com/). Pick where the robot code is running, then press **Connect**:

* **Robot (team)**: the robot, from your team number.
* **Simulation**: robot code running in simulation on this computer.
* **Custom host**: any other address.

Once connected, the badge shows the host and the robot's mode. Enable the robot, in Teleop or Test, from the Driver Station before anything moves.

<figure><img src="../../.gitbook/assets/live-tuner-connect.png" alt="The Live Tuner header connected to a simulation on localhost, with the robot in Teleop and enabled"><figcaption><p>Connected to a simulation, with the robot enabled in Teleop.</p></figcaption></figure>

{% hint style="info" %}
No robot code to hand? Pick **Demo (no robot)**, or open [tuning.yassrobotics.com/?demo](https://tuning.yassrobotics.com/?demo), to try the Live Tuner on a robot simulated in the page.
{% endhint %}

{% hint style="info" %}
If your browser asks whether the page may connect to devices on your local network, allow it. The robot and the simulation are on your local network.
{% endhint %}

## Tune a mechanism

The strip at the top of the page shows the five steps: **Mechanism**, **Checklist**, **Inversions**, **Range** and **Live tune**. Only one step is shown at a time, next to the telemetry, and each one has to be finished, or skipped where that is allowed, before the next. While nothing is running, select a finished step in the strip to open it again.

The telemetry card shows the position, velocity, stator current and output voltage, with a graph of the measured value against the setpoint. Select the motor temperature under the graph to switch between °C and °F.

The steps are the same for every mechanism. Where an elevator, an arm or pivot, and a flywheel differ, the step says so, with a picture of each.

{% stepper %}
{% step %}
### Pick the mechanism

Choose the motor controller under **Telemetry name**, and the subsystem (Commands v2) or mechanism (Commands v3) under **Live Tuning command**. The lists are filled in from what the robot publishes under `NT:/Tuning`, so a motor controller only shows up once live tuning is [turned on](./#turn-it-on) for it. Then set **Type** and press **Continue**:

* **Elevator:** setpoints are in meters.
* **Arm** for an arm that gravity pulls on, or **Pivot** for anything else that turns, such as a turret: setpoints are in degrees. For a pivot, tick **Continuous (no range of motion)** only if it can spin freely forever without wrapping wires.
* **Flywheel:** setpoints are in RPM. Leave **Second flywheel** at **None**, unless it is a [double flywheel](live-tuner.md#double-flywheels).

Changing anything in this step resets every confirmation after it.

{% tabs %}
{% tab title="Elevator" %}
<figure><img src="../../.gitbook/assets/live-tuner-elevator-mechanism.png" alt="The Mechanism step with Elevator/ElevatorMotor as the telemetry name, Elevator as the Live Tuning command and Elevator as the type, so setpoints are in meters, and a Continue button"><figcaption><p>The BroncBotz 2025 elevator: its motor, its mechanism's <code>Live Tuning</code> command, and the Elevator type.</p></figcaption></figure>
{% endtab %}

{% tab title="Arm and pivot" %}
<figure><img src="../../.gitbook/assets/live-tuner-arm-mechanism.png" alt="The Mechanism step with CoralArm/CoralArmMotor as the telemetry name, CoralArm as the Live Tuning command and Arm as the type"><figcaption><p>The BroncBotz 2025 coral arm, as an Arm.</p></figcaption></figure>

<figure><img src="../../.gitbook/assets/live-tuner-pivot-mechanism.png" alt="The Mechanism step with Turret/TurretMotor as the telemetry name, Turret as the Live Tuning command, Pivot as the type, and the Continuous (no range of motion) checkbox unticked"><figcaption><p>The BroncBotz 2026 turret, as a Pivot.</p></figcaption></figure>
{% endtab %}

{% tab title="Flywheel" %}
<figure><img src="../../.gitbook/assets/live-tuner-flywheel-mechanism.png" alt="The Mechanism step with Kicker/KickerMotor as the telemetry name, Kicker as the Live Tuning command, Flywheel as the type and None as the second flywheel"><figcaption><p>The BroncBotz 2026 kicker, as a Flywheel.</p></figcaption></figure>
{% endtab %}
{% endtabs %}

**Check it:** the Checklist step opens.
{% endstep %}

{% step %}
### Go through the pre-motion checklist

Tick every item, then press **Continue**. **Current trip (A)** stops Live Tuning if the stator current stays above it for 250 ms, which catches a stalled or jammed mechanism. You can change it later on the Settings card's **Tuner** tab. A flywheel's checklist has three items, the others four.

<figure><img src="../../.gitbook/assets/live-tuner-arm-checklist.png" alt="The pre-motion checklist with all four items ticked, the current trip at 40 A, and Continue"><figcaption><p>Every item confirmed, with a 40 A current trip.</p></figcaption></figure>

**Check it:** the Inversions step opens.
{% endstep %}

{% step %}
### Check the inversions

This step drives the motor at a small voltage, with no closed loop, to check that a positive voltage moves the mechanism in its positive direction, or spins a flywheel forward, and that the reading goes the same way.

Read the warning, tick the confirmation, and press **Start voltage control**. If the inversions were already checked on this robot, **Skip (already checked)** goes straight to the next step.

<figure><img src="../../.gitbook/assets/live-tuner-elevator-inversions-confirm.png" alt="The Check inversions step before it starts: a warning about driving open loop and followers fighting each other, the confirmation ticked, and the Start voltage control and Skip (already checked) buttons"><figcaption><p>Voltage control only starts once you confirm you are watching the mechanism.</p></figcaption></figure>

Drag the slider, or type a voltage under **Type a voltage** and press **Enter** or **Apply**. With **Back to 0 V on release** on, the slider goes back to 0 V when you let go; a typed voltage holds until you change it. Type 0, or press Space, to stop. The slider's ends start at −1 V and +1 V; select either end to change it.

Start with a fraction of a volt, at least 0.25 V, and raise it until the mechanism moves:

* **Elevator:** gravity pulls it down, so it can need more voltage to move up than down.
* **Arm:** near horizontal, gravity can be stronger than a small voltage, so try a little more voltage before changing an inversion.
* **Flywheel:** it keeps spinning after you let go, so set 0 V and let it stop before you drive it the other way.

{% tabs %}
{% tab title="Elevator" %}
<figure><img src="../../.gitbook/assets/live-tuner-elevator-inversions-up.png" alt="Holding a typed 1 V: the applied voltage reads 1.00 V, Positive voltage moves up is ticked, and Finish check is enabled"><figcaption><p>At +1 V the position rises, so <strong>Positive voltage moves up</strong> is ticked.</p></figcaption></figure>
{% endtab %}

{% tab title="Arm and pivot" %}
<figure><img src="../../.gitbook/assets/live-tuner-arm-inversions-up.png" alt="Holding a typed 1 V on the coral arm: the applied voltage reads 1.00 V, Positive voltage moves forward is ticked, and Finish check is enabled"><figcaption><p>At +1 V the angle rises, so <strong>Positive voltage moves forward</strong> is ticked.</p></figcaption></figure>
{% endtab %}

{% tab title="Flywheel" %}
<figure><img src="../../.gitbook/assets/live-tuner-flywheel-inversions-up.png" alt="Holding a typed 0.5 V on the kicker: the speed reads 80 RPM, Positive voltage moves forward is ticked, and Finish check is enabled"><figcaption><p>At +0.5 V the kicker spins forward.</p></figcaption></figure>
{% endtab %}
{% endtabs %}

One direction is enough. Once it is ticked, set the voltage back to 0 and press **Finish check**. If the reading goes the wrong way, the Live Tuner warns you. Stop and fix `withMotorInverted(...)`, or the encoder inversion, in the robot code before going on.

**Check it:** the direction is ticked, and the Range step opens.
{% endstep %}

{% step %}
### Range of motion

The Live Tuner only sends setpoints inside a range of motion you have set. A flywheel and a continuous pivot spin forever, so they have no range: Live tune opens next, and for a flywheel the speed slider's range caps it instead. For the others, there are two ways to set it:

* **Find ends by driving**: drive to each physical end of travel, and mark each end.
* **Enter range manually**: type each end, or move the mechanism by hand with the robot disabled and press **Use current position** under each end. Then press **Use this range**. Nothing is driven. The ends start at the robot's soft limits, if it publishes any.

To find the ends by driving, choose how to hold the mechanism between drives first. **Voltage, no closed loop** is safe before tuning: press **Use robot kG** so the **Gravity hold voltage** holds it up. On an arm, the gravity hold voltage is scaled by the cosine of the angle, like kG. **Closed loop (tuned gains)** holds it with the gains it has.

Press **Start verification**, then hold **Hold: drive up** or **Hold: drive down**. They drive at the ends of the **Voltage** slider, ±1 V to start with, and the slider drives at any voltage in between. Voltage is only applied while you hold a button or the slider. **When the voltage is let go** sets whether it cuts to 0 V, the default, or ramps down.

Press **Mark upper end here** and **Mark lower end here** at each end. A hard stop, the current trip, or a soft limit on the way marks that end for you. While it drives, the bar at the top of the window reads **VERIFYING RANGE**, and Space stops it. Once both ends are marked, press **Finish verification**. **Skip verification** goes on without a range, so only the robot's soft limits, if it has any, bound the travel.

{% tabs %}
{% tab title="Elevator" %}
<figure><img src="../../.gitbook/assets/live-tuner-elevator-range-verified.png" alt="The range step after driving: the upper end marked at 0.165 m, the lower end at 0.000 m marked by a hard stop, the Voltage slider from -1.00 V to 1.00 V, Cut to 0 V, and the gravity hold voltage set to the robot&#x27;s kG of 0.239 V"><figcaption><p>Driven up to 0.165 m and marked, then driven down until the hard stop marked 0 m.</p></figcaption></figure>
{% endtab %}

{% tab title="Arm and pivot" %}
<figure><img src="../../.gitbook/assets/live-tuner-arm-range-manual.png" alt="The range step with Enter range manually chosen: the lower end set to -42.9° with Use current position, the upper end typed as 45, and Use this range enabled"><figcaption><p>The coral arm: the lower end from where it rests, the upper end typed.</p></figcaption></figure>
{% endtab %}
{% endtabs %}

**Check it:** both ends are marked, or the step was skipped, and the Live tune step opens.
{% endstep %}

{% step %}
### Live tune

Press **Start Live Tuning**. This starts the mechanism's `Live Tuning` command, so the setpoints and gains you send are applied every loop.

On the **Setpoints** tab, send a setpoint by typing it and pressing **Send**, or with the slider. With **Follow slider** off, the setpoint is sent when you let go. The graph shows the measured value against the setpoint, with the stator current, the current trip and the range.

* **Elevator, arm and pivot:** the buttons are **Lower**, **25%**, **Mid**, **75%** and **Upper**, and the slider's ends start at the range of motion. On an arm or pivot, the motion profile is typed in degrees per second, with the RPM that YAMS reads shown under each value.
* **Flywheel:** the buttons are **Idle (0)** and a quarter, half, three quarters and all of the slider's range. Speeds are limited to the slider's range, which starts at the ±4000 RPM safety default; select either end to change it.

Change a gain on the **Settings** card, described in [The Settings card](live-tuner.md#the-settings-card), and send the setpoints again. Tune in the order of your mechanism's tutorial: the [Arm](../../tutorials/arm/tune.md), [Elevator](../../tutorials/elevator/tune.md), [Shooter/FlyWheels](../../tutorials/shooter-flywheels/tune.md) and [Double FlyWheel](../../tutorials/double-flywheel/tune.md) Tune it steps use the Live Tuner too. A pivot has no kG.

Turn on **Alternate setpoints** to switch between **Setpoint A** and **Setpoint B** every 5 seconds. Each step is the same, so you can change a gain and compare the next step with the last one.

{% tabs %}
{% tab title="Elevator" %}
<figure><img src="../../.gitbook/assets/live-tuner-elevator.gif" alt="The elevator stepping between its Mid and 25% setpoints, first with Max velocity cut from 1 m/s to 0.03 m/s, where it ramps slowly to each one, then with Max velocity put back to 1 m/s with its was button, where it gets there quickly"><figcaption><p>Max velocity cut to 0.03 m/s, then put back with <strong>was 1</strong>: the elevator crawls to each setpoint, then gets there quickly again.</p></figcaption></figure>

<figure><img src="../../.gitbook/assets/live-tuner-elevator-gains.png" alt="The Settings card&#x27;s Gains tab for the elevator: kP 33.966, kI 0, kD 0; kS 0.21471 V, kV 10.39, kA 0, kG 0.23861 V with a ± button; max velocity 1 m/s and max acceleration 0.5 m/s²"><figcaption><p>The elevator's gains and motion profile.</p></figcaption></figure>

<figure><img src="../../.gitbook/assets/live-tuner-elevator-alternate.png" alt="Alternate setpoints on, switching between 0.045 m and 0.126 m: the graph shows the elevator stepping up and back down"><figcaption><p>Stepping between 0.045 m and 0.126 m.</p></figcaption></figure>
{% endtab %}

{% tab title="Arm and pivot" %}
<figure><img src="../../.gitbook/assets/live-tuner-arm.gif" alt="The coral arm stepping between about -21° and 1°, first with Max velocity cut from 600 to 20 deg/s, where it moves slowly, then with Max velocity put back with its was button, where it moves quickly. Both times it settles a few degrees below each setpoint"><figcaption><p>The coral arm with Max velocity cut to 20 deg/s, then put back. Both times it settles a few degrees under each setpoint.</p></figcaption></figure>

<figure><img src="../../.gitbook/assets/live-tuner-pivot.gif" alt="The turret stepping between -100° and 0°, first with Max velocity cut from 720 to 45 deg/s, where it turns slowly, then with Max velocity put back with its was button, where it turns quickly"><figcaption><p>The turret, a pivot, with Max velocity cut to 45 deg/s, then put back.</p></figcaption></figure>

<figure><img src="../../.gitbook/assets/live-tuner-arm-gains.png" alt="The coral arm&#x27;s Gains tab: kP 71.8502, kI 0, kD 0; kS 0.19214 V, kV 2.01765, kA 0, kG 0.023981 V; max velocity 600 deg/s, shown as 100 RPM, and max acceleration 300 deg/s², shown as 50 RPM/s"><figcaption><p>The coral arm's gains, with its motion profile in degrees.</p></figcaption></figure>

<figure><img src="../../.gitbook/assets/live-tuner-arm-alternate.png" alt="Alternate setpoints on, switching between -20.9° and 23°: the graph shows the coral arm stepping up and back down"><figcaption><p>The coral arm stepping between −20.9° and 23°.</p></figcaption></figure>
{% endtab %}

{% tab title="Flywheel" %}
<figure><img src="../../.gitbook/assets/live-tuner-flywheel.gif" alt="The kicker stepping between 1000 and 2000 RPM: with kP raised from 0 to 0.5 it runs at about 1150 RPM at the 1000 RPM setpoint, and with kP put back to 0 with its was button it runs at about 1340 RPM. At 2000 RPM it runs at about 1920 RPM both times"><figcaption><p>The kicker with kP raised to 0.5, then put back to 0. With kP, the speed at the 1000 RPM setpoint comes closer to it.</p></figcaption></figure>

<figure><img src="../../.gitbook/assets/live-tuner-flywheel-gains.png" alt="The kicker&#x27;s Gains tab: kP 0, kI 0, kD 0; kS 0, kV 0.5, kA 0"><figcaption><p>The kicker's gains.</p></figcaption></figure>

<figure><img src="../../.gitbook/assets/live-tuner-flywheel-alternate.png" alt="Alternate setpoints on, switching between 1000 and 3000 RPM: the kicker runs at about 1340 and 1920 RPM"><figcaption><p>The kicker stepping between 1000 and 3000 RPM, with neither speed on its setpoint yet.</p></figcaption></figure>
{% endtab %}
{% endtabs %}

**Check it:** the measured value settles on each setpoint without overshooting.
{% endstep %}
{% endstepper %}

## Copy the gains into your code

Values sent from the Live Tuner are lost when the robot code restarts, so copy them from the **Gains** tab into the motor's `SmartMotorControllerConfig`:

| On the Gains tab                         | Goes in                                                                                                                                                                                       |
| ---------------------------------------- | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| kP, kI, kD                               | `withClosedLoopController(kP, kI, kD)`                                                                                                                                                        |
| kS, kV, kA, and kG on an arm or elevator | `withFeedforward(new SimpleMotorFeedforward(kS, kV, kA))` for a flywheel or pivot, `new ArmFeedforward(kS, kG, kV, kA)` for an arm, `new ElevatorFeedforward(kS, kG, kV, kA)` for an elevator |
| Max velocity, max acceleration           | `withTrapezoidalProfile(maxVelocity, maxAcceleration)`                                                                                                                                        |

In simulation you tuned the simulation's gains: when the config has `withSimClosedLoopController(...)` and `withSimFeedforward(...)`, the simulation uses those, so copy the values into those lines instead, and tune again on the real robot for the others. Each tutorial shows the lines before and after: Copy the numbers into your code for the [Arm](../../tutorials/arm/tune.md#copy-the-numbers-into-your-code), the [Elevator](../../tutorials/elevator/tune.md#copy-the-numbers-into-your-code), the [Shooter](../../tutorials/shooter-flywheels/tune.md#copy-the-numbers-into-your-code) and the [Double FlyWheel](../../tutorials/double-flywheel/tune.md#copy-the-numbers-into-your-code).

## Double flywheels

A double flywheel is two YAMS flywheels in one subsystem, sharing its `Live Tuning` command, such as the [`double_flywheel` example](../../../../examples/commands3/double_flywheel). Pick one wheel under **Telemetry name** and the other under **Second flywheel**, and both are tuned together.

<figure><img src="../../.gitbook/assets/live-tuner-double-flywheel-mechanism.png" alt="The Mechanism step with UpperFlyWheel/UpperMotor as the telemetry name, DoubleFlyWheelMechanism as the Live Tuning command, Flywheel as the type, and LowerFlyWheel/LowerMotor as the second flywheel"><figcaption><p>The <code>double_flywheel</code> example's upper and lower wheels.</p></figcaption></figure>

* **Inversions:** check one wheel at a time. The step passes once both wheels have.
* **Setpoints:** each wheel has its own box and slider. **Send both** sends both boxes. Turn on **Same speed for both** to move both sliders together. The buttons, such as **Both 2000 RPM**, and **Alternate setpoints** drive both wheels.
* **Wheel:** the **Wheel** picker on the telemetry card and the Settings card chooses which wheel's readings, gains and voltage drive you see. The graph plots both wheels, the second one dashed.
* **Copy these gains:** on the **Gains** tab, copies one wheel's feedback, feedforward and motion profile to the other, for wheels that are alike.
* **Safety:** the current trip watches each motor, and a stop spins both wheels down.

<figure><img src="../../.gitbook/assets/live-tuner-double-flywheel.png" alt="Live tune for the double flywheel: the upper wheel at 2416 RPM of a 3000 RPM setpoint and the lower wheel at 1611 RPM of 2000 RPM, sent together with Send both; the graph shows the upper wheel solid and the lower wheel dashed"><figcaption><p>3000 and 2000 RPM sent together. With the example's starting gains, both wheels settle short.</p></figcaption></figure>

<figure><img src="../../.gitbook/assets/live-tuner-double-flywheel-settings.png" alt="The Settings card with the Wheel picker on UpperFlyWheel: kP 0.002, kV 0.1, and a Copy these gains to LowerFlyWheel button"><figcaption><p>The upper wheel's gains, ready to copy to the lower wheel.</p></figcaption></figure>

\## The Settings card

The **Settings** card sits under the Live tune card. A line under its tabs says whether that tab's values live on the robot or only in this tuner.

| Tab         | Where it lives                   | What is on it                                                                                                                            |
| ----------- | -------------------------------- | ---------------------------------------------------------------------------------------------------------------------------------------- |
| **Gains**   | On the robot, under `NT:/Tuning` | Feedback, feedforward and motion profile                                                                                                 |
| **Limits**  | On the robot, under `NT:/Tuning` | Current limits, soft limits and the closed-loop ramp rate                                                                                |
| **Tuner**   | Only in this tuner               | The current trip, and the range of motion                                                                                                |
| **Voltage** | Only in this tuner               | What happens when you let go of the voltage, and the gravity hold voltage. Only shown while the Live tune card is on its **Voltage** tab |

Type a new value: it is applied when you press **Enter** or click away. **Esc** puts the old value back. [What to tune and what to watch](../../reference/keys-and-units.md#what-to-tune-and-what-to-watch) lists what each gain does.

* **was …:** under a setting you changed, **was** shows its value before the change. Press it to put that value back, and press it again to go one change further back. When more than one value is saved, the count is shown, as in **was 160 (2)**.
* **Number fields:** a field only takes what fits it. Gains take up to 4 significant digits, and only kG can be negative. Current limits and the current trip are whole amps. A comma is read as a decimal point.
* **Values read from the robot** are shown as the robot sends them, so they can have more than 4 significant digits, such as kP 33.966.
* **±:** on a touch screen, a field that takes negative numbers, such as kG or a soft limit, shows a **±** button to its left while the field is focused, since a phone's number keypad has no minus key.
* **LQR mechanisms:** a mechanism with an [LQR](../../understanding/lqr-controllers.md) shows the LQR's values on the **Gains** tab in place of kP, kI and kD: its tolerances, voltage effort, how far to trust the model and the encoder, and its maximum voltage. Each change rebuilds the LQR on the robot.

<figure><img src="../../.gitbook/assets/live-tuner-settings-limits.png" alt="The Limits tab: On the robot. A stator limit of 40 A, soft limits of 0 m and 0 m, and a closed-loop ramp of 0 s"><figcaption><p>The <strong>Limits</strong> tab, for the elevator.</p></figcaption></figure>

On the **Tuner** tab, **Range of motion** changes the range set in the [Range of motion](live-tuner.md#range-of-motion) step. Live Tuning stops if the mechanism leaves it, and setpoints are limited to it. It can go past what you verified, with a warning under the field, but that can drive the mechanism into a hard stop.

<figure><img src="../../.gitbook/assets/live-tuner-settings-tuner.png" alt="The Tuner tab: Only in this tuner. A current trip of 40 A, and a range of motion from 0.004 m to 0.167 m"><figcaption><p>The <strong>Tuner</strong> tab, with the current trip and the range of motion.</p></figcaption></figure>

## Drive with voltage

The Live tune card's **Voltage** tab moves the mechanism with voltage instead of the closed loop, for example to feel how much voltage it takes, or to bring it somewhere by hand.

Hold **Hold: drive up** or **Hold: drive down**, or drag the **Voltage** slider. The buttons drive at the slider's ends, ±2 V to start with; select either end to change it. The drive stops at the ends of the range of motion, or the soft limits if there is none.

* **During Live Tuning:** with **Stay in voltage control** on, letting go keeps voltage control at the gravity hold voltage, or 0 V. Switch back to **Setpoints**, or turn it off, to hand back to the closed loop, which holds where the mechanism is. A flywheel goes back to the speed it had.
* **Without Live Tuning:** **Start voltage control** drives with voltage only, and the motor gets 0 V, or the gravity hold voltage, between presses.

<figure><img src="../../.gitbook/assets/live-tuner-elevator-voltage.png" alt="The Voltage tab during Live Tuning on the elevator: Hold: drive down (−2.00 V) and Hold: drive up (+2.00 V), the voltage slider at 0 V, Stay in voltage control on, In voltage control. The closed loop is off, and Stopped at the upper end of the verified range (0.165 m)"><figcaption><p>Driven up to the end of the range, where the drive stopped. The closed loop is off until you go back to <strong>Setpoints</strong>.</p></figcaption></figure>

The Settings card's **Voltage** tab sets **When the voltage is let go**, **Cut to 0 V** or **Ramp to 0 V**, and the **Gravity hold voltage**. On an elevator or arm, the gravity hold voltage is added to every drive, and an arm's is scaled by the cosine of its angle, like kG. **Use robot kG** copies kG into it, and **Set kG to this** sends it to the robot as kG.

<figure><img src="../../.gitbook/assets/live-tuner-settings-voltage.png" alt="The Settings card&#x27;s Voltage tab: When the voltage is let go, Cut to 0 V or Ramp to 0 V; the gravity hold voltage at 0 V with Use robot kG (0.24 V), Set kG to this and Set to 0 V"><figcaption><p>The <strong>Voltage</strong> tab, for the elevator.</p></figcaption></figure>

## If the mechanism is outside the range

If the mechanism is outside its range of motion, for example an arm that sagged below its lower end while disabled, the Live tune card says so before you start. Start Live Tuning anyway, then send a setpoint inside the range, or drive it back on the **Voltage** tab. Until it is back inside, the bar at the top of the window shows an **Out of range** warning, and moving further out stops Live Tuning.

## Stop

While anything runs, the bar at the top of the window says what is running and stays there as you scroll.

<figure><img src="../../.gitbook/assets/live-tuner-stop-bar.png" alt="The bar at the top of the window: LIVE TUNING · Elevator/ElevatorMotor · Press Space anywhere to stop, and a red STOP button"><figcaption><p>The bar while Live Tuning runs on the elevator.</p></figcaption></figure>

Press **Space**, or **STOP**, at any time and from anywhere on the page. The Live Tuner sets the setpoint to where the mechanism is, or slows a flywheel down first, and then ends the `Live Tuning` command. The subsystem's default command, if it has one, takes over from there. Closing the page stops Live Tuning too, and so does disabling the robot from the Driver Station.
