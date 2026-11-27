---
icon: chart-line
---

# How do I use LQR instead of PID?

An LQR replaces the PID gains in a motor config: you describe the mechanism and your tolerances, and it works out its own gains. This page sets one up and tunes it. [LQR controllers](../../understanding/lqr-controllers.md) explains how it works, and [LQRConfig](../../reference/editor/lqr-config.md) lists every option.

## Before you start

* A mechanism already set up with a YAMS motor controller in closed loop mode, as in the [Arm tutorial](../../tutorials/arm/). This page changes its motor controller config.
* The mechanism's model: the motor, the gear ratio, and its moment of inertia (or, for an elevator, the carriage mass and drum radius). LQR is only as good as this model.
* Simulation and [AdvantageScope](../../how-to-simulation-and-logging/how-do-i-view-my-datalog-in-advantagescope.md), to try it before the robot.

## Set it up and tune it

{% stepper %}
{% step %}
### Describe the mechanism and your tolerances

Make an `LQRConfig` with the mechanism's model, then its type and tolerances. These are the [starting values](../../reference/editor/lqr-config.md#starting-values); the example robot's values are looser so there is something to tune.

{% tabs %}
{% tab title="Arm" %}
{% code title="LQRArmSubsystem.java (fragment)" %}
```java
LQRConfig lqrConfig = new LQRConfig(
        DCMotor.getNEO(1),
        new MechanismGearing(60),
        KilogramSquareMeters.of(0.25))     // arm moment of inertia
    .withArm(
        Degrees.of(5),                     // position error you accept
        DegreesPerSecond.of(60),           // velocity error you accept
        Degrees.of(3),                     // how far to trust the model's position
        DegreesPerSecond.of(30),           // and its velocity
        Degrees.of(0.5))                   // how far to trust the encoder
    .withControlEffort(Volts.of(12))
    .withMaxVoltage(Volts.of(12));
LQRController lqr = new LQRController(lqrConfig);
```
{% endcode %}

In C++, the same values are in radians and radians per second:

{% code title="LQRArmSubsystem.cpp (fragment)" %}
```cpp
yams::math::LQRConfig lqrConfig;
lqrConfig.WithArmSystem(wpi::math::DCMotor::NEO(1), 0.25, 60.0)  // motor, kg·m², reduction
    .WithQElems({0.0873, 1.047})        // 5° and 60°/s
    .WithRElems({12.0})
    .WithStateStdDevs({0.0524, 0.524})  // 3° and 30°/s
    .WithMeasurementStdDevs({0.00873})  // 0.5°
    .WithMaxVoltage(12_V);
```
{% endcode %}
{% endtab %}

{% tab title="Elevator" %}
{% code title="LQRElevatorSubsystem.java (fragment)" %}
```java
LQRConfig lqrConfig = new LQRConfig(
        DCMotor.getKrakenX60(2),
        new MechanismGearing(GearBox.fromReductionStages(10)),
        KilogramSquareMeters.of(0.01))     // not used by an elevator, but required
    .withElevator(
        Meters.of(0.02),                   // position error you accept
        MetersPerSecond.of(0.5),           // velocity error you accept
        Meters.of(0.005),                  // how far to trust the model's position
        MetersPerSecond.of(0.1),           // and its velocity
        Meters.of(0.001),                  // how far to trust the encoder
        Kilograms.of(5),                   // carriage mass
        Inches.of(1))                      // drum radius
    .withControlEffort(Volts.of(12))
    .withMaxVoltage(Volts.of(12));
LQRController lqr = new LQRController(lqrConfig);
```
{% endcode %}

In C++, in meters:

{% code title="LQRElevatorSubsystem.cpp (fragment)" %}
```cpp
yams::math::LQRConfig lqrConfig;
lqrConfig.WithElevatorSystem(wpi::math::DCMotor::KrakenX60(2), 5.0, 0.0254, 10.0)  // motors, kg, drum m, reduction
    .WithQElems({0.02, 0.5})
    .WithRElems({12.0})
    .WithStateStdDevs({0.005, 0.1})
    .WithMeasurementStdDevs({0.001})
    .WithMaxVoltage(12_V);
```
{% endcode %}
{% endtab %}

{% tab title="Flywheel" %}
{% code title="LQRFlyWheelSubsystem.java (fragment)" %}
```java
LQRConfig lqrConfig = new LQRConfig(
        DCMotor.getNEO(1),
        new MechanismGearing(1),
        KilogramSquareMeters.of(0.000585)) // wheel moment of inertia
    .withFlyWheel(
        RPM.of(100),                       // speed error you accept
        RPM.of(30),                        // how far to trust the model
        RPM.of(1))                         // how far to trust the encoder
    .withControlEffort(Volts.of(12))
    .withMaxVoltage(Volts.of(12));
LQRController lqr = new LQRController(lqrConfig);
```
{% endcode %}

In C++, in radians per second:

{% code title="LQRFlyWheelSubsystem.cpp (fragment)" %}
```cpp
yams::math::LQRConfig lqrConfig;
lqrConfig.WithFlywheelSystem(wpi::math::DCMotor::NEO(1), 0.000585, 1.0)  // motor, kg·m², reduction
    .WithQElems({10.47})                 // 100 RPM
    .WithRElems({12.0})
    .WithStateStdDevs({3.14})            // 30 RPM
    .WithMeasurementStdDevs({0.105})     // 1 RPM
    .WithMaxVoltage(12_V);
```
{% endcode %}
{% endtab %}
{% endtabs %}

**Check it:** the robot code builds.
{% endstep %}

{% step %}
### Give it to the motor config, with a feedforward

Replace the PID gains with the `LQRController`. Keep a feedforward: the LQR's model has no gravity, friction or battery sag, so an arm or elevator needs its kG, and a flywheel its kS, as with a PID. [What the model leaves out](../../understanding/lqr-controllers.md#what-the-model-leaves-out-gravity) shows why.

{% code title="LQRArmSubsystem.java (fragment)" %}
```java
new SmartMotorControllerConfig(this)
    .withControlMode(ControlMode.CLOSED_LOOP)
    .withClosedLoopController(lqr)                      // in place of kP, kI, kD
    .withFeedforward(new ArmFeedforward(0, 0.5, 0, 0))  // kG holds the arm up
    .withGearing(new MechanismGearing(60))
    .withMomentOfInertia(Meters.of(0.5), Kilograms.of(3))
    .withSoftLimits(Degrees.of(-20), Degrees.of(100))
    .withStartingPosition(Degrees.of(-20))
    .withZeroPower(MotorMode.BRAKE)
    .withStatorCurrentLimit(Amps.of(40))
    .withTelemetry("ArmMotor", TelemetryVerbosity.HIGH);
```
{% endcode %}

The config is the same in a Commands v3 `Mechanism`. In C++, pass the `LQRConfig` itself: `config.WithLQR(lqrConfig)`, with `WithFeedforward(...)` for the kG.

An LQR does not support a closed loop tolerance: `withClosedLoopTolerance` with an `LQRController` throws.

**Check it:** the robot starts in simulation, and the mechanism moves to a setpoint you command.
{% endstep %}

{% step %}
### Add a motion profile, if you want one

An LQR can follow a [trapezoidal or exponential profile](../../reference/editor/config/motion-profiles.md), set the usual way. It then tracks the profile's position and velocity instead of jumping straight to the goal. The example arm uses `withTrapezoidalProfile(DegreesPerSecond.of(90), DegreesPerSecondPerSecond.of(180))`.

**Check it:** a big move now starts and stops smoothly.
{% endstep %}

{% step %}
### Graph a step

Run the simulation, command a setpoint far from where the mechanism is, then command it back. In AdvantageScope, graph `closedloop/setpoint/position` (or `velocity` for a flywheel), `mechanism/position` (or `velocity`), and `motor/outputVoltage` on the right axis.

**Check it:** the mechanism reaches the setpoint and holds it. If it holds short, the kG is too small.
{% endstep %}

{% step %}
### Find the encoder and model trust

The trust values tell the [Kalman filter](../../understanding/lqr-controllers.md#the-kalman-filter-an-estimate-better-than-the-encoder) how far to believe each source, as standard deviations: the **encoder trust** is how noisy the encoder is, and the **model trust** is how far the real mechanism strays from what the model predicts. A larger value trusts that source less.

#### Only the ratio matters

The filter weighs the two against each other, so multiplying both by the same number changes nothing. Measured on the LQR arm from [LQR controllers](../../understanding/lqr-controllers.md), multiplying both by 10 gave the same estimate and the same output to three decimal places, and raising the model trust 10 times did exactly what lowering the encoder trust 10 times did. So measure the encoder trust, then set the model trust relative to it.

<figure><img src="../../.gitbook/assets/lqr-trust-noise.gif" alt="Animation of three LQR arms holding still at 45 degrees with the same noisy encoder, whose readings are gray dots scattered about half a degree either side. With the model trust at the starting values, the blue estimate is a flat line at 45 degrees and the voltage stays within a hundredth of a volt. At 10 times the model trust, the orange estimate wobbles with the readings and the voltage swings by about a tenth of a volt. At 100 times, the red estimate follows nearly every reading and the voltage swings by up to 0.4 volts either way."><figcaption><p>The more the filter trusts the encoder over the model, the more of the encoder's noise reaches the motor. Made with YAMS's LQRConfig on a simulated Kraken 100:1 arm, with 0.57° of encoder noise.</p></figcaption></figure>

#### Measure the encoder trust

The encoder trust is the encoder's noise: the standard deviation of its reading while the mechanism holds still. Log the mechanism holding still for a few seconds (a flywheel spinning at a steady voltage), and read the standard deviation in AdvantageScope's Statistics view. [How do I measure my encoder's noise?](how-do-i-measure-my-encoders-noise.md) shows each click, and how to convert the result: an arm's noise is read from its motor's `rotor/position`, because YAMS logs positions in rotations and the Statistics view rounds an arm's to 0.000.

If even the motor's reading does not change, the noise is smaller than one count of the encoder: use one count divided by 3.5 (√12). A NEO's built-in encoder on a 60:1 arm counts 2,520 times a turn, so one count is 0.14° and its trust about 0.04°. Use the measured value when you can: on a robot, vibration and backlash add noise that a still mechanism on the bench does not show.

#### Set the model trust against it

Start the model trust at the [starting values](../../reference/editor/lqr-config.md#starting-values): about the encoder trust for position, and ten times that for velocity. Then change it only for one of these reasons:

*   **Raise it** when the mechanism meets forces the model does not know: gravity, friction, a game piece, or a model that is badly off. The filter then follows the encoder more and the model less. Measured on the arm with no kG, holding at 45°:

    | Model trust     | Ended at | The filter believed | Voltage chatter while holding |
    | --------------- | -------- | ------------------- | ----------------------------- |
    | Starting values | 35.7°    | 44.1°               | 0.003 V                       |
    | 3 times         | 41.0°    | 44.0°               | 0.009 V                       |
    | 10 times        | 43.1°    | 44.1°               | 0.029 V                       |
    | 30 times        | 43.6°    | 44.0°               | 0.081 V                       |

    <figure><img src="../../.gitbook/assets/lqr-trust-gravity.gif" alt="Animation of four LQR arms with no kG, all commanded from 0 to 45 degrees under gravity, with the model trust at 1, 3, 10 and 30 times the starting values. Every arm rises the same way, then stops short: at 1 times it sags to 35.7 degrees, at 3 times it holds 41.0, at 10 times 43.1 and at 30 times 43.7. Below, the voltage each one holds with: the more the model trust is raised, the more it chatters, up to swings of about a quarter of a volt at 30 times."><figcaption><p>Raising the model trust pulls the arm toward its setpoint, and adds chatter. A kG is the real fix. Made with YAMS's LQRConfig on the same simulated arm, with a 6 kg load 0.4 m out.</p></figcaption></figure>

    It helps, but a kG fixes the cause, and even at 100 times the arm still ended 1.2° short. So add the feedforward first, and raise the model trust only for what is left.
* **Lower it** when the output is noisy and the model is good: the filter then smooths the encoder's noise out more.

Stop when `motor/outputVoltage` starts to chatter while the mechanism holds still: the filter is passing the encoder's noise straight to the motor. A model that is only a little off barely needs any of this: with the arm's real inertia 1.5 times the model's, the estimate stayed within about 0.3° of the truth at every setting.

**Check it:** holding still, `motor/outputVoltage` is steady, and the mechanism holds its setpoint.
{% endstep %}

{% step %}
### Tune it live

With `HIGH` telemetry, the LQR's values are tunable under `NT:/Tuning/<Mechanism>/<Motor>/closedloop/lqr/`, in place of kP, kI and kD. Start the [Live Tuning command](../live-tuning/), type a value, and the LQR is rebuilt with it on the next loop, starting from its current estimate so the output does not jump. Change one value at a time, in this order:

1. **The feedforward first.** kG, kS, kV and kA are still added to the LQR's output and tuned as in [Tuning order](../live-tuning/#tuning-order). With a wrong kG, an arm or elevator ends up short whatever the LQR does.
2. **The response, with the qelms and control effort.**
   * Too slow: lower `qelms/position` (on a flywheel, `qelms/velocity`), or raise `relms` up to 12 V.
   * Overshoots or rings: lower `qelms/velocity` on an arm or elevator, or lower `relms`.
   * `motor/outputVoltage` pinned at the limit: the LQR asks for more than the motor has, so raise the qelms or lower `relms`.
3. **The trust values last, if at all,** as in [Find the encoder and model trust](./#find-the-encoder-and-model-trust). If the output is noisy, raise `encoder/position` (on a flywheel, `encoder/velocity`) or lower the model trust; if the mechanism is pushed by something the model does not know, raise `model/position` and `model/velocity`.

Every value must be above 0; one the LQR cannot use leaves it as it was, with a warning on the Driver Station. [LQRConfig](../../reference/editor/lqr-config.md#live-tuning-keys) lists the keys and their units.

<figure><img src="../../.gitbook/assets/lqr-knobs.png" alt="Three step responses. Gentle, with a position tolerance of 0.2 radians and a control effort of 6 volts, is slow and uses little voltage. Balanced, with 0.1 radians and 12 volts, is quick and settles cleanly. Aggressive, with 0.02 radians and 12 volts, is fastest and saturates the motor"><figcaption><p>The two settings you will change most. Illustrative responses.</p></figcaption></figure>

**Check it:** typing a smaller `qelms/position` makes the next move faster.
{% endstep %}

{% step %}
### Copy the values into your config

Live tuning changes nothing in your code. Write the values down and put them in your `LQRConfig`. In Java the units carry over: `Degrees.of(...)`, `RPM.of(...)`, `Meters.of(...)`, `MetersPerSecond.of(...)` and `Volts.of(...)`. In C++, `WithQElems`, `WithStateStdDevs` and `WithMeasurementStdDevs` take radians, radians per second, meters and meters per second, so convert the degrees and RPM first.

**Check it:** after a restart, the mechanism moves as it did with the tuned values.
{% endstep %}
{% endstepper %}

## What you should see

<figure><img src="../../.gitbook/assets/lqr-tuning.gif" alt="Animation of an arm making the same 90 degree move three times: gentle, where it is slow and uses little voltage; aggressive, where it overshoots and the voltage saturates; and balanced, where it is quick and settles cleanly"><figcaption><p>The same move with three sets of priorities. Illustrative, not a recording.</p></figcaption></figure>

## The complete file

The example arm, for Commands v2. The [`lqr_mechanisms` example](../../../../examples/commands2/lqr_mechanisms) also has an LQR flywheel, and a Commands v3 version of both.

{% @github-files/github-code-block url="https://github.com/Yet-Another-Software-Suite/YAMS/blob/master/examples/commands2/lqr_mechanisms/java/first/robot/subsystems/LQRArmSubsystem.java" %}

## If it did not work

* **It holds short of the setpoint.** The feedforward's kG is too small or missing. Check it first; then lower `qelms/position`, so a small error costs more.
* **It oscillates around the setpoint.** Lower the control effort (`relms`), or lower `qelms/velocity`. Check the mechanism for backlash.
* **It is slow.** Raise the control effort up to 12 V, or lower `qelms/position`. With a motion profile, raise the profile's limits: the LQR follows the profile, so a slow profile gives a slow move.
* **It works in simulation but not on the robot.** The model is wrong: re-measure the mass and moment of inertia, check the gear ratio, and compare the model with the real mechanism's kV and kA, as in [Measuring kV and kA](../../understanding/exponential-profiles.md#measuring-kv-and-ka-optional).
* **"IllegalArgumentException" at startup with `withClosedLoopTolerance`.** An LQR does not support a closed loop tolerance; remove it.
