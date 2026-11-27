---
description: Put the arm code on the robot, check it safely, and tune it on the real arm.
icon: robot
---

# Step 6: Run it on the robot

The same code runs on the robot. Before you enable it, check a few things that simulation cannot tell you: that the code talks to the right motor, that it measures the arm the right way round, and that the arm starts where the code thinks it does.

{% hint style="danger" %}
**Be ready to stop the arm.** Keep everyone clear of the arm, and keep a hand on the Driver Station: **Enter** disables the robot and **Space** stops it until it is rebooted. Do every enable in this step with the arm free to move and nothing in its way.
{% endhint %}

{% stepper %}
{% step %}
### Set the CAN ID

The code talks to the `SparkMax` with CAN ID 4 (step 1, part 8).

1. Connect to the SPARK MAX with the [REV Hardware Client](https://docs.revrobotics.com/rev-hardware-client).
2. Update its firmware and set its CAN ID to `4`, or change the `4` in your code to the ID it already has.

Every device on the CAN bus needs its own ID.
{% endstep %}

{% step %}
### Put the arm at its starting position

The NEO's encoder only measures how far the motor turns, not where the arm is. When the code starts, YAMS assumes the arm is at the starting position from step 1, part 6: -5 degrees.

Before each time the robot code starts, rest the arm at that angle. Here that is 5 degrees below level.

{% hint style="warning" %}
If the arm starts somewhere else, every angle, setpoint and soft limit is off by the difference. An arm started 30 degrees too high thinks a hard stop is 30 degrees further away than it is.
{% endhint %}

Change `withStartingPosition` to wherever your arm really rests, such as on a hard stop.
{% endstep %}

{% step %}
### Deploy

Open the command palette with <kbd>Ctrl+Shift+P</kbd> and run **WPILib: Deploy Robot Code**.

**Check it:** the Driver Station shows robot code, and its messages show no errors about the SPARK MAX. A CAN error there means the ID or the wiring is wrong.
{% endstep %}

{% step %}
### Check the direction, with the robot disabled

Keep the robot **disabled**. YAMS still publishes where the arm is.

1. In AdvantageScope or Elastic, watch `NT:/Mechanisms/Arm/ArmMotor/mechanism/position`. It reads about `-0.014` rotations (-5 degrees).
2. Lift the arm by hand, about 45 degrees.

**Check it:**

* The value goes **up**. Positive is up, the same as in simulation.
* It goes up by about `0.125` rotations, which is 45 degrees.

If it goes down instead, set `.withMotorInverted(true)` in step 1, part 5, deploy, and check again. If it moves much more or less than 45 degrees, the gearing in step 1, part 4 does not match your arm.

{% hint style="info" %}
In brake mode the motor resists being moved, and the gearbox makes the arm stiff to lift. If you can't move it by hand, set `.withZeroPower(MotorMode.COAST)` while you check, then set it back.
{% endhint %}

Do not skip this check. If the direction is wrong, the closed loop pushes the arm away from its setpoint, faster and faster, into a hard stop.
{% endstep %}

{% step %}
### First enable

1. Put the arm back at its starting position, and restart the robot code (or deploy again).
2. Enable **Teleoperated**.

**Check it:** the default command takes the arm from -5 degrees up to 0 degrees, smoothly, and holds it there. With the placeholder gains it may sag a little below level. That is fixed in the next part.

**Disable straight away** if the arm moves fast, heads for a hard stop, or shakes. Then go back to part 4.
{% endstep %}

{% step %}
### Tune it on the robot

Tune the real arm the same way you tuned the simulation in [step 5](tune.md). On the robot the dashboard changes `withClosedLoopController` and `withFeedforward`, not the `Sim` versions.

The real arm usually needs a larger `kG` than the simulation, because of the arm's real weight, and some `kS` to get past friction. To find `kS`, raise `closedloop/feedforward/kS` until the arm just starts to creep, then back it off a little.

When you are done, copy the numbers into the config:

{% tabs %}
{% tab title="Java" %}
```java
// The real robot's gains. ArmFeedforward takes kS, kG, kV.
.withClosedLoopController(60, 0, 1)
.withFeedforward(new ArmFeedforward(0.1, 1.2, 0.3))
```
{% endtab %}

{% tab title="C++" %}
```cpp
// The real robot's gains: kP, kI, kD, then kS, kG and kV.
.WithFeedback(60, 0, 1)
.WithFeedforward(wpi::math::ArmFeedforward{
    0.1_V, 1.2_V, wpi::units::unit_t<wpi::math::ArmFeedforward::kv_unit>{0.3}})
```

The C++ tutorial uses one feedforward for both, so the simulation now uses the robot's feedforward too.
{% endtab %}
{% endtabs %}

The numbers are an example. Use the ones you found.
{% endstep %}

{% step %}
### Check the soft limits

1. With live tuning running, type `30` into `NT:/Tuning/Arm/ArmMotor/closedloop/setpoint/position`.
2. The arm should stop at 30 degrees, the upper soft limit, short of the hard stop.

Do the same with `-20` for the lower soft limit. Set `withSoftLimits` in step 1 to just inside your arm's real hard stops.
{% endstep %}
{% endstepper %}

## If it did not work

* **Nothing moves, and the Driver Station shows a CAN error.** The CAN ID in the code does not match the SPARK MAX, or the CAN wiring is loose.
* **The arm runs into a hard stop as soon as it is enabled.** The direction is wrong. Disable, and redo part 4.
* **Angles are off by a fixed amount.** The arm was not at its starting position when the code started. Redo part 2.
* **Angles are off by a factor.** The gearing does not match the arm.
* **The arm sags or shakes.** Tune it, as in part 6.

More problems and fixes: [Troubleshooting](../troubleshooting.md).

## Read more

Your arm works on the robot. From here:

* Keep your constants tidy as the robot grows: [Organizing configs in constants](../../understanding/config-organization.md)
* Everything an arm can do: [Arms](../../understanding/arms.md)
