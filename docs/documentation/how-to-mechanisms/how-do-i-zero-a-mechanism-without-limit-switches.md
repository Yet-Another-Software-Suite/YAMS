---
icon: arrows-down-to-line
---

# How do I zero a mechanism without limit switches?

**Goal:** make your arm or elevator's encoder read the right position, when there is no limit switch and no absolute encoder.

A motor's built-in encoder is **relative**: it only knows how far the mechanism has moved since it was last set. YAMS sets it once, when the robot code starts, to the starting position in your config. Everything after that is counted from there. If the mechanism was somewhere else at power on, or a belt skipped, every position is off by the same amount.

There are three ways to set it without a switch, from simplest to most automatic. They are alternatives: pick one. If you can add a sensor, an [absolute encoder](how-do-i-use-absolute-encoders-on-my-arm.md) or a [limit switch](how-do-i-home-a-mechanism-with-limit-switches.md) is more reliable than any of them.

## Always start in the same place

Pick a position the mechanism can always rest in when the robot is turned on, usually against its lower hard stop, and tell YAMS that position with `withStartingPosition` on the `SmartMotorControllerConfig`. A `Distance` starting position needs the drum size first, so it goes after `withMechanismCircumference` (or `withDrumRadius`): without it, YAMS cannot turn meters into rotations and throws.

{% code title="ElevatorSubsystem.java (fragment)" %}
```java
SmartMotorControllerConfig smcConfig = new SmartMotorControllerConfig(this)
    // ...
    .withMechanismCircumference(Inches.of(5.5))
    // The carriage rests on its lower hard stop when the robot turns on.
    .withStartingPosition(Meters.of(0));
```
{% endcode %}

For an arm, use an `Angle`, such as `.withStartingPosition(Degrees.of(-10))` for an arm resting on a stop 10 degrees below horizontal.

This only works if the whole team follows the rule: **the mechanism must be at that position whenever the robot code starts**, including after a code deploy or a brownout. Put it on the pre-match checklist.

## A "zero here" button

When the encoder is wrong, a human can push the mechanism to its hard stop and press a button that says "you are here". `SmartMotorController.setEncoderPosition` sets the encoder to the position you give it. `motor` is the `SmartMotorController` your mechanism was built with; these methods go in the same class.

{% tabs %}
{% tab title="Commands v2" %}
{% code title="ElevatorSubsystem.java (fragment)" %}
```java
/** Tell the encoder the carriage is on its lower hard stop. Works while disabled. */
public Command zeroHere() {
  return Commands.runOnce(() -> motor.setEncoderPosition(Meters.of(0)))
      .ignoringDisable(true)
      .withName("Elevator Zero Here");
}
```
{% endcode %}

Bind it to a button the drive team will not press by accident, for example `m_operatorController.back().onTrue(elevator.zeroHere());`.
{% endtab %}

{% tab title="Commands v3" %}
{% code title="ElevatorMechanism.java (fragment)" %}
```java
/** Tell the encoder the carriage is on its lower hard stop. */
public Command zeroHere() {
  return Command.noRequirements(coroutine -> motor.setEncoderPosition(Meters.of(0)))
      .named("Elevator Zero Here");
}
```
{% endcode %}

Bind it in your opmode to a button the drive team will not press by accident, for example `controller.back().onTrue(robot.elevator.zeroHere());`.
{% endtab %}
{% endtabs %}

Disable the robot, push the carriage down onto the stop by hand, and press the button.

## Drive into the hard stop and watch the current

The robot can find the hard stop itself: drive the mechanism slowly toward it, and when it stops moving and the current rises, it has arrived. This is current, or stall, homing.

Use the `home()` command from [Home on demand: drive onto the switch](how-do-i-home-a-mechanism-with-limit-switches.md#home-on-demand-drive-onto-the-switch), with two changes: it stops on `isStalled()` below instead of `isLowerSwitchPressed()`, and it sets the encoder to your hard stop, `Meters.of(0)` here, with `elevator.setMeasurementPositionSetpoint(Meters.of(0))` to hold it. The stall check is the only new part, and is the same for Commands v2 and v3:

{% code title="ElevatorSubsystem.java (fragment)" %}
```java
import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.MetersPerSecond;

import org.wpilib.math.filter.Debouncer;
import org.wpilib.units.measure.Current;

// Measure this on your robot: graph current/stator while the carriage pushes on the stop.
// Keep it well below the stator current limit.
private static final Current STALL_CURRENT = Amps.of(20);
// True only once the stall has lasted 0.25 seconds.
private final Debouncer stallDebouncer = new Debouncer(0.25);

/** True once the carriage has pushed on its hard stop, not moving, for 0.25 seconds. */
private boolean isStalled() {
  return stallDebouncer.calculate(
      motor.getStatorCurrent().gte(STALL_CURRENT)
          && motor.getMeasurementVelocity().isNear(MetersPerSecond.of(0), MetersPerSecond.of(0.02)));
}
```
{% endcode %}

The speed check matters: the current also rises for a moment when the motor starts, but the mechanism is moving then. It is the same idea as [detecting a game piece from motor current](../how-to-motor-controllers-and-sensors/how-do-i-sense-activity-based-off-of-the-current.md).

Put the soft limit at the hard stop. A soft limit inside it stops the mechanism short, so homing never finishes: on a TalonFX or TalonFXS, the Talon enforces the soft limits itself and stops the homing voltage too. See [Soft limits keep the mechanism off the switch](how-do-i-home-a-mechanism-with-limit-switches.md#soft-limits-keep-the-mechanism-off-the-switch).

## Check it

1. Move the mechanism away from the hard stop, then zero it with the method you chose.
2. Under **NetworkTables > Mechanisms**, the motor's `measurement/position` (or the arm's angle) should read the hard stop position, 0 m here.
3. Send it to a known height and measure with a tape measure: it should match.

## If it did not work

* **The position is off by the same amount every time.** The mechanism was not at the starting position when the code started, or the zero position in your code is not where the hard stop really is.
* **Stall homing ends too early.** The threshold is too low, or the start up spike is longer than the debounce. Raise the threshold or the debounce time.
* **Stall homing never ends.** The threshold is at or above the current limit, or the soft limit is inside the hard stop. Move the soft limit to the hard stop.
* **Something else.** See [Troubleshooting](../tutorials/troubleshooting.md).

## Read more

Before the first enable on the robot, work through the first-enable checks in the [Arm tutorial's Run it on the robot step](../tutorials/arm/robot.md). The elevator tutorial has no robot step, and the same checks apply to an elevator.
