---
description: >-
  A game piece makes a mechanism heavier, and can move its center of mass, which
  changes the feedforward and feedback gains it needs.
icon: weight-hanging
---

# How do I control a mechanism that carries a game piece?

Gains tuned on an empty mechanism are tuned for that mechanism. Pick up a game piece and the elevator is heavier, and the arm is heavier and has its center of mass further out. An empty arm that holds its angle perfectly can sag below it with a game piece, or overshoot and oscillate when it moves.

How much changes depends on the mechanism: how heavy the game piece is next to the mechanism, and where it is held.

**Before you start:** tune the empty mechanism first, as in its tutorial's Tune it step ([Arm](../tutorials/arm/tune.md), [Elevator](../tutorials/elevator/tune.md)). This page starts from those gains.

## What changes, and what doesn't

| Gain           | Depends on                                                   | With a game piece                                                         |
| -------------- | ------------------------------------------------------------ | ------------------------------------------------------------------------- |
| `kS`           | Friction in the gearbox and bearings                         | Barely changes                                                            |
| `kV`           | The motor and the gearing                                    | **Does not change.** It is the voltage the motor needs to spin at a speed |
| `kG`           | The weight, and for an arm how far out the center of mass is | **Goes up**, often a lot                                                  |
| `kA`           | The mass, or for an arm the moment of inertia                | Goes up                                                                   |
| `kP`, `kD`     | The mass or inertia they have to move                        | The same gains move a heavier mechanism more slowly, with more overshoot  |
| Profile limits | How fast the motors can accelerate the mechanism             | A heavier mechanism may not keep up with the profile                      |

`kG` is usually the one that matters. It holds the mechanism up, so a `kG` that is too small makes the mechanism sit below its setpoint, with `kP` pushing against the difference.

## How much it changes, by mechanism

| Mechanism          | `kG` with the game piece                                                                                                                    | Example                                                                                |
| ------------------ | ------------------------------------------------------------------------------------------------------------------------------------------- | -------------------------------------------------------------------------------------- |
| Elevator           | `kG` empty × (carriage weight + game piece weight) / carriage weight                                                                        | A 1 kg game piece on a 4 kg carriage: 25% more                                         |
| Arm                | `kG` empty × (arm weight × arm's center of mass distance + game piece weight × its distance) / (arm weight × arm's center of mass distance) | A 1 kg game piece 0.6 m out, on a 3 kg arm with its center of mass 0.3 m out: 67% more |
| Arm on an elevator | The elevator's `kG` goes up with the arm and game piece, whatever the arm's angle                                                           |                                                                                        |
| Shooter flywheel   | No `kG`, and `kV` does not change: the game piece passes through. See [How do I shoot with a shooter?](how-to-shoot-with-a-shooter.md)      |                                                                                        |

These give a starting point, and tell you whether a game piece changes the gains enough to matter: a change of a few percent may not be worth a second set of gains. A spring or counterweight takes a fixed amount off an elevator's `kG`. A cascading or continuous elevator's `kG` steps up as each stage is picked up, and a telescoping arm's or an arm with a wrist changes as it moves: see [A mechanism whose gains change as it moves](how-do-i-control-a-mechanism-that-carries-a-game-piece.md#a-mechanism-whose-gains-change-as-it-moves). A game piece held off to one side of an arm moves the angle where gravity pulls hardest, which `ArmFeedforward` does not model, so the arm holds well at some angles and sags at others.

## Measure the gains with the game piece

Measure the loaded gains on the robot, by driving the motor at a voltage with live tuning: on the [YAMS Live Tuner](../how-to-tuning/live-tuning/live-tuner.md)'s **Voltage** tab, or by setting the motor's `voltage/enabled` to `true` and typing into `voltage/setpoint` under `NT:/Tuning/<Mechanism>/<Motor>/`. [Drive a motor at a voltage](../how-to-tuning/live-tuning/#drive-a-motor-at-a-voltage) shows how.

{% stepper %}
{% step %}
### Load it and let go

Load a game piece, and move the mechanism to where gravity pulls hardest: an arm level, or an elevator at any height. Start the `Live Tuning` command and turn `voltage/enabled` on, so that only the voltage you type holds it from here on.

**Check it:** the mechanism drifts down slowly at 0 V, and moves when you type a voltage.
{% endstep %}

{% step %}
### Find the up and down voltages

Raise `voltage/setpoint` until the mechanism just starts to move up, and note it. Lower it until the mechanism just starts to move down, and note that.

**Check it:** the up voltage is larger than the down voltage.
{% endstep %}

{% step %}
### Work out kG and kS

`kG` is halfway between the two voltages. `kS` is half the difference between them. Turn `voltage/enabled` off and type the new `kG` into the loaded slot's feedforward.

**Check it:** the loaded `kG` is larger than the empty one, by about what [the table above](how-do-i-control-a-mechanism-that-carries-a-game-piece.md#how-much-it-changes-by-mechanism) predicts.
{% endstep %}

{% step %}
### Check the setpoints

With that `kG`, send the mechanism to its setpoints with the game piece, and raise `kP` (and `kD`) if it is slow.

**Check it:** the loaded mechanism settles on each setpoint, rather than sagging below it.
{% endstep %}
{% endstepper %}

## Switch gains when the mechanism picks up a game piece

A motor controller holds up to four [slots](../reference/editor/config/closed-loop-control.md#slots) of gains, each with its own [feedforward](../reference/editor/config/feedforward.md#slots). Put the empty gains in one slot and the loaded gains in another, and switch slots when the mechanism picks up or lets go of a game piece. Switching is quick: it only changes which slot the motor controller uses.

{% tabs %}
{% tab title="Commands v2" %}
{% code title="ArmSubsystem.java (fragment)" %}
```java
import org.wpilib.command2.Commands;
import org.wpilib.command2.button.Trigger;
import org.wpilib.math.controller.ArmFeedforward;
import yams.core.motorcontrollers.SmartMotorController.ClosedLoopControllerSlot;

private final SmartMotorControllerConfig smcConfig = new SmartMotorControllerConfig(this)
    // Empty.
    .withClosedLoopController(40, 0, 0, ClosedLoopControllerSlot.SLOT_0)
    .withFeedforward(new ArmFeedforward(0.1, 0.35, 1.2), ClosedLoopControllerSlot.SLOT_0)
    // Holding a game piece: more kG to hold it up, and more kP to move the extra weight.
    .withClosedLoopController(55, 0, 0, ClosedLoopControllerSlot.SLOT_1)
    .withFeedforward(new ArmFeedforward(0.1, 0.60, 1.2), ClosedLoopControllerSlot.SLOT_1)
    // ... the rest of the config
    ;

public ArmSubsystem() {
  // Switch gains as the intake picks up and lets go of a game piece. The commands require
  // nothing, so they do not interrupt the command moving the arm.
  new Trigger(this::hasGamePiece)
      .onTrue(Commands.runOnce(() -> motor.setClosedLoopSlot(ClosedLoopControllerSlot.SLOT_1)))
      .onFalse(Commands.runOnce(() -> motor.setClosedLoopSlot(ClosedLoopControllerSlot.SLOT_0)));
}
```
{% endcode %}
{% endtab %}

{% tab title="Commands v3" %}
{% code title="ArmMechanism.java (fragment)" %}
```java
import org.wpilib.command3.Command;
import org.wpilib.command3.Trigger;
import org.wpilib.math.controller.ArmFeedforward;
import yams.core.motorcontrollers.SmartMotorController.ClosedLoopControllerSlot;

private final SmartMotorControllerConfig smcConfig = new SmartMotorControllerConfig(this)
    // Empty.
    .withClosedLoopController(40, 0, 0, ClosedLoopControllerSlot.SLOT_0)
    .withFeedforward(new ArmFeedforward(0.1, 0.35, 1.2), ClosedLoopControllerSlot.SLOT_0)
    // Holding a game piece: more kG to hold it up, and more kP to move the extra weight.
    .withClosedLoopController(55, 0, 0, ClosedLoopControllerSlot.SLOT_1)
    .withFeedforward(new ArmFeedforward(0.1, 0.60, 1.2), ClosedLoopControllerSlot.SLOT_1)
    // ... the rest of the config
    ;

public ArmMechanism() {
  // Switch gains as the intake picks up and lets go of a game piece. The commands require
  // nothing, so they do not interrupt the command moving the arm.
  new Trigger(this::hasGamePiece)
      .onTrue(Command.noRequirements(coroutine -> motor.setClosedLoopSlot(ClosedLoopControllerSlot.SLOT_1))
          .named("Arm Loaded Gains"))
      .onFalse(Command.noRequirements(coroutine -> motor.setClosedLoopSlot(ClosedLoopControllerSlot.SLOT_0))
          .named("Arm Empty Gains"));
}
```
{% endcode %}
{% endtab %}
{% endtabs %}

The gains in this code are **placeholders** to show the shape of the change; use the ones you measured above.

`hasGamePiece()` is whatever tells you the mechanism has a game piece: a beam break, a distance sensor, or the intake's current, as in [How do I detect a game piece from motor current?](../how-to-motor-controllers-and-sensors/how-do-i-sense-activity-based-off-of-the-current.md). The slot takes effect with the next setpoint, and a mechanism's commands send a setpoint every loop.

[Loosely coupled followers](../reference/editor/config/followers.md#withlooselycoupledfollowers) are switched to the same slot, on every motor controller (SPARK MAX, SPARK Flex, TalonFX, TalonFXS, Thrifty Nova and A301). Give each follower the same slots in its own config.

## A mechanism whose gains change as it moves

A telescoping arm, an arm with a wrist, or an elevator picking up its stages needs a different `kG` as it moves, not only with a game piece. Two ways to handle it:

* **Slots for ranges.** Up to four sets of gains, such as retracted, extended, and each with a game piece. Switch with triggers on the position, as above. This is the simplest, and the gains in each slot can be tuned on their own.
* **Change `kG` as it moves**, with `motor.setKg(kG)`, worked out from the extension or angle. Only do this when `kG` has changed enough to matter, not every loop: on a Talon, SPARK or Thrifty Nova, `setKg` writes the motor controller's configuration, which takes time. On a mechanism whose [closed loop runs on the SystemCore](../reference/editor/config/closed-loop-control.md#where-the-loop-runs), it only changes a number.

## Common pitfalls

### Only `kP` was raised

A heavier mechanism that sags looks like it needs more `kP`. Raising `kP` pushes it closer to the setpoint, but it still sits below it, and a `kP` large enough to hide the sag makes the empty mechanism oscillate. Raise `kG` first, and `kP` only if the mechanism is still slow.

### The slot switches on a flickering sensor

A sensor that flickers as a game piece passes switches the gains back and forth. Debounce the trigger so it only switches once the sensor has settled: in Commands v2, `new Trigger(this::hasGamePiece).debounce(0.1)` (seconds); in Commands v3, `new Trigger(this::hasGamePiece).debounce(Seconds.of(0.1))`, with `import static org.wpilib.units.Units.Seconds;`.

### Slot 3 on a Talon

A TalonFX and a TalonFXS have three slots, `SLOT_0` to `SLOT_2`. Selecting `SLOT_3` throws an exception.

### The profile is too fast for the loaded mechanism

The motion profile's limits are the same in every slot. A profile that the empty mechanism keeps up with can be too fast with a game piece, so the mechanism lags behind it and overshoots at the end. Set the profile's limits for the loaded mechanism.

### The simulation is always empty

The simulation uses the mass and moment of inertia in the config, so it never carries the game piece. Gains for the loaded mechanism can be checked in simulation by setting the config's mass to the loaded mass, but tune them on the robot.

## Check it

Graph the arm's angle and setpoint, pick up a game piece, and send the arm to the same setpoint with and without it. With the right slot selected, the arm should settle on the setpoint both times, rather than sagging below it when loaded. Watch the motor's `closedloop/slot` value on the dashboard change from 0 to 1 as it picks the game piece up.
