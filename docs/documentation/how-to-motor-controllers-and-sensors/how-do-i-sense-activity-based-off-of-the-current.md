---
icon: bolt
---

# How do I detect a game piece from motor current?

**Goal:** know when your intake has a game piece, without a beam break or limit switch, by watching the roller motor's current.

When a game piece jams against a spinning roller, the roller slows and the motor draws more current. **Stator current** is the current through the motor's windings, and it rises with the load. YAMS reads it with `SmartMotorController.getStatorCurrent()`, so you can turn "current has been high for a moment" into a `Trigger`.

This is a fallback for when there is no room or time for a real sensor. A beam break is more reliable, so ask for one if you can.

This page builds on the `IntakeSubsystem` from [How do I set up an intake roller?](../how-to-mechanisms/how-do-i-set-up-an-intake-roller.md)

{% stepper %}
{% step %}
### Measure your current

Simulation does not model a game piece, so this step has to be done on the real robot. Pick the threshold from your robot, not from this page.

1. Deploy the intake from [How do I set up an intake roller?](../how-to-mechanisms/how-do-i-set-up-an-intake-roller.md) to the robot and open the motor's `current/stator` value under **NetworkTables > Mechanisms** as a graph (Elastic, AdvantageScope or the Glass graph widget).
2. Run the intake with nothing in it. Note the current once it is up to speed; a roller like this one often sits around 5 to 15 amps.
3. Feed it a game piece and let it jam. Note how high the current goes and how long it stays there.

Choose a threshold between the two, well clear of both: if it runs at 10 A empty and climbs to the 40 A limit with a game piece, try 25 A.

{% hint style="warning" %}
**Keep the threshold below the stator current limit.** The limit (`withStatorCurrentLimit(Amps.of(40))`) stops the current from going over 40 A, so a stalled roller reads at or just under 40 A. A threshold equal to the limit may never be reached, or flicker on and off.
{% endhint %}

**Check it:** you have two readings, empty and jammed, at least 10 A apart, and a threshold between them.
{% endstep %}

{% step %}
### Add the detection

Add a `Debouncer` and a method to the intake class. The `Debouncer` only reports `true` once the current has stayed over the threshold for 0.1 seconds, so the short spike when the motor starts does not count as a game piece. These lines are a fragment: they go in the `IntakeSubsystem` (or `IntakeMechanism`) class, next to the `motor` field.

{% code title="IntakeSubsystem.java (fragment)" %}
```java
import static org.wpilib.units.Units.Amps;
import org.wpilib.math.filter.Debouncer;
import org.wpilib.units.measure.Current;

// Over this, the roller is pushing against a game piece. Measured on our robot:
// about 10 A empty, 40 A (the current limit) with a game piece jammed.
private static final Current GAME_PIECE_CURRENT = Amps.of(25);

// True only after the current has stayed high for 0.1 seconds.
private final Debouncer gamePieceDebouncer = new Debouncer(0.1);

/** True once the roller has been pushing against a game piece for 0.1 seconds. */
public boolean hasGamePiece() {
  return gamePieceDebouncer.calculate(motor.getStatorCurrent().gte(GAME_PIECE_CURRENT));
}
```
{% endcode %}

**Check it:** the code builds.
{% endstep %}

{% step %}
### Use it

Run the intake until it has a game piece, then stop.

{% tabs %}
{% tab title="Commands v2" %}
{% code title="IntakeSubsystem.java (fragment)" %}
```java
/** Run the intake until a game piece is detected. */
public Command intakeUntilGamePiece() {
  return intake().until(this::hasGamePiece);
}
```
{% endcode %}

To use it as a `Trigger` elsewhere (for example to rumble the controller), wrap it: `new Trigger(intake::hasGamePiece)`.
{% endtab %}

{% tab title="Commands v3" %}
{% code title="IntakeMechanism.java (fragment)" %}
```java
/** Run the intake until a game piece is detected. */
public Command intakeUntilGamePiece() {
  return run(coroutine -> {
    while (true) {
      if (hasGamePiece()) {
        intake.setVoltageSetpoint(Volts.of(0));
        return;
      }
      intake.setVoltageSetpoint(Volts.of(8));
      coroutine.yield();
    }
  }).named("Intake Until Game Piece");
}
```
{% endcode %}

To use it as a `Trigger` elsewhere, wrap it: `new Trigger(intake::hasGamePiece)` with `org.wpilib.command3.Trigger`.
{% endtab %}
{% endtabs %}

**Check it:** the code builds, and the intake runs when you press its button.
{% endstep %}
{% endstepper %}

## Check it

1. On the robot, graph `current/stator` and `hasGamePiece()` together (publish `hasGamePiece()` with `Telemetry.log`, or watch the command end).
2. Start the intake with nothing in it. `hasGamePiece()` should stay `false`, even during the spike when the motor starts.
3. Feed it a game piece. `hasGamePiece()` should turn `true` about 0.1 seconds after the current climbs over the threshold, and the intake should stop.

## If it did not work

* **It never detects the game piece.** The threshold is too high, or the same as the current limit. Lower it toward the middle of your empty and jammed readings.
* **It detects a game piece when the intake starts.** The start up spike lasts longer than 0.1 seconds. Raise the `Debouncer` time to 0.2 or 0.25 seconds, or the threshold.
* **It works with a full battery but not a low one.** A tired battery slows the roller and changes the current. Use `setVoltage` rather than `set` for the intake, and measure again with a used battery.

## Read more

To feed that game piece into a shooter, see [How do I shoot with a shooter?](../how-to-mechanisms/how-to-shoot-with-a-shooter.md)

For more on `Debouncer`, see the WPILib docs:

{% embed url="https://docs.wpilib.org/en/stable/docs/software/advanced-controls/filters/debouncer.html#debouncer" %}
