---
icon: house
---

# How do I home a mechanism with limit switches?

**Goal:** reset your mechanism's encoder to a known position whenever it presses a limit switch at a hard stop.

A relative encoder only knows how far the mechanism has moved since the robot turned on, and a slipped belt or a skipped gear throws it off. A limit switch at a hard stop tells the mechanism where it is: when the mechanism presses it, the motor's encoder is set to that hard stop. This is homing. It works for arms, elevators, pivots and turrets.

[YAMG](https://yamg.yassrobotics.com) generates this code for you: turn on **Homing** in its form.

## Wrap the switch in a `Sensor`

Plug the switch into a SystemCore Smart IO port and read it with a `DigitalInput`. Wrap it in a [`Sensor`](../understanding/sensors.md), so that the simulation can press it at the hard stop, as the mechanism presses the real one.

{% tabs %}
{% tab title="Commands v2" %}
{% code title="ArmSubsystem.java (fragment)" %}
```java
import static org.wpilib.units.Units.Degrees;

import org.wpilib.command2.Commands;
import org.wpilib.command2.button.Trigger;
import org.wpilib.hardware.discrete.DigitalInput;
import yams.core.mechanisms.config.SensorConfig;
import yams.core.motorcontrollers.simulation.Sensor;

// The arm's hard stops: -10 degrees and 100 degrees.
private final Arm arm = new Arm(armConfig, motor);

// A normally open switch between the signal and power pins of Smart IO port 4.
private final DigitalInput lowerSwitchInput = new DigitalInput(4);
private final Sensor lowerSwitch = new SensorConfig("ArmLowerSwitch")
    // The input reads high (true) while the switch is pressed.
    .withField("Pressed", lowerSwitchInput::get, false)
    // In simulation, the switch is pressed while the simulated arm is at its lower hard stop.
    .withSimulatedValue("Pressed", () -> motor.getSimSupplier()
        .map(sim -> sim.getMechanismPosition().lte(Degrees.of(-9.95)))
        .orElse(false), true)
    .getSensor();

public ArmSubsystem() {
  // Set the encoder to the hard stop once each time the switch is pressed.
  new Trigger(this::isLowerSwitchPressed)
      .onTrue(Commands.runOnce(() -> motor.setEncoderPosition(Degrees.of(-10))).withName("Arm Home LowerSwitch"));
}

public boolean isLowerSwitchPressed() {
  return lowerSwitch.getAsBoolean("Pressed");
}
```
{% endcode %}
{% endtab %}

{% tab title="Commands v3" %}
{% code title="ArmMechanism.java (fragment)" %}
```java
import static org.wpilib.units.Units.Degrees;

import org.wpilib.command3.Command;
import org.wpilib.command3.Trigger;
import org.wpilib.hardware.discrete.DigitalInput;
import yams.core.mechanisms.config.SensorConfig;
import yams.core.motorcontrollers.simulation.Sensor;

// The arm's hard stops: -10 degrees and 100 degrees.
private final Arm arm = new Arm(armConfig, motor);

// A normally open switch between the signal and power pins of Smart IO port 4.
private final DigitalInput lowerSwitchInput = new DigitalInput(4);
private final Sensor lowerSwitch = new SensorConfig("ArmLowerSwitch")
    // The input reads high (true) while the switch is pressed.
    .withField("Pressed", lowerSwitchInput::get, false)
    // In simulation, the switch is pressed while the simulated arm is at its lower hard stop.
    .withSimulatedValue("Pressed", () -> motor.getSimSupplier()
        .map(sim -> sim.getMechanismPosition().lte(Degrees.of(-9.95)))
        .orElse(false), true)
    .getSensor();

public ArmMechanism() {
  // Set the encoder to the hard stop once each time the switch is pressed.
  new Trigger(this::isLowerSwitchPressed)
      .onTrue(Command.noRequirements(coroutine -> motor.setEncoderPosition(Degrees.of(-10))).named("Arm Home LowerSwitch"));
}

public boolean isLowerSwitchPressed() {
  return lowerSwitch.getAsBoolean("Pressed");
}
```
{% endcode %}
{% endtab %}
{% endtabs %}

These are fragments of the class that owns the arm: `motor` is its `SmartMotorController`, and the fields go after `motor` and `arm` (see the pitfall below).

For an elevator, compare the height and set it in meters: `smcConfig.convertFromMechanism(sim.getMechanismPosition()).lte(Meters.of(0.0005))` and `motor.setEncoderPosition(Meters.of(0))`.

The trigger runs its command once each time the switch goes from released to pressed, so the encoder is set once per press. The scheduler checks triggers every loop, which also reads the `Sensor` every loop, as its simulated values need.

## Home on demand: drive onto the switch

The trigger above homes the arm whenever it happens to press the switch. To home it on purpose (at the start of autonomous, or from a button in the pits), drive it slowly toward the switch until it is pressed, then set the encoder. A low voltage is enough to move it without slamming the hard stop; start around 1 to 2 volts and raise it only if the arm does not move. Negative voltage here moves the arm down toward the lower switch; flip the sign if yours is the other way.

{% tabs %}
{% tab title="Commands v2" %}
{% code title="ArmSubsystem.java (fragment)" %}
```java
/** Drive down slowly until the lower switch is pressed, then set the encoder to the hard stop. */
public Command home() {
  return arm.setVoltage(Volts.of(-1.5))
      .until(this::isLowerSwitchPressed)
      .andThen(Commands.runOnce(() -> {
        motor.setEncoderPosition(Degrees.of(-10));
        // Hold where it stopped, rather than going back to the old setpoint.
        arm.setMechanismPositionSetpoint(Degrees.of(-10));
      }))
      .withName("Arm Home");
}
```
{% endcode %}
{% endtab %}

{% tab title="Commands v3" %}
{% code title="ArmMechanism.java (fragment)" %}
```java
/** Drive down slowly until the lower switch is pressed, then set the encoder to the hard stop. */
public Command home() {
  return run(coroutine -> {
    while (true) {
      if (isLowerSwitchPressed()) {
        break;
      }
      arm.setVoltageSetpoint(Volts.of(-1.5));
      coroutine.yield();
    }
    motor.setEncoderPosition(Degrees.of(-10));
    // Hold where it stopped. This also starts the closed loop again.
    arm.setMechanismPositionSetpoint(Degrees.of(-10));
  }).named("Arm Home");
}
```
{% endcode %}
{% endtab %}
{% endtabs %}

## Check it

1. Deploy, open the `ArmLowerSwitch` sensor under NetworkTables, and press the switch by hand. `Pressed` should be `true` only while you press it.
2. Run `home()` (or move the arm onto the switch). The arm should stop at the switch and its angle should read -10 degrees.
3. In simulation, the arm should read -10 degrees each time it reaches the lower hard stop (except with a SPARK; see the last pitfall).

## Common pitfalls

### The switch is wired or read like an older controller's

A SystemCore Smart IO port is the opposite of the DIO ports on older FRC controllers. It has a pull-down resistor, not a pull-up, and runs at 3.3 V. Wire a switch between the port's **signal** and **power** pins: the input reads low (`false`) while the switch is open, and high (`true`) while it is closed. A normally open switch therefore reads `true` while it is pressed, so its field is `input.get()`. A normally closed switch reads `false` while it is pressed, so its field is `!input.get()`. See [Digital Inputs - Hardware](https://docs.wpilib.org/en/latest/docs/hardware/sensors/digital-inputs-hardware.html).

Code ported from an older robot often reads `!input.get()`, because the older controllers' pull-up made a pressed switch read `false`. On a SystemCore that reads backwards, and the mechanism homes whenever the switch is _not_ pressed.

{% hint style="danger" %}
Never wire a switch between the power and ground pins, as switches on older controllers were wired between signal and ground: closing it connects power directly to ground, which trips the SystemCore's protection.
{% endhint %}

Check the switch before trusting it: press it by hand and watch the field on your dashboard.

### Soft limits keep the mechanism off the switch

Put the soft limit at the hard stop, where the switch is pressed. A soft limit inside the hard stop stops the mechanism before it reaches the switch, so the switch only homes it when it is already resting there, such as when the robot turns on. A homing command does not get around this: on a TalonFX or TalonFXS in `CLOSED_LOOP` mode, YAMS sets the soft limits on the Talon itself, and the Talon stops every request at them, including the voltage of `home()`.

### The encoder is set every loop

Home with `onTrue`, which sets the encoder once, as the switch is pressed. `whileTrue`, or an `if` in `periodic()`, sets it every loop while the switch is held. A mechanism often rests on its lower switch for most of a match, so that's a write to the motor controller every 20 ms, and a Talon waits for each one to be acknowledged. It also holds the encoder at the hard stop while the switch is pressed, so the encoder doesn't follow the mechanism through the last bit of the switch's travel. Home on the rising edge (`onTrue`, the moment the switch is pressed). `onFalse` is the falling edge: it homes as the mechanism leaves the switch, and where that happens depends on which way the mechanism moves and on the switch's hysteresis.

### The switch is not where the hard stop is

Homing sets the encoder to where the mechanism is when the switch is pressed. A switch that presses a little before the hard stop homes the mechanism to the wrong position by that much. Set the encoder to the position the switch presses at: move the mechanism onto the switch by hand and read where it is.

### The switch bends or moves

The mechanism hits its switches at the hard stops, which can bend them, shift their mounts or wear their levers. A switch that moves presses at a different position, so homing sets the wrong position, and the mechanism is off by that much until it homes again. Check the switches regularly, such as in pit checks: that they are tight and straight, and that the mechanism reads the hard stop when it presses each one.

### The `Sensor` is declared before the mechanism

The simulated value's lambda uses `motor` (or the mechanism). Java does not let a field's initializer use a field declared after it, even in a lambda, so declare the `Sensor` after them.

### The simulation presses the switch from the encoder

Press the simulated switch from where the simulated mechanism really is, `motor.getSimSupplier()`, rather than from the encoder, `arm.getAngle()`. A real switch is pressed by the mechanism, not by what the encoder says. If the encoder is off, a switch pressed from the encoder is pressed in the wrong place, and homing never corrects it.

### The sensor is not read every loop

A `Sensor`'s simulated values only apply when its field is read. A trigger reads it every loop. If you read it yourself, read it every loop, rather than only inside a command that may not be running.

### Two devices share a Smart IO port

A duty cycle absolute encoder, such as a Through Bore Encoder on the SystemCore, also uses a Smart IO port. Give each switch and each encoder its own port.

### A SPARK does not home in simulation

In REVLib 2027.0.0-alpha-7, the simulated SPARK sets its encoder back to its own position on the next loop, whatever sets it. The switches are still pressed at the hard stops in simulation, but `setEncoderPosition` does not change the simulated SPARK's encoder. Homing a real SPARK is not affected.

## If it did not work

See the pitfalls above, or [Troubleshooting](../tutorials/troubleshooting.md).
