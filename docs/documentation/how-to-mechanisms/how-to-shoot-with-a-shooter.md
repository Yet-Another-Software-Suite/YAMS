---
icon: fan
---

# How do I shoot with a shooter?

**Goal:** hold a button to spin the shooter up, and have the feeder push the game piece in only once the wheel is at speed.

A shot has three parts:

1. **Spin up:** the shooter `FlyWheel` goes to a speed and holds it.
2. **Wait for speed:** a `Trigger` turns true once the wheel has been near that speed for 0.1 seconds.
3. **Feed:** the feeder `FlyWheel` pushes the game piece into the wheel while the trigger is true.

**Before you start:** build the shooter as in the [Shooter/FlyWheels tutorial](../tutorials/shooter-flywheels/), and a feeder the same way as the intake in [How do I set up an intake roller?](how-do-i-set-up-an-intake-roller.md), with its `intake()` command renamed `feed()`. The feeder keeps the intake's default command that stops it: `stop()` in Commands v2, `idle()` in Commands v3. The shooter and the feeder are separate subsystems, one YAMS mechanism each, so each can run its own command at the same time.

<figure><img src="../.gitbook/assets/howto-shooter-at-speed.gif" alt="Animation of a flywheel spinning up to 3000 RPM and shooting two balls, with a band of 100 RPM around the setpoint. Each shot drops the speed by several hundred RPM, and it overshoots and rings as it recovers. A lane for the at speed trigger is true inside the band, including for an instant each time the speed passes through it. A lane for the feeder, which uses the trigger with a 0.1 second debounce, is only true once the speed has stayed in the band, so the second ball is not fed until the wheel has settled."><figcaption><p>Each shot slows the wheel and it rings as it recovers. The tolerance band and a debounce keep the feeder off until it has settled.</p></figcaption></figure>

{% stepper %}
{% step %}
### Spin up

Use the tutorial's shooter config as it is. Its gearing and gains are what let the wheel reach the shot speed: a NEO through a 1.5:1 reduction tops out near 3800 RPM at the wheel, `kV` of 0.19 volts per rotation per second gets it close to the target on feedforward alone, and `kP` of 0.2 fixes the rest and recovers after each shot. If your shooter has different gearing, find your own `kV` as in the [tutorial's Tune it step](../tutorials/shooter-flywheels/tune.md): a first guess is 12 volts divided by the wheel's free speed in rotations per second.

Add these two methods to the shooter class:

{% tabs %}
{% tab title="Commands v2" %}
{% code title="ShooterSubsystem.java (fragment)" %}
```java
import static org.wpilib.units.Units.RPM;

import org.wpilib.command2.Command;
import org.wpilib.command2.button.Trigger;
import org.wpilib.units.measure.AngularVelocity;

/** Spin at a speed and hold it. Runs until interrupted. */
public Command spinUp(AngularVelocity speed) {
  return shooter.run(speed);
}

/** True once the wheel has stayed within 100 RPM of a speed for 0.1 seconds. */
public Trigger ready(AngularVelocity speed) {
  return shooter.near(speed, RPM.of(100)).debounce(0.1);
}
```
{% endcode %}
{% endtab %}

{% tab title="Commands v3" %}
{% code title="ShooterMechanism.java (fragment)" %}
```java
import static org.wpilib.units.Units.RPM;
import static org.wpilib.units.Units.Seconds;

import org.wpilib.command3.Command;
import org.wpilib.command3.Trigger;
import org.wpilib.units.measure.AngularVelocity;

/** Spin at a speed and hold it. Runs until interrupted. */
public Command spinUp(AngularVelocity speed) {
  return shooter.run(speed);
}

/** True once the wheel has stayed within 100 RPM of a speed for 0.1 seconds. */
public Trigger ready(AngularVelocity speed) {
  return shooter.near(speed, RPM.of(100)).debounce(Seconds.of(0.1));
}
```
{% endcode %}
{% endtab %}

{% tab title="C++" %}
{% code title="cpp/subsystems/ShooterSubsystem.cpp (fragment)" %}
```cpp
// Spin at a speed and hold it. Runs until interrupted.
wpi::cmd::CommandPtr ShooterSubsystem::SpinUp(wpi::units::revolutions_per_minute_t speed) {
  return m_shooter->Run(wpi::units::degrees_per_second_t{speed});
}

// True once the wheel has stayed within 100 RPM of a speed for 0.1 seconds.
wpi::cmd::Trigger ShooterSubsystem::Ready(wpi::units::revolutions_per_minute_t speed) {
  return m_shooter->IsNear(speed, 100_rpm).Debounce(0.1_s);
}
```
{% endcode %}

Declare both in `ShooterSubsystem.hpp`, as the [tutorial's commands step](../tutorials/shooter-flywheels/commands.md) does for `Run` and `AtSpeed`.
{% endtab %}
{% endtabs %}

The tutorial's `atSpeed` uses a 50 RPM tolerance and no debounce, which is fine for watching the wheel. Feeding needs the wider tolerance and the debounce: see the next step.

**Check it:** the code builds.
{% endstep %}

{% step %}
### Wait for speed

Every shot takes speed out of the wheel, and the speed overshoots and rings as the closed loop brings it back. The 100 RPM tolerance keeps `ready` false while the wheel is too slow or too fast. The speed still passes through the tolerance for an instant on each swing, and the 0.1 second debounce ignores that: `ready` only turns true once the speed has stayed inside the tolerance. Without the debounce, the feeder pulses while the wheel is still ringing.

For a plain boolean outside of a `Trigger`, `FlyWheel` also has `isNear(speed, tolerance)`.

**Check it:** you know the shot speed you will use, and it is well below the wheel's top speed of about 3800 RPM.
{% endstep %}

{% step %}
### Feed

Bind both to the same button. The shooter spins up as soon as the button is held, and the feeder only runs while the button is held **and** the shooter is ready.

{% tabs %}
{% tab title="Commands v2" %}
{% code title="RobotContainer.java (fragment)" %}
```java
private static final AngularVelocity SHOT_SPEED = RPM.of(3000);

private final ShooterSubsystem shooter = new ShooterSubsystem();
private final FeederSubsystem feeder = new FeederSubsystem();

private void configureBindings() {
  // Hold the right trigger: spin up.
  m_driverController.rightTrigger().whileTrue(shooter.spinUp(SHOT_SPEED));

  // While the trigger is held and the wheel is ready: feed.
  m_driverController.rightTrigger()
      .and(shooter.ready(SHOT_SPEED))
      .whileTrue(feeder.feed());
}
```
{% endcode %}
{% endtab %}

{% tab title="Commands v3" %}
{% code title="ExampleTeleop.java (fragment)" %}
```java
// In the opmode constructor. SHOT_SPEED is RPM.of(3000).
var shoot = controller.rightTrigger();

// Hold the right trigger: spin up.
shoot.whileTrue(robot.shooter.spinUp(SHOT_SPEED));

// While the trigger is held and the wheel is ready: feed.
shoot.and(robot.shooter.ready(SHOT_SPEED)).whileTrue(robot.feeder.feed());
```
{% endcode %}
{% endtab %}

{% tab title="C++" %}
{% code title="cpp/RobotContainer.cpp (fragment)" %}
```cpp
// Hold the right trigger: spin up.
driverController.RightTrigger().WhileTrue(shooter.SpinUp(3000_rpm));

// While the trigger is held and the wheel is ready: feed.
(driverController.RightTrigger() && shooter.Ready(3000_rpm)).WhileTrue(feeder.Feed());
```
{% endcode %}
{% endtab %}
{% endtabs %}

**Check it:** the code builds, and holding the right trigger in simulation spins the shooter up.
{% endstep %}
{% endstepper %}

## Check it

1. Start the simulation, and graph `NT:/Mechanisms/Shooter/ShooterMotor/mechanism/velocity` (rotations per second; 3000 RPM is 50) and the feeder's `NT:/Mechanisms/Feeder/FeederMotor/motor/outputVoltage` in AdvantageScope. `Feeder` and `FeederMotor` are the telemetry names you gave the feeder's configs.
2. Hold the right trigger. The shooter's speed should rise to about 50 rotations per second.
3. The feeder's voltage should go to 8 volts only after the shooter's speed has settled, not while it is still climbing.

## If it did not work

* **The wheel settles below 3000 RPM and the feeder never runs.** `kV` is too low (or 0). Raise it until the wheel gets close on its own, then tune `kP`.
* **The wheel never gets near 3000 RPM, even at full voltage.** The gearing is too slow. Check `withGearing`: the motor's free speed divided by the reduction must be well above the shot speed.
* **The feeder pulses on and off.** The tolerance is too tight, or the debounce is missing. Try 100 to 150 RPM and 0.1 seconds.
* **Something else.** See [Troubleshooting](../tutorials/troubleshooting.md).
