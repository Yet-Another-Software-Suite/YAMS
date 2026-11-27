---
icon: hand-fist
---

# Arms

An `Arm` is a mechanism that swings about a pivot and has to hold itself up against gravity: an intake arm, a shooter hood that tilts, a climber hook.

## How an arm works

<figure><img src="../.gitbook/assets/mech-arm.png" alt="An arm drawn at 40 degrees above a level dashed line marked 0 degrees, with straight up marked 90 degrees. Its length, set by withLength, runs from the pivot to the end of the arm, and gravity pulls down on the end. Level is zero because gravity pulls hardest on a level arm and ArmFeedforward uses kG times the cosine of the angle."><figcaption><p>An arm is measured from level.</p></figcaption></figure>

Following WPILib, an arm is at 0° when it is parallel to the ground and 90° when it points straight up. Gravity pulls hardest on a level arm and not at all on a vertical one, and [`ArmFeedforward`](../reference/editor/config/feedforward.md#withfeedforward) uses that angle to work out how much voltage holds the arm still. So the arm's encoder has to read 0 when the arm is level.

If your mechanism does not need gravity accounted for, because it turns about a vertical axis or is balanced, use a [Pivot](turrets-wrists.md) instead.

## Creating an arm

This is the shape of an arm class, not a complete file: the imports and the motor controller object (`spark`, a `SparkMax`) are left out. [Step 2 of the Arm tutorial](../tutorials/arm/mechanism.md) builds the complete file. The code is the same for Commands v2 and Commands v3 except for the package you import from and the class that owns the mechanism.

{% tabs %}
{% tab title="Commands v2" %}
```java
import yams.commands2.config.SmartMotorControllerConfig;
import yams.commands2.mechanisms.Arm;

public class ArmSubsystem extends SubsystemBase
{
  private final SmartMotorControllerConfig motorConfig = new SmartMotorControllerConfig(this)
      .withControlMode(ControlMode.CLOSED_LOOP)
      .withGearing(new MechanismGearing(GearBox.fromReductionStages(3, 4)))
      .withClosedLoopController(4, 0, 0)
      .withFeedforward(new ArmFeedforward(0, 0.3, 1.2))
      .withTrapezoidalProfile(DegreesPerSecond.of(180), DegreesPerSecondPerSecond.of(90))
      .withSoftLimits(Degrees.of(-25), Degrees.of(105))
      .withStartingPosition(Degrees.of(0))
      .withMomentOfInertia(Feet.of(3), Pounds.of(4))
      .withStatorCurrentLimit(Amps.of(40))
      .withZeroPower(MotorMode.BRAKE)
      .withTelemetry("ArmMotor", TelemetryVerbosity.HIGH);
  private final SmartMotorController motor = new SparkWrapper(spark, DCMotor.getNEO(1), motorConfig);
  private final ArmConfig config = new ArmConfig()
      .withLength(Feet.of(3))
      .withHardLimits(Degrees.of(-30), Degrees.of(110))
      .withTelemetry("Arm", TelemetryVerbosity.HIGH);
  private final Arm arm = new Arm(config, motor);

  @Override
  public void periodic()
  {
    arm.updateTelemetry(); // every loop
  }

  @Override
  public void simulationPeriodic()
  {
    arm.simIterate(); // every loop, in simulation
  }
}
```

`this` is the `Subsystem`. The scheduler calls `periodic()` and `simulationPeriodic()` for you.
{% endtab %}

{% tab title="Commands v3" %}
```java
import yams.commands3.config.SmartMotorControllerConfig;
import yams.commands3.mechanisms.Arm;

public class ArmMechanism implements Mechanism
{
  private final SmartMotorControllerConfig motorConfig = new SmartMotorControllerConfig(this)
      .withControlMode(ControlMode.CLOSED_LOOP)
      .withGearing(new MechanismGearing(GearBox.fromReductionStages(3, 4)))
      .withClosedLoopController(4, 0, 0)
      .withFeedforward(new ArmFeedforward(0, 0.3, 1.2))
      .withTrapezoidalProfile(DegreesPerSecond.of(180), DegreesPerSecondPerSecond.of(90))
      .withSoftLimits(Degrees.of(-25), Degrees.of(105))
      .withStartingPosition(Degrees.of(0))
      .withMomentOfInertia(Feet.of(3), Pounds.of(4))
      .withStatorCurrentLimit(Amps.of(40))
      .withZeroPower(MotorMode.BRAKE)
      .withTelemetry("ArmMotor", TelemetryVerbosity.HIGH);
  private final SmartMotorController motor = new SparkWrapper(spark, DCMotor.getNEO(1), motorConfig);
  private final ArmConfig config = new ArmConfig()
      .withLength(Feet.of(3))
      .withHardLimits(Degrees.of(-30), Degrees.of(110))
      .withTelemetry("Arm", TelemetryVerbosity.HIGH);
  private final Arm arm = new Arm(config, motor);

  public void periodic()
  {
    arm.updateTelemetry(); // every loop
  }

  public void simulationPeriodic()
  {
    arm.simIterate(); // every loop, in simulation
  }
}
```

`this` is the Commands v3 `Mechanism`. A `Mechanism` has no periodic hook, so call `periodic()` yourself from `Robot.robotPeriodic()`, and run `simulationPeriodic()` in its own `Notifier`, started from `Robot.simulationInit()`. [Where the tutorials put things](../tutorials/project-setup/#where-the-tutorials-put-things) shows both.
{% endtab %}
{% endtabs %}

The options in `motorConfig` are explained in [SmartMotorControllerConfig](../reference/editor/config/), and the options in `config` in [ArmConfig](../reference/mechanism-configs/arm-config.md). The hard limits are the arm's physical stops, and the soft limits sit 5° inside them.

{% hint style="info" %}
The gains (`kP` of 4, `kG` of 0.3, `kV` of 1.2) and the profile are placeholders. In simulation the arm should move to its target and hold there, perhaps with some overshoot or sag. On your robot they will be wrong: [Step 5 of the Arm tutorial](../tutorials/arm/tune.md) finds the real ones.
{% endhint %}

## Controlling an arm

An arm takes `Angle` targets: `arm.setAngle(Degrees.of(45))` goes to 45° and holds it, `arm.runTo(Degrees.of(45), Degrees.of(2))` ends when it gets there, and `arm.getAngle()` reads where it is. [Controlling a mechanism](../reference/controlling-a-mechanism.md) lists every command, setpoint function and check, and [When mechanism commands end](run-vs-runto.md) explains how they work with a default command.

## Absolute encoders

An arm that does not know where it is when the robot turns on has to be started in the same place every time. An absolute encoder removes that.

{% hint style="warning" %}
YAMS reads an absolute encoder through the motor controller, so it must be one that motor controller supports: see [Encoders](../reference/editor/config/encoders.md#withexternalencoder). For any other encoder, read it yourself and call `SmartMotorController.setEncoderPosition` once at startup.
{% endhint %}

[How do I use absolute encoders on my arm?](../how-to-mechanisms/how-do-i-use-absolute-encoders-on-my-arm.md) has a complete example for each vendor, including the [zero offset](../reference/editor/config/encoders.md#withexternalencoderzerooffset) that makes the encoder read 0 when the arm is level.

## Arms with a brake

Some arms have a brake module that holds the arm so the motor can stop working. That saves battery and keeps the motor cool on an arm that holds a heavy load for a long time.

<figure><img src="../.gitbook/assets/mech-arm-brake.png" alt="Four stages: when the motor drives, the brake lets go, by itself or on command; the arm moves with runTo; when the motor stops driving, the brake bites and holds the arm; then the motor output is zero, so it draws no current and makes no heat. A graph of motor current shows a spike while moving, a steady holding current until the brake bites, and then almost none, where an arm without a brake would draw holding current forever."><figcaption><p>The brake holds the arm once the motor stops driving it.</p></figcaption></figure>

{% hint style="info" %}
This is not `MotorMode.BRAKE`. [Brake mode](../reference/editor/config/motor-basics.md#withzeropower) is electrical: it resists movement but cannot hold an arm up. A brake module is mechanical, and can.
{% endhint %}

Most brake modules are **automatic**: they bite on their own when the motor stops driving and let go when it drives again, with nothing for your code to command. Some are **manual**, released by a solenoid or a servo that your code controls. Check which yours is, because it changes what you can do. YAMS has no brake class of its own for either kind.

### What to aim for

These are suggestions, not rules. With an automatic brake you do not control when it bites, so you cannot always follow them. Get as close as your brake allows.

* **Let the brake do the holding.** Once the arm has arrived, stop driving the motor so the brake takes the load. An arm that keeps holding its setpoint against the brake gains nothing from having one.
* **Avoid driving against the brake.** A motor pushing on an engaged brake heats up and wears the brake. With an automatic brake some of this is unavoidable at the start of each move, while the brake lets go.
* **Avoid a default command that holds an angle.** It keeps the motor driving, which is the opposite of resting. A default command that rests the motor is ideal.
* **Allow for the brake's timing.** A brake takes a moment to bite and to let go. Expect a small lurch or sag at each end of a move, and allow for it where you can.

### Resting the motor

Resting means an output of zero with the closed loop controller stopped. If the closed loop kept running it would go on driving the arm to its setpoint, against the brake.

The `set(0)` helper command does all of it. It stops the closed loop controller, holds the output at zero, and starts the closed loop again when another command takes over. So you can ignore the closed loop entirely: make `set(0)` the arm's default command, and the arm rests whenever nothing else is moving it.

### With an automatic brake

There is nothing to command on the brake. Move with `runTo`, which ends when the arm arrives, and the default command rests the motor so the brake bites:

{% tabs %}
{% tab title="Commands v2" %}
```java
// In the subsystem
public Command rest()
{
  return arm.set(0); // output 0, closed loop stopped; it restarts when another command takes over
}

public Command moveTo(Angle angle)
{
  return arm.runTo(angle, Degrees.of(2)); // driving releases the brake
}

// In RobotContainer
arm.setDefaultCommand(arm.rest());
```
{% endtab %}

{% tab title="Commands v3" %}
```java
// In the mechanism
public Command rest()
{
  return run(coroutine -> coroutine.await(arm.set(0))) // output 0, closed loop stopped
      .withPriority(Command.LOWEST_PRIORITY)           // so any other command can take over
      .named("Arm Rest");
}

public Command moveTo(Angle angle)
{
  return arm.runTo(angle, Degrees.of(2)); // driving releases the brake
}

// In Robot
arm.setDefaultCommand(arm.rest());
```
{% endtab %}
{% endtabs %}

### With a manual brake

If your code controls the brake, release it before the move and engage it after. The default command still rests the motor:

{% tabs %}
{% tab title="Commands v2" %}
```java
// brake is your own actuator: a Solenoid, a Servo, or anything with an on and off
public Command moveTo(Angle angle)
{
  return runOnce(() -> brake.set(false))         // release
      .andThen(arm.runTo(angle, Degrees.of(2)))  // move
      .finallyDo(() -> brake.set(true));         // engage, even if interrupted
}

// In RobotContainer
arm.setDefaultCommand(arm.rest());               // then the motor rests
```
{% endtab %}

{% tab title="Commands v3" %}
```java
// brake is your own actuator: a Solenoid, a Servo, or anything with an on and off
public Command moveTo(Angle angle)
{
  return run(coroutine -> {
    brake.set(false);                                  // release
    coroutine.await(arm.runTo(angle, Degrees.of(2)));  // move
    brake.set(true);                                   // engage
  }).whenCanceled(() -> brake.set(true)).named("Arm Move And Brake");
}

// In Robot
arm.setDefaultCommand(arm.rest());                     // then the motor rests
```
{% endtab %}
{% endtabs %}

{% hint style="info" %}
Resting the motor with `set(0)` or `arm.setDutyCycleSetpoint(0)` stops a closed loop that runs on the SystemCore. The next move, with a command such as `runTo(...)` or a function such as `arm.setMechanismPositionSetpoint(...)`, starts it again for you. Only when you call `arm.getMotorController()` directly do you start and stop it yourself: see [Driving the motor controller yourself](../how-to-mechanisms/how-do-i-write-my-own-mechanism-command.md#driving-the-motor-controller-yourself).
{% endhint %}

Soft limits still apply while the arm moves, with either kind of brake. The brake does not replace them.

## Configuring and tuning an arm, step by step

Do the configuration steps first, in simulation, then tune the real arm with [live tuning](../how-to-tuning/live-tuning/). Tuning something that is configured wrong wastes the time.

{% hint style="danger" %}
Like all tuning, this can be dangerous. Keep clear of the mechanism, start with low current limits and small moves, and practice in simulation first.
{% endhint %}

### Configure it

{% stepper %}
{% step %}
#### Set the gearing

Give the reduction from the motor to the arm's pivot with [`withGearing`](../reference/editor/config/gearing-and-units.md#withgearing). Check it by moving the arm a known angle by hand and reading `mechanism/position`.
{% endstep %}

{% step %}
#### Make up positive

Apply a small positive voltage with `setVoltage`. If the arm goes down, flip [`withMotorInverted`](../reference/editor/config/motor-basics.md#withmotorinverted). If the angle counts down while it goes up, an [encoder is backwards](../reference/editor/config/encoders.md#withencoderinverted).
{% endstep %}

{% step %}
#### Make level read zero

Hold the arm level. With an absolute encoder, set its [zero offset](../reference/editor/config/encoders.md#withexternalencoderzerooffset) so it reads 0 there. Without one, set a [starting position](../reference/editor/config/encoders.md#withstartingposition) for where the arm rests when the robot turns on.
{% endstep %}

{% step %}
#### Set the limits

Put [`withHardLimits`](../reference/mechanism-configs/shared-options.md#withhardlimits) where the stops are and [`withSoftLimits`](../reference/editor/config/limits-and-protection.md#withsoftlimits) a few degrees inside them.
{% endstep %}

{% step %}
#### Protect the motor

Set a low [stator current limit](../reference/editor/config/limits-and-protection.md#withstatorcurrentlimit-and-withsupplycurrentlimit) to start with, and `MotorMode.BRAKE`.
{% endstep %}

{% step %}
#### Describe it to the simulator

Set [`withLength`](../reference/mechanism-configs/arm-config.md#withlength) and [`withMomentOfInertia`](../reference/editor/config/simulation.md#withmomentofinertia), run it in simulation, and check the arm moves the way you expect before enabling the real one.
{% endstep %}
{% endstepper %}

### What is different when tuning an arm

[Step 5 of the Arm tutorial](../tutorials/arm/tune.md) walks through tuning an arm, and [Tuning order](../how-to-tuning/live-tuning/#tuning-order) lists the order with the keys to watch. What sets an arm apart:

* **kG comes first, found with the arm level.** Gravity pulls hardest on a level arm, and `ArmFeedforward` scales kG by the cosine of the angle, so a kG that holds the arm level holds it everywhere.
* **Zero must really be level.** If the encoder is a few degrees off, kG is applied at the wrong strength and the arm sags at some angles and drifts up at others.
* **Straight up needs no kG.** Check the arm near vertical too, where kP alone holds it.
* **A game piece changes the gains.** Test it loaded. If it needs different gains, put them in a second [slot](../reference/editor/config/closed-loop-control.md#slots): [How do I control a mechanism that carries a game piece?](../how-to-mechanisms/how-do-i-control-a-mechanism-that-carries-a-game-piece.md) shows how.

### Tuning an arm with a brake

Tune the arm itself with the steps above first. If your brake can be held off, hold it off while you do. If it is automatic and cannot be, tune with it in place and expect the gains to carry a little of its drag.

Then look at how the arm and the brake work together. These are things to check and try, not steps that every brake allows:

* **Watch the start of a move.** A lurch or a spike in `current/stator` at the start is the motor pushing on a brake that has not let go yet. A slower start to the [profile](../reference/editor/config/motion-profiles.md#withtrapezoidalprofile) gives an automatic brake time to release. With a manual brake, release it a moment earlier.
* **Watch the end of a move.** If `mechanism/position` sags as the brake bites, the motor let go before the brake held. With a manual brake, engage it before resting the motor. With an automatic one, a sag of a known size can be allowed for in the setpoint.
* **Check the rest.** With the arm braked, `current/stator` should be close to zero. If it is not, something is still commanding the motor: look for a default command that holds a setpoint.
* **Widen the tolerance if it hunts.** A brake holds wherever it bites. If `runTo` takes a long time to finish, the last fraction of a degree is not worth the wait.
* **Keep an eye on `motor/temperature`** over a few minutes of use. If the motor still runs hot, it is working against the brake somewhere.

## ArmConfig options

Every `ArmConfig` option is on the [ArmConfig](../reference/mechanism-configs/arm-config.md) page under this one, with a graphic and an example for each. Options shared by every mechanism are in [Shared options](../reference/mechanism-configs/shared-options.md).

An `Arm` needs its length, hard limits and a starting position before it can be created in simulation: see [what simulation needs](../reference/mechanism-configs/#what-simulation-needs).

## Code reference

{% @github-files/github-code-block url="https://github.com/Yet-Another-Software-Suite/YAMS/blob/master/examples/commands2/simple_robot-2027/src/main/java/first/robot/subsystems/ArmSubsystem.java" %}

{% @github-files/github-code-block url="https://github.com/Yet-Another-Software-Suite/YAMS/blob/master/examples/commands3/simple_robot-2027/src/main/java/first/robot/mechanisms/ArmMechanism.java" %}

{% @github-files/github-code-block url="https://github.com/Yet-Another-Software-Suite/YAMS/blob/master/examples/commands2/simple_robot-2027/src/main/java/first/robot/subsystems/ExponentiallyProfiledArmSubsystem.java" %}
