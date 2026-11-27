---
icon: hand-fist
---

# How do I set an arm's target angle?

**Goal:** send your `Arm` to an angle from a button, an autonomous routine or your own code.

This page assumes you already have an `Arm` in a subsystem. If you do not, build one with the [Arm tutorial](../tutorials/arm/) first; it ends with complete files you can copy.

YAMS drives the arm to a target with **closed loop** control: you give it an angle, and its controller keeps adjusting the voltage until the encoder says the arm is there. **Open loop** commands (`set` and `setVoltage`) send an output straight to the motor instead. [Open loop and closed loop](../tutorials/how-yams-drives-a-motor.md#open-loop-and-closed-loop) explains both.

An arm's target is an `Angle` (`org.wpilib.units.measure.Angle`), for example `Degrees.of(25)`. YAMS gives you three ways to set it:

| Use                                       | When                                                                    | Ends?                                                                         |
| ----------------------------------------- | ----------------------------------------------------------------------- | ----------------------------------------------------------------------------- |
| `arm.run(angle)`                          | Hold an angle for as long as a button is held, or as a default command. | No. It runs until something interrupts it.                                    |
| `arm.runTo(angle, tolerance)`             | A step in a sequence or autonomous routine: go there, then move on.     | Yes, once the arm has stayed within `tolerance` of the angle for 0.1 seconds. |
| `arm.setMechanismPositionSetpoint(angle)` | Your own code, inside a command you already wrote.                      | It is a method, not a command. It sets the target once.                       |

All three start the closed loop controller again if an open loop command stopped it. [When mechanism commands end](../understanding/run-vs-runto.md) explains `run` and `runTo` with the default command.

## Add the commands to your subsystem

These methods go in the subsystem (Commands v2) or mechanism (Commands v3) that owns the `arm` field. They are fragments: the rest of the class is in the [Arm tutorial](../tutorials/arm/commands.md).

{% tabs %}
{% tab title="Commands v2" %}
{% code title="ExampleSubsystem.java (fragment)" %}
```java
/** Go to an angle and hold it. Never ends on its own. */
public Command run(Angle angle) {
  return arm.run(angle);
}

/** Go to an angle, then end once the arm is within tolerance. */
public Command runTo(Angle angle, Angle tolerance) {
  return arm.runTo(angle, tolerance);
}

/** Set the target from your own code. */
public void setAngleSetpoint(Angle angle) {
  arm.setMechanismPositionSetpoint(angle);
}
```
{% endcode %}
{% endtab %}

{% tab title="Commands v3" %}
{% code title="ExampleMechanism.java (fragment)" %}
```java
/** Go to an angle and hold it. Never ends on its own. */
public Command setAngle(Angle angle) {
  return arm.run(angle);
}

/** Go to an angle, then end once the arm is within tolerance. */
public Command moveTo(Angle angle, Angle tolerance) {
  return arm.runTo(angle, tolerance);
}

/** Hold an angle at the lowest priority, so any other command can take over. For the default command. */
public Command hold(Angle angle) {
  return run(coroutine -> coroutine.await(arm.run(angle)))
      .withPriority(Command.LOWEST_PRIORITY)
      .named("Arm Hold");
}

/** Set the target from your own code. */
public void setAngleSetpoint(Angle angle) {
  arm.setMechanismPositionSetpoint(angle);
}
```
{% endcode %}
{% endtab %}

{% tab title="C++" %}
{% code title="cpp/subsystems/ArmSubsystem.cpp (fragment)" %}
```cpp
// Go to an angle and hold it. Never ends on its own.
wpi::cmd::CommandPtr ArmSubsystem::Run(wpi::units::degree_t angle) {
  return m_arm->Run(angle);
}

// Go to an angle, then end once the arm is within tolerance.
wpi::cmd::CommandPtr ArmSubsystem::RunTo(wpi::units::degree_t angle, wpi::units::degree_t tolerance) {
  return m_arm->RunTo(angle, tolerance);
}

// Set the target from your own code.
void ArmSubsystem::SetAngleSetpoint(wpi::units::degree_t angle) {
  m_arm->SetMechanismPositionSetpoint(angle);
}
```
{% endcode %}

Declare the three in `ArmSubsystem.hpp`, as the [Arm tutorial](../tutorials/arm/commands.md) does for `Run` and `RunTo`.
{% endtab %}
{% endtabs %}

## Bind them to buttons

Every target must be inside your soft limits. The [Arm tutorial](../tutorials/arm/motor-controller.md) sets `.withSoftLimits(Degrees.of(-20), Degrees.of(30))` in the `SmartMotorControllerConfig`, so -5 and 25 degrees are both reachable.

{% tabs %}
{% tab title="Commands v2" %}
{% code title="RobotContainer.java (fragment)" %}
```java
// Hold the arm at 0 degrees when nothing else is using it.
m_exampleSubsystem.setDefaultCommand(m_exampleSubsystem.run(Degrees.of(0)));

// Hold -5 or 25 degrees while A or B is held.
m_driverController.a().whileTrue(m_exampleSubsystem.run(Degrees.of(-5)));
m_driverController.b().whileTrue(m_exampleSubsystem.run(Degrees.of(25)));

// Go to 25 degrees, then (once within 2 degrees) do the next thing.
m_driverController.x().onTrue(
    m_exampleSubsystem.runTo(Degrees.of(25), Degrees.of(2))
        .andThen(Commands.print("Arm is at 25 degrees")));
```
{% endcode %}
{% endtab %}

{% tab title="Commands v3" %}
{% code title="ExampleTeleop.java (fragment)" %}
```java
// In Robot(): hold the arm at 0 degrees when nothing else is using it.
exampleMechanism.setDefaultCommand(exampleMechanism.hold(Degrees.of(0)));

// In the opmode constructor: hold -5 or 25 degrees while A or B is held.
controller.a().whileTrue(robot.exampleMechanism.setAngle(Degrees.of(-5)));
controller.b().whileTrue(robot.exampleMechanism.setAngle(Degrees.of(25)));

// Go to 25 degrees on a press of X. The command ends once the arm is within 2 degrees,
// and the default command takes over.
controller.x().onTrue(robot.exampleMechanism.moveTo(Degrees.of(25), Degrees.of(2)));
```
{% endcode %}
{% endtab %}

{% tab title="C++" %}
{% code title="cpp/RobotContainer.cpp (fragment)" %}
```cpp
// Hold the arm at 0 degrees when nothing else is using it.
arm.SetDefaultCommand(arm.Run(0_deg));

// Hold -5 or 25 degrees while A or B is held.
driverController.FaceDown().WhileTrue(arm.Run(-5_deg));
driverController.FaceRight().WhileTrue(arm.Run(25_deg));

// Go to 25 degrees on a press of X. The command ends once the arm is within 2 degrees.
driverController.FaceLeft().OnTrue(arm.RunTo(25_deg, 2_deg));
```
{% endcode %}
{% endtab %}
{% endtabs %}

In Commands v3 a `Mechanism` already has a `run(...)` method that takes a coroutine, so the wrappers are called `setAngle`, `moveTo` and `hold`, as in the tutorial.

If the target comes from a `Supplier<Angle>` (for example an aiming calculation), `run(supplier)` and `runTo(supplier, tolerance)` follow the supplier every loop. `runTo` ends once the arm has been within the tolerance of the supplier's current value for 0.1 seconds.

## Check it

1. Start the simulation (or enable the robot) and open **NetworkTables > Mechanisms > Arm > mechanism** in your dashboard. `Arm` is the name you gave `ArmConfig.withTelemetry`.
2. Hold B. The drawn arm should swing to 25 degrees and stay there.
3. Release B. The default command takes over and the arm returns to 0 degrees.

## If it did not work

* **The arm stops short of the target.** The target is outside your soft limits. YAMS clamps the setpoint to the nearest limit and prints a warning ("Setpoint is higher than Mechanism ... upper limit") in the Driver Station. Widen the soft limits or pick a target inside them. A `runTo` to a clamped target never ends, because the arm never gets within tolerance.
* **The arm sags below the target, or falls when you let go.** `kG` is still 0. The gains in the tutorial are placeholders; find `kG` and then `kP` with [Step 5: Tune it](../tutorials/arm/tune.md).
* **`runTo` never ends.** The tolerance is tighter than the arm can hold. Try 2 or 3 degrees, and tune `kP`.
* **Something else.** See [Troubleshooting](../tutorials/troubleshooting.md).

## Read more

Tune the arm so it reaches the target quickly and holds it: [Arm tutorial: Tune it](../tutorials/arm/tune.md).
