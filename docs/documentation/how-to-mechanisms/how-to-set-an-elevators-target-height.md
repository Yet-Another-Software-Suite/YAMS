---
icon: elevator
---

# How do I set an elevator's target height?

**Goal:** send your `Elevator` to a height from a button, an autonomous routine or your own code.

This page assumes you already have an `Elevator` in a subsystem. If you do not, build one with the [Elevator tutorial](../tutorials/elevator/) first; it ends with complete files you can copy.

An elevator works the same way as an arm: [How do I set an arm's target angle?](how-to-set-an-arms-target-angle.md) explains `run`, `runTo` and the setpoint method, the default command and the Commands v3 names. This page covers only what differs.

## What differs from an arm

* **The target is a `Distance`** (`org.wpilib.units.measure.Distance`), such as `Meters.of(1)` or `Inches.of(30)`, not an `Angle`. The tolerance of `runTo` is a `Distance` too, such as `Centimeters.of(2)`.
* **The setpoint method is `setMeasurementPositionSetpoint(height)`**, the method for a distance, not `setMechanismPositionSetpoint`.
* **YAMS needs the drum or sprocket size** to turn meters into motor rotations: `withDrumRadius(...)` or `withMechanismCircumference(...)` in the `SmartMotorControllerConfig`. See [How do I tell YAMS my drum or sprocket size?](../how-to-motor-controllers-and-sensors/how-to-find-your-mechanism-circumference.md)
* **The carriage needs `kG` at every height,** since gravity pulls it down the same wherever it is, where an arm's `kG` is scaled by the cosine of its angle.

## Add the commands and bind them

The [Elevator tutorial](../tutorials/elevator/motor-controller.md) sets `.withSoftLimits(Meters.of(0), Meters.of(2))`, so every target here is between 0 and 2 meters.

{% tabs %}
{% tab title="Commands v2" %}
{% code title="ExampleSubsystem.java (fragment)" %}
```java
/** Go to a height and hold it. Never ends on its own. */
public Command run(Distance height) {
  return elevator.run(height);
}

/** Go to a height, then end once the carriage is within tolerance. */
public Command runTo(Distance height, Distance tolerance) {
  return elevator.runTo(height, tolerance);
}

/** Set the target from your own code. */
public void setHeightSetpoint(Distance height) {
  elevator.setMeasurementPositionSetpoint(height);
}
```
{% endcode %}

{% code title="RobotContainer.java (fragment)" %}
```java
m_exampleSubsystem.setDefaultCommand(m_exampleSubsystem.run(Meters.of(0)));
m_driverController.a().whileTrue(m_exampleSubsystem.run(Meters.of(0.5)));
m_driverController.b().whileTrue(m_exampleSubsystem.run(Meters.of(1)));
m_driverController.x().onTrue(m_exampleSubsystem.runTo(Meters.of(1), Centimeters.of(2)));
```
{% endcode %}
{% endtab %}

{% tab title="Commands v3" %}
{% code title="ExampleMechanism.java (fragment)" %}
```java
/** Go to a height and hold it. Never ends on its own. */
public Command setHeight(Distance height) {
  return elevator.run(height);
}

/** Go to a height, then end once the carriage is within tolerance. */
public Command moveTo(Distance height, Distance tolerance) {
  return elevator.runTo(height, tolerance);
}

/** Hold a height at the lowest priority, so any other command can take over. For the default command. */
public Command hold(Distance height) {
  return run(coroutine -> coroutine.await(elevator.run(height)))
      .withPriority(Command.LOWEST_PRIORITY)
      .named("Elevator Hold");
}

/** Set the target from your own code. */
public void setHeightSetpoint(Distance height) {
  elevator.setMeasurementPositionSetpoint(height);
}
```
{% endcode %}

{% code title="ExampleTeleop.java (fragment)" %}
```java
// In Robot():
exampleMechanism.setDefaultCommand(exampleMechanism.hold(Meters.of(0)));
// In the opmode constructor:
controller.a().whileTrue(robot.exampleMechanism.setHeight(Meters.of(0.5)));
controller.b().whileTrue(robot.exampleMechanism.setHeight(Meters.of(1)));
controller.x().onTrue(robot.exampleMechanism.moveTo(Meters.of(1), Centimeters.of(2)));
```
{% endcode %}
{% endtab %}

{% tab title="C++" %}
{% code title="cpp/subsystems/ElevatorSubsystem.cpp (fragment)" %}
```cpp
wpi::cmd::CommandPtr ElevatorSubsystem::Run(wpi::units::meter_t height) {
  return m_elevator->Run(height);
}

wpi::cmd::CommandPtr ElevatorSubsystem::RunTo(wpi::units::meter_t height, wpi::units::meter_t tolerance) {
  return m_elevator->RunTo(height, tolerance);
}

void ElevatorSubsystem::SetHeightSetpoint(wpi::units::meter_t height) {
  m_elevator->SetMeasurementPositionSetpoint(height);
}
```
{% endcode %}

{% code title="cpp/RobotContainer.cpp (fragment)" %}
```cpp
elevator.SetDefaultCommand(elevator.Run(0_m));
driverController.FaceDown().WhileTrue(elevator.Run(0.5_m));
driverController.FaceRight().WhileTrue(elevator.Run(1_m));
driverController.FaceLeft().OnTrue(elevator.RunTo(1_m, 2_cm));
```
{% endcode %}
{% endtab %}
{% endtabs %}

## Check it

1. Start the simulation (or enable the robot) and open **NetworkTables > Mechanisms > Elevator > mechanism** in your dashboard. `Elevator` is the name you gave `ElevatorConfig.withTelemetry`.
2. Hold B. The drawn carriage should rise to 1 meter and stay there.
3. Release B. The default command takes over and the carriage returns to the bottom.

## If it did not work

* **The carriage stops short of the target.** The target is outside your soft limits. YAMS clamps the setpoint to the nearest limit and prints a warning in the Driver Station. A `runTo` to a clamped target never ends.
* **The carriage sags below the target, or slides down when you let go.** `kG` is still 0. The tutorial's gains are placeholders. Find `kG` first (the voltage that holds the carriage still), then `kP`, with the [Elevator tutorial's Tune it step](../tutorials/elevator/tune.md).
* **The carriage goes to the wrong height, off by the same factor every time.** The drum radius, circumference or cascading stage count is wrong. See [How do I tell YAMS my drum or sprocket size?](../how-to-motor-controllers-and-sensors/how-to-find-your-mechanism-circumference.md)
* **Something else.** See [Troubleshooting](../tutorials/troubleshooting.md).
