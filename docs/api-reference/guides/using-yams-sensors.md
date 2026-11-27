# Using YAMS sensors

**Goal:** Wrap a sensor, such as a beam break or a limit switch, in a YAMS `Sensor`, so your code reads it the same way on the robot and in simulation, and so you can say what it reads in simulation.

***

## Why wrap a sensor

In simulation there is no real sensor, so code that waits for a beam break or a limit switch never sees it change. A YAMS `Sensor` fixes that. Your code reads it with the same call everywhere, and:

* **On the robot**, it returns the real reading, from the supplier you give it.
* **In simulation**, it returns the value shown in the Sim GUI, which starts at the default you give it and which you can change by hand. You can also script it: make it read a value while a condition is true, such as the arm being at the angle where it breaks the beam.

[Sensors](https://yams.yassrobotics.com/documentation/details/sensors) in the documentation explains the idea with pictures. This guide is the API for it in Java and C++.

The examples use a beam break that detects a game piece. A beam break usually reads `true` while the beam is clear and `false` when something blocks it, so the code inverts it: the `NotePresent` field reads `true` when a note is there. If your sensor reads `true` when it detects something, drop the `!`.

***

## Create and read a sensor

{% tabs %}
{% tab title="Java" %}
{% stepper %}
{% step %}
### Create the sensor

Give the sensor a name, and add one field for each reading: a field name, a supplier for the real reading, and the default it starts at in simulation.

```java
import org.wpilib.hardware.discrete.DigitalInput;
import yams.core.mechanisms.config.SensorConfig;
import yams.core.motorcontrollers.simulation.Sensor;

DigitalInput noteDetector = new DigitalInput(1);  // beam break on DIO 1

Sensor intakeSensor = new SensorConfig("IntakeSensor")
    // true when a note blocks the beam; starts false in simulation
    .withField("NotePresent", () -> !noteDetector.get(), false)
    .getSensor();
```

`getSensor()` builds the `Sensor` the first time and returns the same one after that.

**Check it:** run the simulation and open **Other Devices** in the Sim GUI. The sensor shows as **Sensor\[IntakeSensor]**, with a `NotePresent` field reading `false`.
{% endstep %}

{% step %}
### Read it

```java
boolean hasNote = intakeSensor.getAsBoolean("NotePresent");
```

Read a field with the getter for its type: `getAsBoolean`, `getAsDouble`, `getAsInt` or `getAsLong`.

**Check it:** change `NotePresent` to `true` in the Sim GUI, and `hasNote` becomes `true`. Publish it with `Telemetry.log(...)` or bind a trigger to it to see it change.
{% endstep %}
{% endstepper %}

### The complete subsystem

```java
package first.robot.subsystems;

import org.wpilib.command2.SubsystemBase;
import org.wpilib.command2.button.Trigger;
import org.wpilib.hardware.discrete.DigitalInput;
import yams.core.mechanisms.config.SensorConfig;
import yams.core.motorcontrollers.simulation.Sensor;

public class IntakeSubsystem extends SubsystemBase {

    private final DigitalInput noteDetector = new DigitalInput(1);
    private final Sensor       intakeSensor = new SensorConfig("IntakeSensor")
        // true when a note blocks the beam; starts false in simulation
        .withField("NotePresent", () -> !noteDetector.get(), false)
        .getSensor();

    public boolean hasNote() {
        return intakeSensor.getAsBoolean("NotePresent");
    }

    /** True while a note is in the intake. */
    public Trigger noteDetected() {
        return new Trigger(this::hasNote);
    }
}
```

In a Commands v3 project the sensor code is the same: the class implements `org.wpilib.command3.Mechanism` instead of extending `SubsystemBase`, and `Trigger` comes from `org.wpilib.command3`.
{% endtab %}

{% tab title="C++" %}
{% stepper %}
{% step %}
### Create the sensor

Give the sensor a name, and add one field for each reading: a field name, a supplier for the real reading, and the default it starts at in simulation. Keep the `SimSensorConfig` and the hardware as members: the sensor reads them for as long as it is used.

```cpp
#include <wpi/hardware/discrete/DigitalInput.hpp>
#include <yams/mechanisms/config/SimSensorConfig.hpp>
#include <yams/motorcontrollers/simulation/Sensor.hpp>

wpi::DigitalInput noteDetector{1};  // beam break on DIO 1

yams::mechanisms::config::SimSensorConfig intakeSensor{"IntakeSensor"};
intakeSensor
    // true when a note blocks the beam; starts false in simulation
    .WithField("NotePresent", [&noteDetector] { return !noteDetector.Get(); }, false);
```

`GetSensor()` builds the `Sensor` the first time you call it and returns the same one after that.

**Check it:** run the simulation and open **Other Devices** in the Sim GUI. Once your code has called `GetSensor()`, the sensor shows as **Sensor\[IntakeSensor]**, with a `NotePresent` field reading `false`.
{% endstep %}

{% step %}
### Read it

```cpp
bool hasNote = intakeSensor.GetSensor().GetAsBoolean("NotePresent");
```

Read a field with the getter for its type: `GetAsBoolean`, `GetAsDouble`, `GetAsInt` or `GetAsLong`.

**Check it:** change `NotePresent` to `true` in the Sim GUI, and `hasNote` becomes `true`.
{% endstep %}
{% endstepper %}

### The complete subsystem

```cpp
#include <wpi/commands2/SubsystemBase.hpp>
#include <wpi/commands2/button/Trigger.hpp>
#include <wpi/hardware/discrete/DigitalInput.hpp>
#include <yams/mechanisms/config/SimSensorConfig.hpp>
#include <yams/motorcontrollers/simulation/Sensor.hpp>

class IntakeSubsystem : public wpi::cmd::SubsystemBase {
 public:
    IntakeSubsystem() {
        intakeSensor_
            // true when a note blocks the beam; starts false in simulation
            .WithField("NotePresent", [this] { return !noteDetector_.Get(); }, false);
    }

    bool HasNote() {
        return intakeSensor_.GetSensor().GetAsBoolean("NotePresent");
    }

    /** True while a note is in the intake. */
    wpi::cmd::Trigger NoteDetected() {
        return wpi::cmd::Trigger{[this] { return HasNote(); }};
    }

 private:
    wpi::DigitalInput noteDetector_{1};
    yams::mechanisms::config::SimSensorConfig intakeSensor_{"IntakeSensor"};
};
```
{% endtab %}
{% endtabs %}

***

## Say what it reads in simulation

Changing a field by hand is fine for a quick test. To make the simulation behave like the robot, give the field a **simulated value**: a value it reads while a condition is true. When the condition turns false, the field goes back to the value it had before. On the real robot, simulated values are ignored.

### While a condition is true

This is the one to use most of the time. The condition is any `BooleanSupplier` (Java) or `std::function<bool()>` (C++), such as a mechanism being where it would break the beam.

{% tabs %}
{% tab title="Java" %}
```java
Sensor intakeSensor = new SensorConfig("IntakeSensor")
    .withField("NotePresent", () -> !noteDetector.get(), false)
    // In simulation, a note is present while the intake roller is spinning
    .withSimulatedValue("NotePresent", () -> roller.getSpeed().gt(RPM.of(100)), true)
    .getSensor();
```
{% endtab %}

{% tab title="C++" %}
```cpp
intakeSensor_
    .WithField("NotePresent", [this] { return !noteDetector_.Get(); }, false)
    // In simulation, a note is present while the intake roller is spinning
    .WithSimulatedValue("NotePresent", [this] { return roller_->GetVelocity() > 600_deg_per_s; }, true);
```
{% endtab %}
{% endtabs %}

Here `roller` is the intake's YAMS `FlyWheel`. Use whatever condition matches your robot: the [`simple_robot-2027` arm](using-yams-sensors.md#examples) uses the arm being near 40 degrees.

**Check it:** in simulation, run the intake. `NotePresent` turns `true` in the Sim GUI while the roller spins, and back to `false` when it stops.

### During a match time window

`withSimulatedValue(field, start, end, value)` makes the field read `value` while the **match time** is between `start` and `end`. Match time is WPILib's `Timer.getMatchTime()`: the time **left** in the current period, counting down, not the time since the period started. So a window of 5 s to 8 s is active from 8 seconds left until 5 seconds left.

{% hint style="warning" %}
When nothing is running a match timer, match time reads -1 and a time window never opens. WPILib's simulation matches a Driver Station with no FMS, where match time only counts down in practice mode. If the window never fires, use a condition instead.
{% endhint %}

{% tabs %}
{% tab title="Java" %}
```java
import static org.wpilib.units.Units.Seconds;

// In simulation, a note is present from 8 s left until 5 s left in the period
.withSimulatedValue("NotePresent", Seconds.of(5), Seconds.of(8), true)
```
{% endtab %}

{% tab title="C++" %}
```cpp
// In simulation, a note is present from 8 s left until 5 s left in the period
.WithSimulatedValue("NotePresent", 5_s, 8_s, true)
```
{% endtab %}
{% endtabs %}

**Check it:** with the match timer running, `NotePresent` turns `true` when 8 seconds are left and back to `false` when 5 are left.

### After the sensor is built

To add a simulated value later, call `addSimTrigger` (Java) or `AddSimTrigger` (C++) on the `Sensor`, with the value as a `HALValue`:

{% tabs %}
{% tab title="Java" %}
```java
import org.wpilib.driverstation.RobotState;
import org.wpilib.hardware.hal.HALValue;

intakeSensor.addSimTrigger("NotePresent", HALValue.makeBoolean(true), RobotState::isAutonomous);
```
{% endtab %}

{% tab title="C++" %}
```cpp
#include <wpi/driverstation/RobotState.hpp>
#include <yams/motorcontrollers/simulation/SensorData.hpp>

intakeSensor_.GetSensor().AddSimTrigger(
    "NotePresent",
    yams::motorcontrollers::simulation::SensorData::Convert(true),
    [] { return wpi::RobotState::IsAutonomous(); });
```
{% endtab %}
{% endtabs %}

**Check it:** enable autonomous in the Sim GUI, and `NotePresent` reads `true` for as long as autonomous runs.

When several simulated values are active at once, the one added first wins. A simulated value also wins over a value you typed in the Sim GUI, until its condition turns false.

***

## Several fields on one sensor

One sensor can hold several related readings. In this fragment, `encoder` is the arm's encoder and `upperLimit` and `lowerLimit` are `DigitalInput` limit switches:

{% tabs %}
{% tab title="Java" %}
```java
Sensor armSensor = new SensorConfig("Arm")
    .withField("PositionDegrees", encoder::getPosition, 0.0)
    .withField("AtUpperLimit",    upperLimit::get,      false)
    .withField("AtLowerLimit",    lowerLimit::get,      false)
    .getSensor();

double  position = armSensor.getAsDouble("PositionDegrees");
boolean atTop    = armSensor.getAsBoolean("AtUpperLimit");
```
{% endtab %}

{% tab title="C++" %}
```cpp
armSensor_
    .WithField("PositionDegrees", [this] { return encoder_.GetPosition(); }, 0.0)
    .WithField("AtUpperLimit",    [this] { return upperLimit_.Get(); },     false)
    .WithField("AtLowerLimit",    [this] { return lowerLimit_.Get(); },     false);

double position = armSensor_.GetSensor().GetAsDouble("PositionDegrees");
bool   atTop    = armSensor_.GetSensor().GetAsBoolean("AtUpperLimit");
```
{% endtab %}
{% endtabs %}

The supplier and the default set the field's type: a `double` supplier with `0.0` makes a `double` field, and a `boolean` supplier with `false` a `boolean` field. Read each field with the getter for its type.

***

## If it did not work

* **The sensor is not in the Sim GUI.** Look under **Other Devices**, not NetworkTables. In C++, the sensor only appears once `GetSensor()` has been called.
* **In simulation, the field never changes when the real sensor would.** That is expected: in simulation a sensor reads the Sim GUI and its simulated values, never the supplier. Add a simulated value for it.
* **A time window never fires.** Match time is -1 when no match timer runs, and it counts down. Use a condition instead, as in [While a condition is true](using-yams-sensors.md#while-a-condition-is-true).
* **The value is the opposite of what you expect.** Your sensor reads `true` when it detects something, so remove the `!` from the supplier.
* **A value typed in the Sim GUI does not stick.** A simulated value is active and wins. Wait until its condition is false, or remove it.
* **It throws when you read a field.** The getter does not match the field's type. A field made with a `false` default must be read with `getAsBoolean` / `GetAsBoolean`; Java throws an `IllegalStateException`, and C++ a `std::logic_error`.

***

## Examples

The `simple_robot-2027` arm declares a beam break with a time window and a condition:

{% @github-files/github-code-block url="https://github.com/Yet-Another-Software-Suite/YAMS/blob/master/examples/commands2/simple_robot-2027/src/main/java/first/robot/subsystems/ArmSubsystem.java#L87-L92" %}

***

## Related pages

* [Sensors](https://yams.yassrobotics.com/documentation/details/sensors): what a sensor is and how it gets its value, with pictures
* [How do I home a mechanism with limit switches?](https://yams.yassrobotics.com/documentation/how-to-mechanisms/how-do-i-home-a-mechanism-with-limit-switches)
* [SensorConfig (Java)](../java-reference/config/sensor-config.md)
* [SimSensorConfig (C++)](../c++-reference/config/sim-sensor-config.md)
* [SensorConfig (C++)](../c++-reference/config/sensorconfig.md): how the Java `SensorConfig` maps to C++
* [Simulation](simulation.md)
