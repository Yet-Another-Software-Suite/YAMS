# Simulation

**Goal:** Enable physics-accurate simulation for YAMS mechanisms in a WPILib robot project.

***

{% stepper %}
{% step %}
#### Provide physical parameters in the configs

YAMS builds its simulation model from fields on the mechanism config and on `SmartMotorControllerConfig`. Set the fields that match your mechanism's geometry and mass:

| Mechanism  | Mechanism config                                                 | `SmartMotorControllerConfig`                                                                                         |
| ---------- | ---------------------------------------------------------------- | -------------------------------------------------------------------------------------------------------------------- |
| `Arm`      | `ArmConfig.withLength(...)`, `.withHardLimits(...)`              | `.withMomentOfInertia(length, mass)`, `.withStartingPosition(Angle)`, `.withGearing(...)`                            |
| `Elevator` | `ElevatorConfig.withCarriageWeight(...)`, `.withHardLimits(...)` | `.withMechanismCircumference(...)` or `.withDrumRadius(...)`, `.withStartingPosition(Distance)`, `.withGearing(...)` |
| `FlyWheel` | none required (`FlyWheelConfig.withDiameter(...)` is optional)   | `.withMomentOfInertia(...)`, `.withGearing(...)`                                                                     |
| `Pivot`    | `PivotConfig.withHardLimits(...)`                                | `.withMomentOfInertia(...)`, `.withStartingPosition(Angle)`, `.withGearing(...)`                                     |

The C++ equivalents are `ArmConfig::WithArmLength`, `ElevatorConfig::WithCarriageMass`, `SmartMotorControllerConfig::WithMOI`, and `SmartMotorControllerConfig::WithMechanismCircumference`.

You do not need to manually construct a sim supplier. The mechanism creates an `ArmSimSupplier`, `ElevatorSimSupplier`, or `DCMotorSimSupplier` and hands it to the motor controller when it is constructed in simulation.

**Check it:** run the simulation. The code starts without an exception. If a value is missing, the mechanism's constructor throws and the message names it, as the warning under [Notes](simulation.md#notes) lists.
{% endstep %}

{% step %}
#### Optionally tune simulation-only settings

`SmartMotorControllerConfig` has simulation-only overrides, so the simulated mechanism can use different gains or a different starting position than the real one:

```java
motorConfig
    .withSimClosedLoopController(4, 0, 0)                    // Placeholder PID used only in simulation
    .withSimFeedforward(new ArmFeedforward(0, 0.3, 0.5))     // Placeholder feedforward used only in simulation
    .withSimStartingPosition(Degrees.of(0))                  // where the simulated mechanism starts
    .withSimulationPeriod(Milliseconds.of(5));               // physics step, defaults to 20 ms
```

For most use cases these are not needed.

**Check it:** if you set any of these, the simulated mechanism starts at the simulation starting position and responds to the simulation gains, while the real robot is unchanged.
{% endstep %}

{% step %}
#### Call `updateTelemetry()` from `periodic()`

`updateTelemetry()` publishes current state (both real and simulated) to NetworkTables so your dashboard reflects simulated values, and applies anything you change through live tuning.

```java
@Override
public void periodic() {
    arm.updateTelemetry();
}
```

In Commands v3, call the mechanism's `periodic()` from `Robot.robotPeriodic()`, before `scheduler.run()`.

**Check it:** the mechanism's values show in NetworkTables under **Mechanisms**, named after its `withTelemetry(...)`.
{% endstep %}

{% step %}
#### Call `simIterate()` from `simulationPeriodic()`

`simIterate()` advances the physics model by one simulation period (20 ms unless changed with `withSimulationPeriod`) and writes the simulated sensor values back to the motor controller.

{% tabs %}
{% tab title="Commands v2" %}
```java
@Override
public void simulationPeriodic() {
    arm.simIterate();
}
```
{% endtab %}

{% tab title="Commands v3" %}
A Commands v3 `Mechanism` is not polled for you. Start its simulation in `Robot.simulationInit()`, in its own `Notifier`, so it runs every loop without adding to the robot loop's time:

```java
@Override
public void simulationInit() {
    startSimulation("Arm", arm::simulationPeriodic);   // calls the YAMS Arm's simIterate()
}

/** Run a mechanism's simulation in its own Notifier, every robot loop period. */
private void startSimulation(String name, Runnable simulationPeriodic) {
    Notifier notifier = new Notifier(simulationPeriodic);
    notifier.setName(name + " Simulation");
    notifier.startPeriodic(getPeriod());
    simulationNotifiers.add(notifier);   // a List<Notifier> field, so the Notifiers are not garbage collected
}
```

This is the same helper as in the Commands v3 examples and in [Update it every loop](https://yams.yassrobotics.com/documentation/tutorials/arm/mechanism#update-it-every-loop) in the Arm tutorial.
{% endtab %}
{% endtabs %}

**Check it:** command the mechanism to a new setpoint in simulation, and its position or velocity in telemetry changes. If it never moves, this call is missing.
{% endstep %}

{% step %}
#### View the Mechanism2d visualization

YAMS publishes a `Mechanism2d` for each mechanism at `Mechanisms/<name>/mechanism`, where `<name>` is the telemetry name on the mechanism config (for example `"Shoulder"`). Telemetry logs a `Mechanism2d` as it is when it is logged, so each mechanism logs it again on every visualization update, which `simIterate()` runs. In the Sim GUI, open **NetworkTables -> Mechanisms -> Shoulder -> mechanism**. In AdvantageScope, drag `Mechanisms/Shoulder/mechanism` onto a **Mechanism** tab.

**Check it:** the mechanism is drawn at its starting position, and moves when you command it.
{% endstep %}
{% endstepper %}

***

## Shared battery simulation

Every YAMS motor controller reports its simulated supply current to one shared, static `BatterySim` (`yams.core.motorcontrollers.simulation.BatterySim` in Java, `yams::motorcontrollers::simulation::BatterySim` in C++). The loaded battery voltage is computed from the sum of all of those currents, so a drivetrain pulling hard sags the voltage seen by your arm, elevator, and shooter, just like on a real robot. There is nothing to set up: it works as soon as more than one YAMS mechanism is simulated.

By default the battery is an ideal 12 V source behind a fixed 20 milliohm internal resistance. To also model the battery draining over a match, enable discharge simulation once at startup:

{% tabs %}
{% tab title="Java" %}
```java
import yams.core.motorcontrollers.simulation.BatterySim;

// 18 Ah battery, 12.9 V fully charged, 20 milliohm internal resistance
BatterySim.enableDischarge(18.0, Volts.of(12.9), MilliOhms.of(20));

double stateOfCharge = BatterySim.getStateOfCharge();  // 1.0 = full, 0.0 = empty
```
{% endtab %}

{% tab title="C++" %}
```cpp
#include "yams/motorcontrollers/simulation/BatterySim.hpp"

using yams::motorcontrollers::simulation::BatterySim;

// 18 Ah battery, 12.9 V fully charged, 20 milliohm internal resistance
BatterySim::EnableDischarge(18.0, 12.9_V, 0.02_Ohm);

double stateOfCharge = BatterySim::GetStateOfCharge();  // 1.0 = full, 0.0 = empty
```
{% endtab %}
{% endtabs %}

With discharge enabled, the open circuit voltage falls and the internal resistance rises as the state of charge drops. `resetDischarge()` refills the battery and `disableDischarge()` returns to the fixed model.

## A disabled robot's simulated motors get no power

While the robot is disabled, the `ArmSimSupplier`, `DCMotorSimSupplier` and `ElevatorSimSupplier` apply 0 V to the simulated motor on every simulation step, as a real disabled robot does. An arm falls and a flywheel coasts down, instead of holding the last voltage it was given.

A closed loop that YAMS runs on the SystemCore (an [LQR](../java-reference/math/lqr-controller.md), an exponential profile, a velocity trapezoidal profile, or any closed loop on an [A301](../java-reference/motor-controllers/a301-wrapper.md)) sends no voltage while the robot is disabled. It follows the mechanism instead, resetting its PID, motion profile and LQR to where the mechanism is, so when the robot is enabled it starts from there rather than jumping back to where the mechanism was. Without a setpoint, the motion profile also follows the mechanism, so the first setpoint after enabling is profiled from where the mechanism is.

**Check it:** command an arm up, then disable the robot, and the arm falls in the Mechanism2d. Enable it and command it again, and it moves from where it fell, without first jumping back. Commands end when the robot is disabled, so a button still held must be pressed again.

## SPARK closed loop runs at 1 kHz in simulation

A real SPARK MAX or SPARK FLEX closes its loop onboard every millisecond. `SparkWrapper` simulates that: inside each `simIterate()` call it steps the simulated SPARK's closed loop once per millisecond across the simulation period (20 steps for the default 20 ms period), instead of once per period. Gains tuned in simulation therefore behave much closer to the real controller. No configuration is needed.

## If it did not work

* **The mechanism is not in the NetworkTables menu.** Look under **Mechanisms**, not **SmartDashboard**, and check that `updateTelemetry()` runs every loop.
* **It never moves in simulation.** `simIterate()` is not running: check `simulationPeriodic()` (Commands v2) or `simulationInit()` (Commands v3).
* **It throws when the code starts.** A physical value from the table in the first step is missing.
* **It falls or stops when the robot is disabled.** That is expected: a disabled robot's simulated motors get 0 V.

[Troubleshooting](https://yams.yassrobotics.com/documentation/tutorials/troubleshooting) covers more.

***

## Notes

{% hint style="info" %}
The three sim suppliers YAMS provides internally are `ArmSimSupplier`, `ElevatorSimSupplier`, and `DCMotorSimSupplier`. For most use cases, you do not construct these directly; the mechanism instantiates them automatically from the physical parameters in the configs (arm length, carriage weight, MOI, etc.).
{% endhint %}

{% hint style="warning" %}
In simulation, the `Arm` constructor throws an `ArmConfigurationException` if the arm length, hard limits, or starting position are missing, and the `Elevator` constructor throws an `ElevatorConfigurationException` if the carriage weight or hard limits are missing, or, in Java, a `SmartMotorControllerConfigurationException` if the starting position is (C++ throws an `ElevatorConfigurationException` for that too). The starting position must also lie inside the hard limits.
{% endhint %}

***

## Complete periodic example

```java
public class ShoulderSubsystem extends SubsystemBase {

    private final Arm arm;

    // ... constructor omitted, see Building an Arm

    @Override
    public void periodic() {
        arm.updateTelemetry();   // publishes angles, voltages, setpoints to NT
    }

    @Override
    public void simulationPeriodic() {
        arm.simIterate();        // steps physics, writes simulated encoder back to motor
    }
}
```

***

## Examples

The `simple_robot-2027` arm demonstrates physics simulation with `simIterate()` and a `Mechanism2d` display:

{% @github-files/github-code-block url="https://github.com/Yet-Another-Software-Suite/YAMS/blob/master/examples/commands2/simple_robot-2027/src/main/java/first/robot/subsystems/ArmSubsystem.java" %}

The elevator in the same project shows the same pattern for linear mechanisms:

{% @github-files/github-code-block url="https://github.com/Yet-Another-Software-Suite/YAMS/blob/master/examples/commands2/simple_robot-2027/src/main/java/first/robot/subsystems/ElevatorSubsystem.java" %}

***

## Related pages

* [ArmConfig](../java-reference/config/arm-config.md)
* [ElevatorConfig](../java-reference/config/elevator-config.md)
* [FlyWheelConfig](../java-reference/config/flywheel-config.md)
* [Building an Arm](building-an-arm.md)
* [Building an Elevator](building-an-elevator.md)
* [Building a FlyWheel](building-a-flywheel.md)
