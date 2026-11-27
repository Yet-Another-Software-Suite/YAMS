---
description: >-
  Step the simulated physics every millisecond from a Notifier, so a closed loop
  running on the motor controller sees a fresh measurement on every run.
icon: microscope
---

# How do I run a high fidelity simulation?

A TalonFX, TalonFXS or SPARK can run its closed loop on the motor controller, once every millisecond (1 kHz). By default YAMS steps the simulated physics once every 20 ms. Between two steps the simulated motor controller runs its loop 20 times against a measurement that has not changed, the way a real controller would if its encoder only updated every 20 ms.

For most robots that is close enough. It stops being close enough with stiff gains: a loop that holds steady on the robot can oscillate in simulation, and gains tuned in simulation come out softer than the robot can take.

A high fidelity simulation steps the physics every 1 ms instead, from a `Notifier`, so every run of the motor controller's loop sees where the mechanism actually is.

{% hint style="info" %}
This only changes simulation. Nothing on this page has any effect on the real robot.
{% endhint %}

## What changes

|                                                                                                                                         | Default                                             | High fidelity                                      |
| --------------------------------------------------------------------------------------------------------------------------------------- | --------------------------------------------------- | -------------------------------------------------- |
| Physics step ([`withSimulationPeriod`](../reference/editor/config/simulation.md#withsimulationperiod))                                  | 20 ms                                               | 1 ms                                               |
| What calls `simIterate()`                                                                                                               | `simulationPeriodic()`, every 20 ms                 | A `Notifier`, every 1 ms                           |
| Loop on the motor controller                                                                                                            | Runs every 1 ms, gets a new measurement every 20 ms | Runs every 1 ms, gets a new measurement every 1 ms |
| Loop on the SystemCore ([`withClosedLoopControlPeriod`](../reference/editor/config/closed-loop-control.md#withclosedloopcontrolperiod)) | 20 ms                                               | Unchanged                                          |
| `updateTelemetry()`                                                                                                                     | `periodic()`                                        | Unchanged                                          |

## Which motor controllers it helps

[Where the loop runs](../reference/editor/config/closed-loop-control.md#where-the-loop-runs) decides how much you gain.

* **TalonFX and TalonFXS.** PID and motion profiles run on the motor controller. Phoenix simulates the motor controller's own loop, and YAMS gives it the rotor position and velocity on every `simIterate()`. YAMS also raises the update rate of the motor's status signals to match the simulation period, in simulation only, so the values you read keep up with the physics.
* **SPARK MAX and SPARK Flex.** PID and trapezoidal profiles run on the motor controller. YAMS already runs the SPARK's simulated loop once per millisecond inside every step, but the mechanism's velocity is frozen for the whole step. At 1 ms every run of the loop gets a fresh one.
* **Exponential profiles or** [**LQR**](../understanding/lqr-controllers.md) **on any motor controller, velocity trapezoidal profiles and other loops a Thrifty Nova can't run, and every loop on an A301.** The loop runs on the SystemCore, every closed loop control period. Those loops gain nothing from a faster simulation period apart from smoother physics.

## Set it up

{% stepper %}
{% step %}
#### Set the simulation period to 1 ms

Add `withSimulationPeriod` to the `SmartMotorControllerConfig` of every motor the `Notifier` will step. For a swerve drive that is the drive motor and the azimuth motor of every module.

{% tabs %}
{% tab title="Java" %}
```java
import static org.wpilib.units.Units.Milliseconds;

.withSimulationPeriod(Milliseconds.of(1))
```
{% endtab %}

{% tab title="C++" %}
```cpp
.WithSimulationPeriod(1_ms)
```
{% endtab %}
{% endtabs %}

The simulated supply current filter and the shared [battery simulation](../understanding/battery-simulation.md) follow the simulation period on their own, so there is nothing else to set for them.

{% hint style="warning" %}
**The simulation period must match how often `simIterate()` is called.** Each call moves the simulated mechanism forward by exactly one simulation period, however much time has really passed. With a 20 ms period called every 1 ms, the mechanism moves 20 times faster than real time. With a 1 ms period called every 20 ms, it moves 20 times slower.
{% endhint %}
{% endstep %}

{% step %}
#### Step the simulation from a `Notifier`

Create a `Notifier` that calls `simIterate()` every millisecond, and only start it in simulation. Then take `simIterate()` out of `simulationPeriodic()`. If both call it, the mechanism is stepped twice and moves faster than real time.

Keep `updateTelemetry()` where it is. Publishing telemetry every millisecond only costs time.

{% tabs %}
{% tab title="Java, Commands v2" %}
{% code title="subsystems/ArmSubsystem.java" %}
```java
import static org.wpilib.units.Units.Milliseconds;

import org.wpilib.framework.RobotBase;
import org.wpilib.system.Notifier;

public class ArmSubsystem extends SubsystemBase {
  // ...

  // Steps the arm's physics simulation every millisecond, in simulation only.
  private final Notifier simNotifier = new Notifier(() -> arm.simIterate());

  public ArmSubsystem() {
    if (RobotBase.isSimulation()) {
      simNotifier.setName("Arm simulation");
      // Must match withSimulationPeriod on the arm's motor.
      simNotifier.startPeriodic(Milliseconds.of(1));
    }
  }

  @Override
  public void periodic() {
    // Publishes the arm's telemetry, and applies anything changed through live tuning.
    arm.updateTelemetry();
  }

  // No simulationPeriodic(): the Notifier calls arm.simIterate() instead.
}
```
{% endcode %}
{% endtab %}

{% tab title="Java, Commands v3" %}
In Commands v3 the simulation already runs in a `Notifier` that `Robot.simulationInit()` starts for each mechanism (see [Step 4: Simulate it](../tutorials/arm/simulate.md)). Keep that pattern and only change the period: start the arm's `Notifier` every 1 ms instead of every robot loop.

{% code title="Robot.java (fragment)" %}
```java
import static org.wpilib.units.Units.Milliseconds;

import org.wpilib.units.measure.Time;

@Override
public void simulationInit() {
  // Must match withSimulationPeriod on the arm's motor.
  startSimulation("Arm", arm::simulationPeriodic, Milliseconds.of(1));
}

// Keep the tutorial's startSimulation(name, simulationPeriodic), which runs every robot loop,
// for mechanisms left on the default 20 ms period, and add this overload beside it.

/** Runs a mechanism's simulation in its own Notifier, every period. */
private void startSimulation(String name, Runnable simulationPeriodic, Time period) {
  Notifier notifier = new Notifier(simulationPeriodic);
  notifier.setName(name + " Simulation");
  notifier.startPeriodic(period);
  simulationNotifiers.add(notifier);
}
```
{% endcode %}

`arm` is the `ArmMechanism` field in the tutorial's `Robot`, and `ArmMechanism.simulationPeriodic()` stays as it is, calling `arm.simIterate()`. Mechanisms left on the default 20 ms period keep calling the two-argument `startSimulation(...)`.
{% endtab %}

{% tab title="C++" %}
{% code title="include/subsystems/ArmSubsystem.hpp" %}
```cpp
#include <wpi/system/Notifier.hpp>

class ArmSubsystem : public wpi::cmd::SubsystemBase {
  // ...
 private:
  // Steps the arm's physics simulation every millisecond, in simulation only.
  // Declare it after m_arm, so it is stopped and destroyed before the arm is.
  wpi::Notifier m_simNotifier{[this] { m_arm->SimIterate(); }};
};
```
{% endcode %}

{% code title="cpp/subsystems/ArmSubsystem.cpp" %}
```cpp
#include <wpi/framework/RobotBase.hpp>

ArmSubsystem::ArmSubsystem() {
  // ...
  if (wpi::RobotBase::IsSimulation()) {
    m_simNotifier.SetName("Arm simulation");
    // Must match WithSimulationPeriod on the arm's motor.
    m_simNotifier.StartPeriodic(1_ms);
  }
}

void ArmSubsystem::Periodic() {
  // Publishes the arm's telemetry, and applies anything changed through live tuning.
  m_arm->UpdateTelemetry();
}

// No SimulationPeriodic(): the Notifier calls m_arm->SimIterate() instead.
```
{% endcode %}
{% endtab %}
{% endtabs %}

The same pattern works for every mechanism class, and for a `SmartMotorController` used without one: call its `simIterate()` from the `Notifier`. A mechanism class also updates its Mechanism2d drawing on every step, and an `Arm` or `Pivot` enforces its simulated hard limits there too, so those keep up with the physics. For a swerve drive, call the drive's `simIterate()` from the `Notifier`.
{% endstep %}

{% step %}
#### Check your simulation gains

Gains that were tuned against 20 ms physics may now behave differently, usually better. If you set [`withSimClosedLoopController`](../reference/editor/config/simulation.md#withsimclosedloopcontroller-and-withsimfeedforward) softer than the real gains to stop oscillation in simulation, try the real gains again first.

Nothing else needs to change:

* **Leave `withClosedLoopControlPeriod` alone.** It sets how often a loop on the SystemCore runs, not the physics. An LQR controller must keep the default 20 ms.
* **Leave the real-robot config alone.** `withSimulationPeriod` is ignored on the robot.
{% endstep %}
{% endstepper %}

## Things to know

* **The `Notifier` runs on its own thread**, at the same time as your periodic methods and commands. YAMS runs its own SystemCore closed loops the same way. Call `simIterate()` for a mechanism from one place only.
* **It costs CPU.** Every mechanism now runs its physics 1000 times a second. If your computer cannot keep up, the `Notifier` runs late, and since each call still adds exactly 1 ms, the simulation runs slower than real time. With many mechanisms, one `Notifier` that calls every `simIterate()` is lighter than one per mechanism. A period of 2 ms or 5 ms is a good compromise: it is still 10 or 4 times finer than the default.
* **Keep unit tests on the default period.** Stepping every millisecond makes tests much slower, especially on CI.

## Check it

Start the simulation and hold the arm at a setpoint. With stiff gains that oscillated before, the arm should now settle. The mechanism should also move at real speed: a move that takes 1 second on the robot should take about 1 second in simulation. If it is 20 times too fast or too slow, the simulation period and the `Notifier` period do not match.

## Read more

* Make the battery run down in simulation too: [How do I enable battery discharge simulation?](how-do-i-enable-battery-discharge-simulation.md)

## Code Reference

* [`SmartMotorControllerConfig`](https://javadocs.yassrobotics.com/yams/yams/core/motorcontrollers/SmartMotorControllerConfig.html)
