---
description: Run the elevator in simulation and drive it with a controller.
icon: flask
---

# Step 4: Simulate it

The code from the first three steps runs on your computer exactly as it will on the robot. YAMS simulates the motor, the gearing, the carriage and gravity, so the closed loop, the limits and the commands all behave the way they will on the real elevator.

{% stepper %}
{% step %}
### Start the simulation

{% tabs %}
{% tab title="Java, Commands v2" %}
Open the command palette with <kbd>Ctrl+Shift+P</kbd> and run **WPILib: Simulate Robot Code**. If you are asked which extensions to run, tick **Sim GUI**.
{% endtab %}

{% tab title="Java, Commands v3" %}
Open the command palette with <kbd>Ctrl+Shift+P</kbd> and run **WPILib: Simulate Robot Code**. If you are asked which extensions to run, tick **Sim GUI**.

The buttons are bound in the `ExampleTeleop` op mode, so that is the op mode to run when you enable.

If the simulated mechanism never moves, check that `Robot.java` has a `simulationInit()` that calls `startSimulation(...)` for the mechanism.
{% endtab %}

{% tab title="C++" %}
Open the command palette with <kbd>Ctrl+Shift+P</kbd> and run **WPILib: Simulate Robot Code**. If you are asked which extensions to run, tick **Sim GUI**.

The first C++ build compiles the whole project, so it takes a few minutes. Later builds are much faster.
{% endtab %}
{% endtabs %}

<figure><img src="../../.gitbook/assets/image (16).png" alt="The Sim GUI, the window that opens with the simulation, with the Robot State, System Joysticks, Joysticks and NetworkTables panels"><figcaption><p>The Sim GUI.</p></figcaption></figure>
{% endstep %}

{% step %}
### Drive it

1. Connect a controller to your computer, then drag it from **System Joysticks** to **Joystick\[0]**. Commands v3: drag it to **Joystick\[1]** instead, because the Command v3 template's controller is on port 1 (`DriverConstants.DRIVER_CONTROLLER_PORT`).
2. Open the simulated mechanism with **NetworkTables -> Mechanisms -> Elevator -> mechanism**. It is named after the `withTelemetry("Elevator", ...)` on the `ElevatorConfig`. It shows the carriage as a bar that grows upward as the carriage rises, with short marks beside it for the soft limits and the hard stops.

<figure><img src="../../.gitbook/assets/Robot Simulation 8_30_2025 7_06_08 PM.png" alt="The Sim GUI with the elevator&#x27;s Mechanism2d window open: a vertical bar for the carriage height, next to marks for the limits"><figcaption></figcaption></figure>

3. Resize **Elevator/mechanism** to your liking.
4. Commands v3: pick the `ExampleTeleop` op mode first. Then press **Teleoperated** in **Robot State**. The controller now drives the simulated elevator just as it would the real one.

No controller? The Sim GUI can use the keyboard as a joystick: drag **Keyboard 0** to **Joystick\[0]** (**Joystick\[1]** for Commands v3).

<figure><img src="../../.gitbook/assets/Robot Simulation 2025-08-30 18-37-59.gif" alt="The simulated elevator in its Mechanism2d window, the carriage bar rising and falling between heights as buttons are pressed"><figcaption></figcaption></figure>
{% endstep %}
{% endstepper %}

## What you should see

* Before you enable, the carriage is drawn at 0 m, its starting position, at the bottom.
* When you enable, the default command holds it at 0 m, so it stays at the bottom.
* Holding **A** raises it toward 0.5 m, and holding **B** toward 1 m. Let go, and it goes back down to 0 m. It moves no faster than 0.5 m/s, the limit from the motion profile.
* Holding **X** drives it up, and **Y** drives it down, until the soft limits stop it at 0 m and 2 m.
* It settles below each setpoint, roughly 15 cm with the placeholder gains. That is gravity: `kG` is 0, so only `kP` holds the carriage up, and `kP` only pushes once the carriage has dropped below the setpoint. The next step fixes it.

## If it did not work

* **The elevator is not in the NetworkTables menu.** Look under **Mechanisms**, not **SmartDashboard**.
* **The elevator does not move.** Check the robot is enabled in **Teleoperated** and the controller is on **Joystick\[0]** (**Joystick\[1]** for Commands v3). Commands v3: check you picked `ExampleTeleop`, and that `Robot.simulationInit()` starts the elevator's simulation with `startSimulation(...)`.
* **The code stops with an error when the simulation starts.** Read the first error. YAMS names the setting that is missing, such as soft limits in meters set before `withDrumRadius`.

More problems and fixes: [Troubleshooting](../troubleshooting.md).
