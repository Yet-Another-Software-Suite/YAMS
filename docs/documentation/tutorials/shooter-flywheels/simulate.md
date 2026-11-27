---
description: Run the flywheel in simulation and drive it with a controller.
icon: flask
---

# Step 4: Simulate it

The code from the first three steps runs on your computer exactly as it will on the robot. YAMS simulates the motor, the gearing and the weight of the wheel, so the closed loop and the commands behave the way they will on the real flywheel.

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
2. Open the simulated mechanism with **NetworkTables -> Mechanisms -> Shooter -> mechanism**. It is named after the `withTelemetry("Shooter", ...)` on the `FlyWheelConfig`. It shows the wheel as a line from its center that turns as the wheel turns.

<figure><img src="../../.gitbook/assets/Robot Simulation 9_9_2025 9_12_17 AM.png" alt="The Sim GUI with the shooter&#x27;s Mechanism2d window open: a single line from the center of the window, which turns with the flywheel"><figcaption></figcaption></figure>

3. Resize **Shooter/mechanism** to your liking.
4. Commands v3: pick the `ExampleTeleop` op mode first. Then press **Teleoperated** in **Robot State**. The controller now drives the simulated flywheel just as it would the real one.

No controller? The Sim GUI can use the keyboard as a joystick: drag **Keyboard 0** to **Joystick\[0]** (**Joystick\[1]** for Commands v3).

<figure><img src="../../.gitbook/assets/Robot Simulation 2025-09-09 09-05-32.gif" alt="The simulated flywheel in its Mechanism2d window, the line spinning while a button is held and slowing to a stop after it is released"><figcaption></figcaption></figure>

#### Watch the speed, not the drawing

The drawing is only updated 50 times a second. At shooter speeds the line turns most of a rotation between updates, so it can look slow, frozen, or even turning backward. To see the real speed, graph it:

* In the Sim GUI, open **NetworkTables -> Mechanisms -> Shooter -> ShooterMotor -> mechanism** and find **velocity**. Or graph `NT:/Mechanisms/Shooter/ShooterMotor/mechanism/velocity` in [AdvantageScope](https://docs.advantagescope.org/).
* It is in **rotations per second**. Multiply by 60 for RPM: 3000 RPM reads as `50`.
{% endstep %}
{% endstepper %}

## What you should see

* Before you enable, and with no button held, the speed is 0.
* Holding **A** spins the wheel up to 2000 RPM (about `33` on the graph), and holding **B** to 3000 RPM (`50`). It gets there in well under a second and stays within a few tens of RPM.
* Holding **X** drives it forward, and **Y** backward, at 30% duty cycle, with the closed loop off.
* Let go of any button, and the default command sets a duty cycle of 0, so the wheel spins down.

## If it did not work

* **The shooter is not in the NetworkTables menu.** Look under **Mechanisms**, not **SmartDashboard**.
* **The wheel does not spin.** Check the robot is enabled in **Teleoperated** and the controller is on **Joystick\[0]** (**Joystick\[1]** for Commands v3). Commands v3: check you picked `ExampleTeleop`, and that `Robot.simulationInit()` starts the shooter's simulation with `startSimulation(...)`.
* **The wheel takes many seconds to reach speed.** The motor controller config is missing `withMomentOfInertia`, so the simulation uses a much heavier wheel. See step 1, part 3.
* **The speed settles well below the setpoint.** `kV` is too low, or 0. See step 1, part 2.

More problems and fixes: [Troubleshooting](../troubleshooting.md).
