---
icon: microchip
---

# TunerX in simulation

Phoenix Tuner X can connect to your simulation and show every simulated TalonFX and TalonFXS signal (position, velocity, closed loop error, applied voltage, current draw) as it would on real hardware. You can check gain behavior, confirm soft and hard limits trigger, and plot signals without a robot.

{% hint style="info" %}
This works because YAMS feeds its physics simulation results back to the simulated Talons through Phoenix's `getSimState()` API. It is specific to **CTRE TalonFX and TalonFXS** motors. REV SPARKs and the Thrifty Nova use WPILib's simulation, and are visible through AdvantageScope or Elastic over NetworkTables instead.
{% endhint %}

{% hint style="warning" %}
Changes you make in Tuner X are not YAMS [live tuning](./), and are not saved to your YAMS config. YAMS applies its config to the Talon each time the code starts, which replaces the settings YAMS manages. Use Tuner X to watch the signals, and change gains, profiles and limits with live tuning, then copy them into your code.
{% endhint %}

{% stepper %}
{% step %}
### Run the simulation

Run your robot code in simulation, for example with **WPILib: Simulate Robot Code** in VS Code, or `./gradlew simulateJava` in a terminal.

**Check it:** the simulation GUI opens and the robot code prints no errors.
{% endstep %}

{% step %}
### Connect Tuner X to localhost

Open Phoenix Tuner X, then open the flyout menu in the upper left. Click the arrow next to the connection field and choose the **localhost** preset, which CTRE describes as the setting for simulation. See [Connecting Tuner X](https://v6.docs.ctr-electronics.com/en/stable/docs/tuner/connecting.html) in the Phoenix 6 docs.

**Check it:** the connection field says localhost, and Tuner X shows it is connected.
{% endstep %}

{% step %}
### Find your devices

Open the **Devices** page. Every simulated TalonFX and TalonFXS shows up with the CAN ID from your code, all on one CAN bus.

**Check it:** the list has one entry for each TalonFX or TalonFXS in your code. If it is empty, check the simulation is still running and the connection field says localhost.
{% endstep %}

{% step %}
### Plot the signals

Select a motor and plot its signals. Enable the robot and move the mechanism.

**Check it:** the position, velocity, applied voltage and current change as the simulated mechanism moves.
{% endstep %}
{% endstepper %}

{% hint style="warning" %}
In simulation every CAN device appears on the same CAN bus, so two simulated devices with the same CAN ID on different buses clash. Give them unique IDs.
{% endhint %}
