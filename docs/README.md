---
description: >-
  Yet Another Mechanism System: the mechanism library built from years of FRC
  experience. Describe your mechanism once, and get the commands, simulation,
  telemetry, and live tuning.
layout:
  width: wide
  title:
    visible: false
  description:
    visible: false
  tableOfContents:
    visible: false
  outline:
    visible: false
  pagination:
    visible: false
  metadata:
    visible: true
  tags:
    visible: true
  actions:
    visible: true
  anchors:
    visible: true
---

# Yet Another Mechanism System

<h2 align="center"><mark style="color:green;">Yet Another Mechanism System</mark></h2>

<p align="center">Describe your mechanism once. YAMS gives you the commands, simulation, telemetry, and live tuning that top FRC teams spend seasons building, all in one open-source library.</p>

<table data-card-size="large" data-view="cards"><thead><tr><th></th><th></th><th></th><th data-hidden data-card-target data-type="content-ref"></th><th data-hidden data-card-cover data-type="image">Cover image</th></tr></thead><tbody><tr><td><i class="fa-wand-magic-sparkles">:wand-magic-sparkles:</i></td><td><strong>Generate a mechanism</strong></td><td>Want to skip straight to working code? Describe your mechanism in a form, and YAMG, our mechanism generator, writes the YAMS subsystem for you, ready to drop into your robot. It handles elevators, arms, pivots, turrets, and flywheels, with support for absolute encoders and homing switches.</td><td><a href="https://yamg.yassrobotics.com">https://yamg.yassrobotics.com</a></td><td><a href=".gitbook/assets/yamg.png">yamg.png</a></td></tr><tr><td><i class="fa-robot">:robot:</i></td><td><strong>Examples</strong></td><td>Download complete robot projects built with YAMS from the latest release, for Commands v2 and v3: BroncBotz, Everybot, Kitbot, the REV and WCP 2026 robots, and more, each ready to simulate.</td><td><a href="https://github.com/Yet-Another-Software-Suite/YAMS/releases/latest">https://github.com/Yet-Another-Software-Suite/YAMS/releases/latest</a></td><td><a href=".gitbook/assets/examples.png">examples.png</a></td></tr></tbody></table>

<table data-card-size="large" data-view="cards"><thead><tr><th></th><th></th><th></th><th data-hidden data-card-target data-type="content-ref"></th></tr></thead><tbody><tr><td><i class="fa-book">:book:</i></td><td><strong>Documentation</strong></td><td>Step-by-step tutorials for arms, elevators, flywheels, and swerve, from simulation and  live tuning, to running on the robot.</td><td><a href="https://app.gitbook.com/o/MwECAyhaWCMK5V9K79gd/s/ZM0CFmYiQzcrY4zDcTtZ/">Documentation</a></td></tr><tr><td><i class="fa-code">:code:</i></td><td><strong>API reference</strong></td><td>Every class and every config option, what it does, and how to use it.</td><td><a href="https://app.gitbook.com/o/MwECAyhaWCMK5V9K79gd/s/ezOwaXLQ3h1N7tr3zYnj/">API Reference</a></td></tr></tbody></table>

{% columns %}
{% column %}
### Getting started

1. **Create a project and install YAMS.** It is one vendordep in a standard WPILib 2027 project.
2. **Describe your mechanism.** Configure the motor controller, then the mechanism, inside a subsystem, as in the code here. [How YAMS drives a motor](https://yams.yassrobotics.com/documentation/tutorials/how-yams-drives-a-motor) explains how those pieces fit together, the [Arm tutorial](https://yams.yassrobotics.com/documentation/tutorials/arm) walks you through them, or [YAMG](https://yamg.yassrobotics.com) writes them for you from a form.
3. **Run it in simulation.** Every mechanism simulates itself. Drive it, tune it, and graph it in AdvantageScope before the robot is built.
4. **Run it on the robot.** Set the CAN IDs, check the motor direction with the robot disabled, and rest the mechanism at its starting position. Then enable it and tune it on the real mechanism, as in [Run it on the robot](https://yams.yassrobotics.com/documentation/tutorials/arm/robot). If it misbehaves, [Troubleshooting](https://yams.yassrobotics.com/documentation/tutorials/troubleshooting) covers the common failures.

<a href="https://yams.yassrobotics.com/documentation/tutorials/project-setup" class="button primary" data-icon="rocket-launch">Get started</a> <a href="https://yamg.yassrobotics.com" class="button secondary" data-icon="wand-magic-sparkles">Open YAMG</a> <a href="https://github.com/Yet-Another-Software-Suite/YAMS/releases/latest" class="button secondary" data-icon="download">Download examples</a>
{% endcolumn %}

{% column %}
{% code title="An arm, from motor to command (Java, Commands v2)" %}
```java
// This goes inside your arm subsystem's constructor: `this` is the subsystem.

// 1. Configure the motor controller: control, gains, gearing and limits.
SmartMotorControllerConfig motorConfig = new SmartMotorControllerConfig(this)
    .withControlMode(ControlMode.CLOSED_LOOP)
    .withClosedLoopController(4, 0, 0) // Example gain, tune it for your arm.
    .withTrapezoidalProfile(DegreesPerSecond.of(180), DegreesPerSecondPerSecond.of(90))
    .withFeedforward(new ArmFeedforward(0, 0, 0, 0)) // Placeholder gains, until you tune them.
    .withGearing(new MechanismGearing(GearBox.fromReductionStages(3, 4)))
    .withSoftLimits(Degrees.of(-30), Degrees.of(100))
    .withStatorCurrentLimit(Amps.of(40))
    .withZeroPower(MotorMode.BRAKE)
    .withStartingPosition(Degrees.of(0))
    .withTelemetry("ArmMotor", TelemetryVerbosity.HIGH);

// 2. Create the motor controller.
TalonFX armMotor = new TalonFX(1, new CANBus(CANPort.CAN_S0));
SmartMotorController motor = new TalonFXWrapper(armMotor, DCMotor.getKrakenX60(1), motorConfig);

// 3. Configure the arm.
ArmConfig armConfig = new ArmConfig()
    .withLength(Meters.of(0.135))
    .withHardLimits(Degrees.of(-100), Degrees.of(200))
    .withTelemetry("Arm", TelemetryVerbosity.HIGH);

// 4. Create the arm, and use its commands.
Arm arm = new Arm(armConfig, motor);
Command raise = arm.setAngle(Degrees.of(90));
```
{% endcode %}

For the whole subsystem with its imports, see [the complete arm file](https://yams.yassrobotics.com/documentation/tutorials/arm/mechanism#the-complete-file) in the Arm tutorial.
{% endcolumn %}
{% endcolumns %}

{% columns %}
{% column %}
### Built from years of FRC experience

The best FRC teams have spent years building their own mechanism code: motion profiles that plan for what a motor can really do, simulations that include the battery drain under load, gains they can tune while the robot runs, absolute encoders that know where a turret is the moment the robot turns on. That code usually stays on one team's robot, while every other team tries to reinvent the robotic wheel every year.&#x20;

YAMS brings those ideas together in one open-source library. Now any team can use these features from its very first mechanism. Built by Yet Another Software Suite, the team behind YAGSL, whose main developers have been doing FRC for over 10 years.&#x20;

<a href="https://app.gitbook.com/s/ZM0CFmYiQzcrY4zDcTtZ/why-yams" class="button primary" data-icon="rocket">Why YAMS?</a> <a href="https://github.com/Yet-Another-Software-Suite/YAMS" class="button secondary" data-icon="github">Source on GitHub</a>
{% endcolumn %}

{% column %}
<figure><img src=".gitbook/assets/battery-sim-mechanisms.png" alt="How much current an arm, an elevator, a flywheel, a turret and a swerve drive draw over time, and the battery sag when their peaks line up"><figcaption><p>YAMS simulates the drain across every mechanism at once, so you catch brownouts in sim instead of on the field. </p></figcaption></figure>
{% endcolumn %}
{% endcolumns %}

{% columns %}
{% column %}
<figure><img src=".gitbook/assets/exponential-profile-compare.png" alt="A trapezoidal and an exponential profile for the same move: the exponential one plans for the motor having less torque as it speeds up"><figcaption><p>Exponential profiles plan for a motor losing torque as it speeds up.</p></figcaption></figure>
{% endcolumn %}

{% column %}
### Everything a mechanism needs, built in

* **Simulation** of every mechanism with no extra code, including a [battery simulation](https://yams.yassrobotics.com/documentation/how-to-simulation-and-logging/how-do-i-enable-battery-discharge-simulation) of the whole robot.
* [**Live tuning**](https://yams.yassrobotics.com/documentation/how-to-tuning/live-tuning) of the gains, the profile, and the limits from a dashboard, applied while the robot runs.
* [**Telemetry**](https://yams.yassrobotics.com/documentation/understanding/telemetry) of the motor and the mechanism, at the verbosity you choose.
* **Control** beyond PID: trapezoidal and [exponential profiles](https://yams.yassrobotics.com/documentation/how-to-motor-controllers-and-sensors/how-do-i-use-exponential-profiles), and [LQR](https://yams.yassrobotics.com/documentation/details/editor/lqr-controllers).
* [**Absolute encoders**](https://yams.yassrobotics.com/documentation/how-to-mechanisms/how-do-i-use-absolute-encoders-on-my-arm) that know where the mechanism is from power up, plus [EasyCRT](https://yams.yassrobotics.com/documentation/details/easycrt) to find a turret's angle from two of them.
* **Sensors** that simulate themselves: wrap a limit switch, a beam break, or any other [sensor](https://yams.yassrobotics.com/documentation/details/sensors), and the simulation triggers it just like the real one, such as a [homing switch](https://yams.yassrobotics.com/documentation/how-to-mechanisms/how-do-i-home-a-mechanism-with-limit-switches) that presses when the mechanism reaches its hard stop.
* **Every common mechanism**: arms, elevators, pivots, turrets, flywheels, double-jointed arms, differential mechanisms, and swerve drives.
* **Your hardware**: TalonFX, TalonFXS, SPARK MAX, SPARK Flex, and Thrifty Nova are all supported in Java and C++, using WPILib Commands v2 or v3.
{% endcolumn %}
{% endcolumns %}

<h2 align="center"><mark style="color:$success;">Join a community of over 500 teams</mark></h2>

<p align="center">Stuck on a config, found a bug, or just wanna talk about robots? Here's where to find us. </p>

{% columns %}
{% column %}
### YASS Discord&#x20;

The Discord is the fastest way to reach us! Ask questions, report bugs, or just say hi with over 1,000 FRC enthusiasts.&#x20;

<a href="https://discord.gg/yass" class="button primary" data-icon="discord">Join Discord</a>
{% endcolumn %}

{% column %}
### Instagram

Quick updates, announcements,  Team YASS shoutouts, and Rivet trivia.



<a href="https://instagram.com/yassrobotics" class="button primary" data-icon="instagram">Follow on Instagram</a>
{% endcolumn %}

{% column %}
### GitHub

File issues, open pull requests, or browse the source for any of our projects.&#x20;



<a href="https://github.com/Yet-Another-Software-Suite/YAMS/issues" class="button primary" data-icon="github">Report an issue</a> <a href="https://github.com/Yet-Another-Software-Suite/YAMS/pulls" class="button secondary" data-icon="code-pull-request">Pull requests</a>
{% endcolumn %}
{% endcolumns %}
