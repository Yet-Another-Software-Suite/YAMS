# Overview

These guides are short API recipes: the classes and calls you need for one job, in Java and C++, with links to the full reference for each class.

{% hint style="info" %}
**New to YAMS?** Start with the [tutorials in the documentation](https://yams.yassrobotics.com/documentation/tutorials/project-setup). They build each mechanism one step at a time, explain the ideas (PID, feedforward, motion profiles, simulation), and show how to tune it and run it on the robot. Come back here once you know what you want to build and need the exact calls.
{% endhint %}

The guides assume you have:

* A WPILib 2027 project with the YAMS vendordep installed, as in [Create a project and install YAMS](https://yams.yassrobotics.com/documentation/tutorials/project-setup).
* Basic command-based programming: subsystems (Commands v2) or mechanisms (Commands v3), commands and triggers.

## The guides

* [Configuring a motor controller](configuring-a-motor.md): the `SmartMotorControllerConfig` and the wrapper for your motor controller. Every mechanism starts here.
* [Building an arm subsystem](building-an-arm.md), [an elevator](building-an-elevator.md) and [a flywheel](building-a-flywheel.md), in Java.
* [Building positional mechanisms (C++)](building-mechanisms-cpp.md) and [a flywheel (C++)](building-a-flywheel-cpp.md).
* [Setting up swerve drive](setting-up-swerve.md), and [in C++](setting-up-swerve-cpp.md).
* [Using YAMS sensors](using-yams-sensors.md): sensors that work the same in simulation and on the robot.
* [Simulation](simulation.md): what each mechanism needs to simulate itself, and how to run it.

## The gains in these guides

The PID and feedforward numbers in the code are placeholders that show where each value goes. They are not tuned for your mechanism. With them, expect the mechanism to move toward its setpoint slowly, overshoot, or sag under gravity. Tune them as in [Tune it](https://yams.yassrobotics.com/documentation/tutorials/arm/tune) in the arm tutorial.

## If something does not work

[Troubleshooting](https://yams.yassrobotics.com/documentation/tutorials/troubleshooting) covers the common failures: code that does not build, a mechanism that does not show up or does not move in simulation, and problems on the real robot.
