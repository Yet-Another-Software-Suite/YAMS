---
description: Learn how to program a shooter flywheel.
icon: fan
---

# Shooter/FlyWheels

{% hint style="info" %}
Two motors on one shaft or gearbox? Add the second as a follower with `withFollowers`. Two motors that aren't on the same shaft or gearbox, such as one on each side of the mechanism? Make the second a loosely coupled follower with `withLooselyCoupledFollowers`: the mechanism still works as this tutorial shows. [Followers](../../reference/editor/config/followers.md) explains both.
{% endhint %}

## Intro

At the end of this tutorial you will have a `Shooter` that works in both real life and simulation with the same code!

<figure><img src="../../.gitbook/assets/Robot Simulation 2025-09-09 09-07-39.gif" alt="The simulated shooter in its Mechanism2d window: a line that spins with the flywheel while a button is held"><figcaption></figcaption></figure>

## Details

This `Shooter` uses the following hardware and controls:

* A `SparkMax` driving one NEO controls the `Shooter`
* A `1.5:1` gearbox on the `Shooter`, so its top speed is about 5676 RPM (a NEO's free speed) / 1.5 = 3784 RPM
* Two 4 inch flywheel wheels on one shaft, 1.5 lb in all
* Holding the bottom face button (`A` on an Xbox controller) spins the `Shooter` at 2000 RPM
* Holding the right face button (`B` on an Xbox controller) spins the `Shooter` at 3000 RPM
* Holding the left face button (`X` on an Xbox controller) drives the `Shooter` forward
* Holding the top face button (`Y` on an Xbox controller) drives the `Shooter` backward
* With no button held, the `Shooter` coasts

## The steps

<figure><img src="../../.gitbook/assets/tutorial-shooter-steps.png" alt="The four steps of the shooter tutorial, each building on the one before. Step 1, the motor controller: a SparkMax and a SmartMotorControllerConfig. Step 2, the flywheel: a FlyWheelConfig and the SmartMotorController. Step 3, commands and buttons: run and set commands bound to a controller. Step 4, simulation: the same code run on your computer."><figcaption><p>The first four steps, one page each. Then you tune it and run it on the robot.</p></figcaption></figure>

Every step has a tab for **Java with Commands v2**, **Java with Commands v3** and **C++**.

| Step                                                | What you do                                                                                   |
| --------------------------------------------------- | --------------------------------------------------------------------------------------------- |
| [Before you start](../project-setup/)               | Create a WPILib project and install YAMS and REVLib. Skip this if you already have a project. |
| [Step 1: The motor controller](motor-controller.md) | Configure the `SparkMax` and wrap it in a `SmartMotorController`.                             |
| [Step 2: The flywheel](mechanism.md)                | Describe the flywheel with a `FlyWheelConfig` and create the `FlyWheel`.                      |
| [Step 3: Commands and buttons](commands.md)         | Turn the `FlyWheel` into commands and bind them to controller buttons.                        |
| [Step 4: Simulate it](simulate.md)                  | Run the code on your computer and drive the flywheel with a controller.                       |
| [Step 5: Tune it](tune.md)                          | Find `kV` and `kP` while the simulation runs, and copy them into your code.                   |
| [Step 6: Run it on the robot](../arm/robot.md)      | The arm's robot step, with the flywheel differences listed at the end of step 5.              |

## Complete Example

The following files show a complete, working Shooter from the YAMS `simple_robot-2027` Commands v2 example. It is not the same hardware as this tutorial: it drives two Kraken X60s through `TalonFX` motor controllers instead of a NEO through a `SparkMax`, with different gearing and gains. The YAMS parts work the same way.

{% @github-files/github-code-block url="https://github.com/Yet-Another-Software-Suite/YAMS/blob/master/examples/commands2/simple_robot-2027/src/main/java/first/robot/subsystems/ShooterSubsystem.java" %}

{% @github-files/github-code-block url="https://github.com/Yet-Another-Software-Suite/YAMS/blob/master/examples/commands2/simple_robot-2027/src/main/java/first/robot/RobotContainer.java" %}

The same robot in the other two flavors:

* **Java, Commands v3:** [`mechanisms/ShooterMechanism.java`](https://github.com/Yet-Another-Software-Suite/YAMS/blob/master/examples/commands3/simple_robot-2027/src/main/java/first/robot/mechanisms/ShooterMechanism.java) in `examples/commands3/simple_robot-2027`
* **C++:** [`ShooterSubsystem.h`](https://github.com/Yet-Another-Software-Suite/YAMS/blob/master/examples/cpptest/src/main/include/subsystems/ShooterSubsystem.h) and [`ShooterSubsystem.cpp`](https://github.com/Yet-Another-Software-Suite/YAMS/blob/master/examples/cpptest/src/main/cpp/subsystems/ShooterSubsystem.cpp) in `examples/cpptest`
