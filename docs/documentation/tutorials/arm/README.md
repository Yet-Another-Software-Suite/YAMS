---
description: Learn how to program an Arm.
icon: hand-fist
---

# Arm

## Intro

At the end of this tutorial you will have an `Arm` that works in both real life and simulation with the same code!

{% columns %}
{% column %}
#### Simulation

<figure><img src="../../.gitbook/assets/arm.gif" alt=""><figcaption></figcaption></figure>
{% endcolumn %}

{% column %}
#### Real Life

{% embed url="https://www.youtube.com/shorts/sKhi0ieo6WE" %}
{% endcolumn %}
{% endcolumns %}

## Details

This `Arm` uses the following hardware and controls:

* A `SparkMax` driving one NEO controls the `Arm`
* A `12:1` gearbox on the `Arm`
* Holding the bottom face button (`A` on an Xbox controller) takes the `Arm` to -5 degrees
* Holding the right face button (`B` on an Xbox controller) takes the `Arm` to 15 degrees
* Holding the left face button (`X` on an Xbox controller) drives the `Arm` up
* Holding the top face button (`Y` on an Xbox controller) drives the `Arm` down
* With no button held, the `Arm` goes to 0 degrees

## The steps

<figure><img src="../../.gitbook/assets/tutorial-arm-steps.png" alt="The four steps of the arm tutorial, each building on the one before. Step 1, the motor controller: a SparkMax and a SmartMotorControllerConfig. Step 2, the arm: an ArmConfig and the SmartMotorController. Step 3, commands and buttons: run and set commands bound to a controller. Step 4, simulation: the same code run on your computer."><figcaption><p>Four steps, one page each.</p></figcaption></figure>

Every step has a tab for **Java with Commands v2**, **Java with Commands v3** and **C++**.

| Step                                                | What you do                                                                                   |
| --------------------------------------------------- | --------------------------------------------------------------------------------------------- |
| [Before you start](../project-setup/)               | Create a WPILib project and install YAMS and REVLib. Skip this if you already have a project. |
| [Step 1: The motor controller](motor-controller.md) | Configure the `SparkMax` and wrap it in a `SmartMotorController`.                             |
| [Step 2: The arm](mechanism.md)                     | Describe the arm with an `ArmConfig` and create the `Arm`.                                    |
| [Step 3: Commands and buttons](commands.md)         | Turn the `Arm` into commands and bind them to controller buttons.                             |
| [Step 4: Simulate it](simulate.md)                  | Run the code on your computer and drive the arm with a controller.                            |
| [Step 5: Tune it](tune.md)                          | Tune the gains while the simulation runs, and copy them into your code.                       |
| [Step 6: Run it on the robot](robot.md)             | Check the arm safely on the robot, then tune it there.                                        |

## More than one motor

Two motors on one shaft or gearbox? Add the second as a follower with `withFollowers`. Two motors that aren't on the same shaft or gearbox, such as one on each side of the mechanism? Make the second a loosely coupled follower with `withLooselyCoupledFollowers`: the mechanism still works as this tutorial shows. [Followers](../../reference/editor/config/followers.md) explains both.

## Complete Example

The following files show a complete, working Arm from the YAMS `simple_robot-2027` Commands v2 example. It uses a `TalonFX` with a `CANcoder` instead of the `SparkMax` in this tutorial; the YAMS code is otherwise the same.

{% @github-files/github-code-block url="https://github.com/Yet-Another-Software-Suite/YAMS/blob/master/examples/commands2/simple_robot-2027/src/main/java/first/robot/subsystems/ArmSubsystem.java" %}

{% @github-files/github-code-block url="https://github.com/Yet-Another-Software-Suite/YAMS/blob/master/examples/commands2/simple_robot-2027/src/main/java/first/robot/RobotContainer.java" %}

The same robot in the other two flavors:

* **Java, Commands v3:** [`mechanisms/ArmMechanism.java`](https://github.com/Yet-Another-Software-Suite/YAMS/blob/master/examples/commands3/simple_robot-2027/src/main/java/first/robot/mechanisms/ArmMechanism.java) in `examples/commands3/simple_robot-2027`
* **C++:** [`ArmSubsystem.h`](https://github.com/Yet-Another-Software-Suite/YAMS/blob/master/examples/cpptest/src/main/include/subsystems/ArmSubsystem.h) and [`ArmSubsystem.cpp`](https://github.com/Yet-Another-Software-Suite/YAMS/blob/master/examples/cpptest/src/main/cpp/subsystems/ArmSubsystem.cpp) in `examples/cpptest`
