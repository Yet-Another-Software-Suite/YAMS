---
description: Learn how to program an Elevator.
icon: elevator
---

# Elevator

{% hint style="info" %}
Two motors on one shaft or gearbox? Add the second as a follower with `withFollowers`. Two motors that aren't on the same shaft or gearbox, such as one on each side of the mechanism? Make the second a loosely coupled follower with `withLooselyCoupledFollowers`: the mechanism still works as this tutorial shows. [Followers](../../reference/editor/config/followers.md) explains both.
{% endhint %}

## Intro

At the end of this tutorial you will have an `Elevator` that works in both real life and simulation with the same code!

{% columns %}
{% column %}
#### Simulation

<figure><img src="../../.gitbook/assets/Robot Simulation 2025-08-30 18-27-07.gif" alt="The simulated elevator in its Mechanism2d window: the carriage bar rises to a height and stops there"><figcaption></figcaption></figure>

<figure><img src="../../.gitbook/assets/Robot Simulation 2025-08-30 18-37-59.gif" alt="The simulated elevator moving between heights as controller buttons are pressed, staying between its limit marks"><figcaption></figcaption></figure>
{% endcolumn %}

{% column %}
#### Real Life

{% embed url="https://www.youtube.com/shorts/XKmsW8-85Dg" %}
{% endcolumn %}
{% endcolumns %}

## Details

This `Elevator` uses the following hardware and controls:

* A `SparkMax` driving one NEO controls the `Elevator`
* A `12:1` gearbox on the `Elevator`
* \#25 chain on a 22 tooth sprocket lifts the carriage
* Holding the bottom face button (`A` on an Xbox controller) takes the `Elevator` to 0.5 m
* Holding the right face button (`B` on an Xbox controller) takes the `Elevator` to 1 m
* Holding the left face button (`X` on an Xbox controller) drives the `Elevator` up
* Holding the top face button (`Y` on an Xbox controller) drives the `Elevator` down
* With no button held, the `Elevator` goes to 0 m

## The steps

<figure><img src="../../.gitbook/assets/tutorial-elevator-steps.png" alt="The four steps of the elevator tutorial, each building on the one before. Step 1, the motor controller: a SparkMax and a SmartMotorControllerConfig. Step 2, the elevator: an ElevatorConfig and the SmartMotorController. Step 3, commands and buttons: run and set commands bound to a controller. Step 4, simulation: the same code run on your computer."><figcaption><p>The first four steps, one page each. Then you tune it and run it on the robot.</p></figcaption></figure>

Every step has a tab for **Java with Commands v2**, **Java with Commands v3** and **C++**.

| Step                                                | What you do                                                                                   |
| --------------------------------------------------- | --------------------------------------------------------------------------------------------- |
| [Before you start](../project-setup/)               | Create a WPILib project and install YAMS and REVLib. Skip this if you already have a project. |
| [Step 1: The motor controller](motor-controller.md) | Configure the `SparkMax` and wrap it in a `SmartMotorController`.                             |
| [Step 2: The elevator](mechanism.md)                | Describe the elevator with an `ElevatorConfig` and create the `Elevator`.                     |
| [Step 3: Commands and buttons](commands.md)         | Turn the `Elevator` into commands and bind them to controller buttons.                        |
| [Step 4: Simulate it](simulate.md)                  | Run the code on your computer and drive the elevator with a controller.                       |
| [Step 5: Tune it](tune.md)                          | Find `kG`, `kP` and the profile while the simulation runs, and copy them into your code.      |
| [Step 6: Run it on the robot](../arm/robot.md)      | The arm's robot step, with the elevator differences listed at the end of step 5.              |

## Complete Example

The following files show a complete, working Elevator from the YAMS `simple_robot-2027` Commands v2 example. It uses an exponential profile instead of the trapezoidal profile in this tutorial.

{% @github-files/github-code-block url="https://github.com/Yet-Another-Software-Suite/YAMS/blob/master/examples/commands2/simple_robot-2027/src/main/java/first/robot/subsystems/ElevatorSubsystem.java" %}

{% @github-files/github-code-block url="https://github.com/Yet-Another-Software-Suite/YAMS/blob/master/examples/commands2/simple_robot-2027/src/main/java/first/robot/RobotContainer.java" %}

The same robot in the other two flavors:

* **Java, Commands v3:** [`mechanisms/ElevatorMechanism.java`](https://github.com/Yet-Another-Software-Suite/YAMS/blob/master/examples/commands3/simple_robot-2027/src/main/java/first/robot/mechanisms/ElevatorMechanism.java) in `examples/commands3/simple_robot-2027`
* **C++:** [`ElevatorSubsystem.h`](https://github.com/Yet-Another-Software-Suite/YAMS/blob/master/examples/cpptest/src/main/include/subsystems/ElevatorSubsystem.h) and [`ElevatorSubsystem.cpp`](https://github.com/Yet-Another-Software-Suite/YAMS/blob/master/examples/cpptest/src/main/cpp/subsystems/ElevatorSubsystem.cpp) in `examples/cpptest`
