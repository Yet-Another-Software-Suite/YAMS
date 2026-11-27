---
description: Make commands from the Arm and bind them to controller buttons.
icon: gamepad
---

# Step 3: Commands and buttons

The `Arm` makes commands for you. Your class hands them out, and the buttons schedule them.

{% hint style="success" %}
**In a hurry?** Jump to [the complete file](commands.md#the-complete-file), copy it, and change the values called out under **For your robot**. The numbered parts explain each piece in a sentence or two. You do not need to understand all of it to get moving: the pages under **Understanding** go deeper when you want them.
{% endhint %}

{% stepper %}
{% step %}
### Make the commands

The `Arm` makes the commands. Your class only hands them out, so that the rest of the robot code never touches the motor directly.

| What it does                                                  | Java, Commands v2             | Java, Commands v3             | C++                              |
| ------------------------------------------------------------- | ----------------------------- | ----------------------------- | -------------------------------- |
| Go to an angle and hold it. It never ends on its own.         | `arm.run(angle)`              | `arm.setAngle(angle)`         | `m_arm->Run(angle)`              |
| Go to an angle, and end when the arm is within the tolerance. | `arm.runTo(angle, tolerance)` | `arm.runTo(angle, tolerance)` | `m_arm->RunTo(angle, tolerance)` |
| Drive at a duty cycle from -1 to 1, with the closed loop off. | `arm.set(dutycycle)`          | `arm.set(dutycycle)`          | `m_arm->Set(dutycycle)`          |

{% tabs %}
{% tab title="Java, Commands v2" %}
{% code title="subsystems/ArmSubsystem.java" %}
```java
/** Go to an angle and hold it. This command does not end on its own. */
public Command run(Angle angle) {
  return arm.run(angle);
}

/** Go to an angle, and end once the arm is within the tolerance of it. */
public Command runTo(Angle angle, Angle tolerance) {
  return arm.runTo(angle, tolerance);
}

/** Drive the arm with a duty cycle from -1 to 1. The closed loop is not used. */
public Command set(double dutycycle) {
  return arm.set(dutycycle);
}
```
{% endcode %}
{% endtab %}

{% tab title="Java, Commands v3" %}
{% code title="mechanisms/ArmMechanism.java" %}
```java
/** Go to an angle and hold it. This command does not end on its own. */
public Command setAngle(Angle angle) {
  return arm.setAngle(angle);
}

/** Go to an angle, and end once the arm is within the tolerance of it. */
public Command moveTo(Angle angle, Angle tolerance) {
  return arm.runTo(angle, tolerance);
}

/** Drive the arm with a duty cycle from -1 to 1. The closed loop is not used. */
public Command set(double dutycycle) {
  return arm.set(dutycycle);
}
```
{% endcode %}
{% endtab %}

{% tab title="C++" %}
{% code title="include/subsystems/ArmSubsystem.hpp" %}
```cpp
/** Go to an angle and hold it. This command does not end on its own. */
wpi::cmd::CommandPtr Run(wpi::units::degree_t angle);

/** Go to an angle, and end once the arm is within the tolerance of it. */
wpi::cmd::CommandPtr RunTo(wpi::units::degree_t angle, wpi::units::degree_t tolerance);

/** Drive the arm with a duty cycle from -1 to 1. The closed loop is not used. */
wpi::cmd::CommandPtr Set(double dutycycle);
```
{% endcode %}

{% code title="cpp/subsystems/ArmSubsystem.cpp" %}
```cpp
wpi::cmd::CommandPtr ArmSubsystem::Run(wpi::units::degree_t angle) {
  return m_arm->Run(angle);
}

wpi::cmd::CommandPtr ArmSubsystem::RunTo(wpi::units::degree_t angle, wpi::units::degree_t tolerance) {
  return m_arm->RunTo(angle, tolerance);
}

wpi::cmd::CommandPtr ArmSubsystem::Set(double dutycycle) {
  return m_arm->Set(dutycycle);
}
```
{% endcode %}
{% endtab %}
{% endtabs %}

**For your robot:** Rename these to suit your robot, or add more. Each one is a single line.

Want the details? [When mechanism commands end](../../understanding/run-vs-runto.md)
{% endstep %}

{% step %}
### Set the default command

The **default command** runs whenever no other command is using the arm. Here it holds the arm at 0 degrees. In Commands v3 the default command is given the lowest priority, so that any other command for the mechanism can interrupt it.

{% tabs %}
{% tab title="Java, Commands v2" %}
{% code title="RobotContainer.java" %}
```java
import first.robot.subsystems.ArmSubsystem;

private final ArmSubsystem arm = new ArmSubsystem();

// When no other command is using the arm, hold it at 0 degrees.
arm.setDefaultCommand(arm.run(Degrees.of(0)));
```
{% endcode %}
{% endtab %}

{% tab title="Java, Commands v3" %}
{% code title="mechanisms/ArmMechanism.java" %}
```java
/** Hold an angle at the lowest priority, so that any other arm command can interrupt it. */
public Command hold(Angle angle) {
  return run(coroutine -> coroutine.await(arm.setAngle(angle)))
      .withPriority(Command.LOWEST_PRIORITY)
      .named("Arm Hold");
}
```
{% endcode %}

{% code title="Robot.java" %}
```java
public class Robot extends OpModeRobot {
  // ...

  public Robot() {
    // When no other command is using the arm, hold it at 0 degrees.
    arm.setDefaultCommand(arm.hold(Degrees.of(0)));
  }

  // ...
}
```
{% endcode %}
{% endtab %}

{% tab title="C++" %}
{% code title="include/RobotContainer.hpp" %}
```cpp
#include "subsystems/ArmSubsystem.hpp"

ArmSubsystem arm;
```
{% endcode %}

{% code title="cpp/RobotContainer.cpp" %}
```cpp
// When no other command is using the arm, hold it at 0 degrees.
arm.SetDefaultCommand(arm.Run(0_deg));
```
{% endcode %}
{% endtab %}
{% endtabs %}

**For your robot:** Choose what your mechanism should do when nobody is asking anything of it.
{% endstep %}

{% step %}
### Bind the buttons

`whileTrue` runs a command for as long as its button is held, and cancels it on release. The default command then takes back over.

<figure><img src="../../.gitbook/assets/tutorial-arm-buttons.gif" alt="Animation of the four face buttons of a controller, the arm, and the command that is running. With no button held the default command runs: hold the arm at 0 degrees. While A is held its command runs: hold the arm at minus 5 degrees. While B is held: hold the arm at 15 degrees. While X is held the arm is driven at a 30 percent duty cycle with the closed loop off. Each time a button is released the default command takes back over."><figcaption><p>A held button owns the arm. Let go, and the default command brings it back to 0°.</p></figcaption></figure>

{% tabs %}
{% tab title="Java, Commands v2" %}
{% code title="RobotContainer.java" %}
```java
// Hold an angle while the bottom (A) or right (B) face button is held.
driverController.faceDown().whileTrue(arm.run(Degrees.of(-5)));
driverController.faceRight().whileTrue(arm.run(Degrees.of(15)));
// Drive the arm up or down while the left (X) or top (Y) face button is held.
driverController.faceLeft().whileTrue(arm.set(0.3));
driverController.faceUp().whileTrue(arm.set(-0.3));
```
{% endcode %}
{% endtab %}

{% tab title="Java, Commands v3" %}
{% code title="ExampleTeleop.java" %}
```java
// Hold an angle while the bottom (A) or right (B) face button is held.
robot.exampleController.faceDown().whileTrue(robot.arm.setAngle(Degrees.of(-5)));
robot.exampleController.faceRight().whileTrue(robot.arm.setAngle(Degrees.of(15)));
// Drive the arm up or down while the left (X) or top (Y) face button is held.
robot.exampleController.faceLeft().whileTrue(robot.arm.set(0.3));
robot.exampleController.faceUp().whileTrue(robot.arm.set(-0.3));
```
{% endcode %}
{% endtab %}

{% tab title="C++" %}
{% code title="cpp/RobotContainer.cpp" %}
```cpp
// Hold an angle while the bottom (A) or right (B) face button is held.
driverController.FaceDown().WhileTrue(arm.Run(-5_deg));
driverController.FaceRight().WhileTrue(arm.Run(15_deg));
// Drive the arm up or down while the left (X) or top (Y) face button is held.
driverController.FaceLeft().WhileTrue(arm.Set(0.3));
driverController.FaceUp().WhileTrue(arm.Set(-0.3));
```
{% endcode %}
{% endtab %}
{% endtabs %}

**For your robot:** Pick your own buttons and setpoints. Use `onTrue` instead if one press should start a command that keeps running after the button is released.
{% endstep %}
{% endstepper %}

## The complete file

{% tabs %}
{% tab title="Java, Commands v2" %}
The highlighted lines are new in this step.

Add the commands to `ArmSubsystem.java`, then create the subsystem and bind the buttons in `RobotContainer.java`. The template bound the right face button to its example command; that binding is replaced.

<pre class="language-java" data-title="subsystems/ArmSubsystem.java" data-line-numbers><code class="lang-java">package first.robot.subsystems;

import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.DegreesPerSecond;
import static org.wpilib.units.Units.DegreesPerSecondPerSecond;
import static org.wpilib.units.Units.Feet;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
<strong>import org.wpilib.command2.Command;
</strong>import org.wpilib.command2.SubsystemBase;
import org.wpilib.hardware.bus.CANPort;
import org.wpilib.math.controller.ArmFeedforward;
import org.wpilib.math.system.DCMotor;
<strong>import org.wpilib.units.measure.Angle;
</strong>import yams.commands2.config.SmartMotorControllerConfig;
import yams.commands2.mechanisms.Arm;
import yams.core.gearing.GearBox;
import yams.core.gearing.MechanismGearing;
import yams.core.mechanisms.config.ArmConfig;
import yams.core.motorcontrollers.SmartMotorController;
import yams.core.motorcontrollers.enums.ControlMode;
import yams.core.motorcontrollers.enums.MotorMode;
import yams.core.motorcontrollers.local.SparkWrapper;
import yams.core.telemetry.enums.TelemetryVerbosity;

public class ArmSubsystem extends SubsystemBase {

  // What the motor controller should do: its control mode, gains, gearing and limits.
  private final SmartMotorControllerConfig smcConfig = new SmartMotorControllerConfig(this)
      .withControlMode(ControlMode.CLOSED_LOOP)
      // Feedback constants (PID), on the real robot and in simulation.
      .withClosedLoopController(50, 0, 0)
      .withSimClosedLoopController(50, 0, 0)
      // The fastest the closed loop is allowed to move and accelerate the arm.
      .withTrapezoidalProfile(DegreesPerSecond.of(90), DegreesPerSecondPerSecond.of(45))
      // Feedforward constants, on the real robot and in simulation.
      .withFeedforward(new ArmFeedforward(0, 0, 0))
      .withSimFeedforward(new ArmFeedforward(0, 0, 0))
      // Gearing from the motor rotor to the arm: a 3:1 stage and a 4:1 stage, 12:1 in total.
      .withGearing(new MechanismGearing(GearBox.fromReductionStages(3, 4)))
      .withMotorInverted(false)
      .withZeroPower(MotorMode.BRAKE)
      .withStatorCurrentLimit(Amps.of(40))
      // Where the arm is when the robot turns on.
      .withStartingPosition(Degrees.of(-5))
      // The closed loop will not take the arm past these angles.
      .withSoftLimits(Degrees.of(-20), Degrees.of(30))
      // Telemetry name and how much to publish.
      .withTelemetry("ArmMotor", TelemetryVerbosity.HIGH);

  // The vendor motor controller object: a SPARK MAX with CAN ID 4.
  private final SparkMax spark = new SparkMax(CANPort.CAN_S0, 4, MotorType.kBrushless);

  // The SmartMotorController applies the config to the SPARK MAX, which drives one NEO.
  private final SmartMotorController motor = new SparkWrapper(spark, DCMotor.getNEO(1), smcConfig);

  private final ArmConfig armConfig = new ArmConfig()
      // The hard stops, used by the simulation.
      .withHardLimits(Degrees.of(-30), Degrees.of(40))
      // The length of the arm, used by the simulation.
      .withLength(Feet.of(3))
      // Telemetry name and how much to publish.
      .withTelemetry("Arm", TelemetryVerbosity.HIGH);

  // The arm mechanism.
  private final Arm arm = new Arm(armConfig, motor);

<strong>  /** Go to an angle and hold it. This command does not end on its own. */
</strong><strong>  public Command run(Angle angle) {
</strong><strong>    return arm.run(angle);
</strong><strong>  }
</strong>
<strong>  /** Go to an angle, and end once the arm is within the tolerance of it. */
</strong><strong>  public Command runTo(Angle angle, Angle tolerance) {
</strong><strong>    return arm.runTo(angle, tolerance);
</strong><strong>  }
</strong>
<strong>  /** Drive the arm with a duty cycle from -1 to 1. The closed loop is not used. */
</strong><strong>  public Command set(double dutycycle) {
</strong><strong>    return arm.set(dutycycle);
</strong><strong>  }
</strong>
  @Override
  public void periodic() {
    // Publishes the arm's telemetry, and applies anything changed through live tuning.
    arm.updateTelemetry();
  }

  @Override
  public void simulationPeriodic() {
    // Runs the arm's physics simulation.
    arm.simIterate();
  }
}
</code></pre>

<pre class="language-java" data-title="RobotContainer.java" data-line-numbers><code class="lang-java">package first.robot;

<strong>import static org.wpilib.units.Units.Degrees;
</strong>
import org.wpilib.command2.Command;
import org.wpilib.command2.button.CommandGamepad;
import org.wpilib.command2.button.Trigger;
import first.robot.Constants.OperatorConstants;
import first.robot.commands.Autos;
import first.robot.commands.ExampleCommand;
<strong>import first.robot.subsystems.ArmSubsystem;
</strong>import first.robot.subsystems.ExampleSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem exampleSubsystem = new ExampleSubsystem();
<strong>  private final ArmSubsystem arm = new ArmSubsystem();
</strong>
  private final CommandGamepad driverController =
      new CommandGamepad(OperatorConstants.DRIVER_CONTROLLER_PORT);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();

<strong>    // When no other command is using the arm, hold it at 0 degrees.
</strong><strong>    arm.setDefaultCommand(arm.run(Degrees.of(0)));
</strong>  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link org.wpilib.command2.button.CommandGenericHID}'s
   * subclasses for {@link CommandGamepad Gamepad} gamepads or {@link
   * org.wpilib.command2.button.CommandJoystick Flight joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    new Trigger(exampleSubsystem::exampleCondition).onTrue(new ExampleCommand(exampleSubsystem));

<strong>    // Hold an angle while the bottom (A) or right (B) face button is held.
</strong><strong>    driverController.faceDown().whileTrue(arm.run(Degrees.of(-5)));
</strong><strong>    driverController.faceRight().whileTrue(arm.run(Degrees.of(15)));
</strong><strong>    // Drive the arm up or down while the left (X) or top (Y) face button is held.
</strong><strong>    driverController.faceLeft().whileTrue(arm.set(0.3));
</strong><strong>    driverController.faceUp().whileTrue(arm.set(-0.3));
</strong>  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.exampleAuto(exampleSubsystem);
  }
}
</code></pre>
{% endtab %}

{% tab title="Java, Commands v3" %}
The highlighted lines are new in this step.

Add the commands to `ArmMechanism.java`, set the default command in `Robot.java`, and bind the buttons in `ExampleTeleop.java`.

A `Mechanism` already has a `run(...)` method that takes a coroutine, so the commands here have their own names. The default command is wrapped with `Command.LOWEST_PRIORITY`, so that any other command for the arm can interrupt it.

<pre class="language-java" data-title="mechanisms/ArmMechanism.java" data-line-numbers><code class="lang-java">package first.robot.mechanisms;

import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.DegreesPerSecond;
import static org.wpilib.units.Units.DegreesPerSecondPerSecond;
import static org.wpilib.units.Units.Feet;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
<strong>import org.wpilib.command3.Command;
</strong>import org.wpilib.command3.Mechanism;
import org.wpilib.hardware.bus.CANPort;
import org.wpilib.math.controller.ArmFeedforward;
import org.wpilib.math.system.DCMotor;
<strong>import org.wpilib.units.measure.Angle;
</strong>import yams.commands3.config.SmartMotorControllerConfig;
import yams.commands3.mechanisms.Arm;
import yams.core.gearing.GearBox;
import yams.core.gearing.MechanismGearing;
import yams.core.mechanisms.config.ArmConfig;
import yams.core.motorcontrollers.SmartMotorController;
import yams.core.motorcontrollers.enums.ControlMode;
import yams.core.motorcontrollers.enums.MotorMode;
import yams.core.motorcontrollers.local.SparkWrapper;
import yams.core.telemetry.enums.TelemetryVerbosity;

public class ArmMechanism implements Mechanism {

  // What the motor controller should do: its control mode, gains, gearing and limits.
  private final SmartMotorControllerConfig smcConfig = new SmartMotorControllerConfig(this)
      .withControlMode(ControlMode.CLOSED_LOOP)
      // Feedback constants (PID), on the real robot and in simulation.
      .withClosedLoopController(50, 0, 0)
      .withSimClosedLoopController(50, 0, 0)
      // The fastest the closed loop is allowed to move and accelerate the arm.
      .withTrapezoidalProfile(DegreesPerSecond.of(90), DegreesPerSecondPerSecond.of(45))
      // Feedforward constants, on the real robot and in simulation.
      .withFeedforward(new ArmFeedforward(0, 0, 0))
      .withSimFeedforward(new ArmFeedforward(0, 0, 0))
      // Gearing from the motor rotor to the arm: a 3:1 stage and a 4:1 stage, 12:1 in total.
      .withGearing(new MechanismGearing(GearBox.fromReductionStages(3, 4)))
      .withMotorInverted(false)
      .withZeroPower(MotorMode.BRAKE)
      .withStatorCurrentLimit(Amps.of(40))
      // Where the arm is when the robot turns on.
      .withStartingPosition(Degrees.of(-5))
      // The closed loop will not take the arm past these angles.
      .withSoftLimits(Degrees.of(-20), Degrees.of(30))
      // Telemetry name and how much to publish.
      .withTelemetry("ArmMotor", TelemetryVerbosity.HIGH);

  // The vendor motor controller object: a SPARK MAX with CAN ID 4.
  private final SparkMax spark = new SparkMax(CANPort.CAN_S0, 4, MotorType.kBrushless);

  // The SmartMotorController applies the config to the SPARK MAX, which drives one NEO.
  private final SmartMotorController motor = new SparkWrapper(spark, DCMotor.getNEO(1), smcConfig);

  private final ArmConfig armConfig = new ArmConfig()
      // The hard stops, used by the simulation.
      .withHardLimits(Degrees.of(-30), Degrees.of(40))
      // The length of the arm, used by the simulation.
      .withLength(Feet.of(3))
      // Telemetry name and how much to publish.
      .withTelemetry("Arm", TelemetryVerbosity.HIGH);

  // The arm mechanism.
  private final Arm arm = new Arm(armConfig, motor);

<strong>  /** Go to an angle and hold it. This command does not end on its own. */
</strong><strong>  public Command setAngle(Angle angle) {
</strong><strong>    return arm.setAngle(angle);
</strong><strong>  }
</strong>
<strong>  /** Go to an angle, and end once the arm is within the tolerance of it. */
</strong><strong>  public Command moveTo(Angle angle, Angle tolerance) {
</strong><strong>    return arm.runTo(angle, tolerance);
</strong><strong>  }
</strong>
<strong>  /** Drive the arm with a duty cycle from -1 to 1. The closed loop is not used. */
</strong><strong>  public Command set(double dutycycle) {
</strong><strong>    return arm.set(dutycycle);
</strong><strong>  }
</strong>
<strong>  /** Hold an angle at the lowest priority, so that any other arm command can interrupt it. */
</strong><strong>  public Command hold(Angle angle) {
</strong><strong>    return run(coroutine -> coroutine.await(arm.setAngle(angle)))
</strong><strong>        .withPriority(Command.LOWEST_PRIORITY)
</strong><strong>        .named("Arm Hold");
</strong><strong>  }
</strong>
  /** Call this from Robot.robotPeriodic(), before the scheduler runs. */
  public void periodic() {
    // Publishes the arm's telemetry, and applies anything changed through live tuning.
    arm.updateTelemetry();
  }

  /** Robot.simulationInit() runs this in its own Notifier, every robot loop. */
  public void simulationPeriodic() {
    // Runs the arm's physics simulation.
    arm.simIterate();
  }
}
</code></pre>

<pre class="language-java" data-title="Robot.java" data-line-numbers><code class="lang-java">package first.robot;

<strong>import static org.wpilib.units.Units.Degrees;
</strong>
import java.util.ArrayList;
import java.util.List;
import org.wpilib.command3.Scheduler;
import org.wpilib.command3.button.CommandGamepad;
import org.wpilib.framework.OpModeRobot;
import org.wpilib.system.Notifier;
import org.wpilib.telemetry.Telemetry;
import first.robot.constants.DriverConstants;
import first.robot.mechanisms.ArmMechanism;
import first.robot.mechanisms.ExampleMechanism;

public class Robot extends OpModeRobot {
  final ExampleMechanism exampleMechanism = new ExampleMechanism();
  final ArmMechanism arm = new ArmMechanism();
  final CommandGamepad exampleController =
      new CommandGamepad(DriverConstants.DRIVER_CONTROLLER_PORT);

  // The scheduler that runs every command.
  private final Scheduler scheduler = Scheduler.getDefault();
  // Each mechanism's simulation Notifier, kept so that it is not garbage collected.
  private final List&#x3C;Notifier> simulationNotifiers = new ArrayList&#x3C;>();

<strong>  public Robot() {
</strong><strong>    // When no other command is using the arm, hold it at 0 degrees.
</strong><strong>    arm.setDefaultCommand(arm.hold(Degrees.of(0)));
</strong><strong>  }
</strong>
  @Override
  public void robotPeriodic() {
    // Runs every loop, on the robot and in simulation.
    // Update the mechanism before the scheduler runs its commands.
    arm.periodic();
    scheduler.run();
    // Publish the scheduler, so that a dashboard shows which commands are running.
    Telemetry.log("Scheduler", scheduler, Scheduler.proto);
  }

  @Override
  public void simulationInit() {
    // Runs once, when the simulation starts.
    // Run each mechanism's simulation in its own Notifier, every robot loop.
    startSimulation("Arm", arm::simulationPeriodic);
  }

  /** Runs a mechanism's simulation in its own Notifier, every robot loop period. */
  private void startSimulation(String name, Runnable simulationPeriodic) {
    Notifier notifier = new Notifier(simulationPeriodic);
    notifier.setName(name + " Simulation");
    notifier.startPeriodic(getPeriod());
    simulationNotifiers.add(notifier);
  }
}
</code></pre>

<pre class="language-java" data-title="ExampleTeleop.java" data-line-numbers><code class="lang-java">package first.robot;

<strong>import static org.wpilib.units.Units.Degrees;
</strong>
import org.wpilib.opmode.OpMode;
import org.wpilib.opmode.Teleop;

@Teleop
public class ExampleTeleop implements OpMode {
  public ExampleTeleop(Robot robot) {
    robot.exampleController.rightStick().whileTrue(robot.exampleMechanism.exampleCommand());

<strong>    // Hold an angle while the bottom (A) or right (B) face button is held.
</strong><strong>    robot.exampleController.faceDown().whileTrue(robot.arm.setAngle(Degrees.of(-5)));
</strong><strong>    robot.exampleController.faceRight().whileTrue(robot.arm.setAngle(Degrees.of(15)));
</strong><strong>    // Drive the arm up or down while the left (X) or top (Y) face button is held.
</strong><strong>    robot.exampleController.faceLeft().whileTrue(robot.arm.set(0.3));
</strong><strong>    robot.exampleController.faceUp().whileTrue(robot.arm.set(-0.3));
</strong>  }
}
</code></pre>
{% endtab %}

{% tab title="C++" %}
The highlighted lines are new in this step.

Declare the commands in `ArmSubsystem.hpp`, define them in `ArmSubsystem.cpp`, then create the subsystem and bind the buttons in `RobotContainer`. The template bound the right face button to its example command; that binding is replaced.

<pre class="language-cpp" data-title="include/subsystems/ArmSubsystem.hpp" data-line-numbers><code class="lang-cpp">#pragma once

#include &#x3C;optional>

#include &#x3C;rev/SparkMax.h>
<strong>#include &#x3C;wpi/commands2/CommandPtr.hpp>
</strong>#include &#x3C;wpi/commands2/SubsystemBase.hpp>
#include &#x3C;wpi/hardware/bus/CANPort.hpp>
<strong>#include &#x3C;wpi/units/angle.hpp>
</strong>
#include "yams/mechanisms/config/ArmConfig.hpp"
#include "yams/mechanisms/positional/Arm.hpp"
#include "yams/motorcontrollers/SmartMotorControllerConfig.hpp"
#include "yams/motorcontrollers/local/SparkWrapper.hpp"

class ArmSubsystem : public wpi::cmd::SubsystemBase {
 public:
  ArmSubsystem();

<strong>  /** Go to an angle and hold it. This command does not end on its own. */
</strong><strong>  wpi::cmd::CommandPtr Run(wpi::units::degree_t angle);
</strong>
<strong>  /** Go to an angle, and end once the arm is within the tolerance of it. */
</strong><strong>  wpi::cmd::CommandPtr RunTo(wpi::units::degree_t angle, wpi::units::degree_t tolerance);
</strong>
<strong>  /** Drive the arm with a duty cycle from -1 to 1. The closed loop is not used. */
</strong><strong>  wpi::cmd::CommandPtr Set(double dutycycle);
</strong>
  void Periodic() override;
  void SimulationPeriodic() override;

 private:
  // The vendor motor controller object: a SPARK MAX with CAN ID 4.
  rev::spark::SparkMax m_spark{wpi::CANPort::CAN_S0, 4, rev::spark::SparkMax::MotorType::kBrushless};

  // What the motor controller should do. It is filled in by the constructor.
  yams::motorcontrollers::SmartMotorControllerConfig m_smcConfig;

  // The SmartMotorController, created in the constructor once the config is ready.
  std::optional&#x3C;yams::motorcontrollers::local::SparkWrapper> m_motor;

  // The arm mechanism and its config, also set up in the constructor.
  yams::mechanisms::config::ArmConfig m_armConfig;
  std::optional&#x3C;yams::mechanisms::positional::Arm> m_arm;
};
</code></pre>

<pre class="language-cpp" data-title="cpp/subsystems/ArmSubsystem.cpp" data-line-numbers><code class="lang-cpp">#include "subsystems/ArmSubsystem.hpp"

#include &#x3C;wpi/math/controller/ArmFeedforward.hpp>
#include &#x3C;wpi/math/system/DCMotor.hpp>
#include &#x3C;wpi/units/angle.hpp>
#include &#x3C;wpi/units/angular_acceleration.hpp>
#include &#x3C;wpi/units/angular_velocity.hpp>
#include &#x3C;wpi/units/current.hpp>
#include &#x3C;wpi/units/length.hpp>
#include &#x3C;wpi/units/voltage.hpp>

#include "yams/gearing/GearBox.hpp"
#include "yams/gearing/MechanismGearing.hpp"

using namespace yams::gearing;
using Cfg = yams::motorcontrollers::SmartMotorControllerConfig;

ArmSubsystem::ArmSubsystem() {
  // What the motor controller should do: its control mode, gains, gearing and limits.
  m_smcConfig
      .WithSubsystem(this)
      .WithClosedLoopMode()
      // Feedback constants (PID), on the real robot and in simulation.
      .WithFeedback(50, 0, 0)
      .WithSimClosedLoopController(50, 0, 0)
      // The fastest the closed loop is allowed to move and accelerate the arm.
      .WithTrapezoidProfile(90_deg_per_s, 45_deg_per_s_sq)
      // Feedforward constants: kS, kG and kV.
      .WithFeedforward(wpi::math::ArmFeedforward{
          0_V, 0_V, wpi::units::unit_t&#x3C;wpi::math::ArmFeedforward::kv_unit>{0}})
      // Gearing from the motor rotor to the arm: a 3:1 stage and a 4:1 stage, 12:1 in total.
      .WithMotorGearing(MechanismGearing{GearBox::FromReductionStages({3.0, 4.0})})
      .WithMotorInverted(false)
      .WithZeroPower(Cfg::MotorMode::BRAKE)
      .WithStatorCurrentLimit(40_A)
      // Where the arm is when the robot turns on.
      .WithStartingPosition(-5_deg)
      // The closed loop will not take the arm past these angles.
      .WithMechanismLimits(-20_deg, 30_deg)
      // Telemetry name and how much to publish.
      .WithTelemetry("ArmMotor", Cfg::TelemetryVerbosity::HIGH);

  // The SmartMotorController applies the config to the SPARK MAX, which drives one NEO.
  m_motor.emplace(&#x26;m_spark, wpi::math::DCMotor::NEO(1), &#x26;m_smcConfig);

  m_armConfig
      // The hard stops, used by the simulation.
      .WithMinAngle(-30_deg)
      .WithMaxAngle(40_deg)
      // The length of the arm, used by the simulation.
      .WithArmLength(3_ft)
      // Telemetry name.
      .WithTelemetryName("Arm");

  // The arm mechanism.
  m_arm.emplace(&#x26;m_armConfig, &#x26;m_motor.value());
}

<strong>wpi::cmd::CommandPtr ArmSubsystem::Run(wpi::units::degree_t angle) {
</strong><strong>  return m_arm->Run(angle);
</strong><strong>}
</strong>
<strong>wpi::cmd::CommandPtr ArmSubsystem::RunTo(wpi::units::degree_t angle, wpi::units::degree_t tolerance) {
</strong><strong>  return m_arm->RunTo(angle, tolerance);
</strong><strong>}
</strong>
<strong>wpi::cmd::CommandPtr ArmSubsystem::Set(double dutycycle) {
</strong><strong>  return m_arm->Set(dutycycle);
</strong><strong>}
</strong>
void ArmSubsystem::Periodic() {
  // Publishes the arm's telemetry, and applies anything changed through live tuning.
  m_arm->UpdateTelemetry();
}

void ArmSubsystem::SimulationPeriodic() {
  // Runs the arm's physics simulation.
  m_arm->SimIterate();
}
</code></pre>

<pre class="language-cpp" data-title="include/RobotContainer.hpp" data-line-numbers><code class="lang-cpp">#pragma once

#include "Constants.hpp"
<strong>#include "subsystems/ArmSubsystem.hpp"
</strong>#include "subsystems/ExampleSubsystem.hpp"
#include "wpi/commands2/CommandPtr.hpp"
#include "wpi/commands2/button/CommandGamepad.hpp"

/**
 * This class is where the bulk of the robot should be declared.  Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls).  Instead, the structure of the robot (including subsystems,
 * commands, and trigger mappings) should be declared here.
 */
class RobotContainer {
 public:
  RobotContainer();

  wpi::cmd::CommandPtr GetAutonomousCommand();

 private:
  // Replace with CommandPS4Controller or CommandJoystick if needed
  wpi::cmd::CommandGamepad driverController{
      OperatorConstants::DRIVER_CONTROLLER_PORT};

  // The robot's subsystems are defined here...
  ExampleSubsystem subsystem;
<strong>  ArmSubsystem arm;
</strong>
  void ConfigureBindings();
};
</code></pre>

<pre class="language-cpp" data-title="cpp/RobotContainer.cpp" data-line-numbers><code class="lang-cpp">#include "RobotContainer.hpp"

#include "commands/Autos.hpp"
#include "commands/ExampleCommand.hpp"
#include "wpi/commands2/button/Trigger.hpp"
<strong>#include "wpi/units/angle.hpp"
</strong>
RobotContainer::RobotContainer() {
  // Initialize all of your commands and subsystems here

<strong>  // When no other command is using the arm, hold it at 0 degrees.
</strong><strong>  arm.SetDefaultCommand(arm.Run(0_deg));
</strong>
  // Configure the button bindings
  ConfigureBindings();
}

void RobotContainer::ConfigureBindings() {
  // Configure your trigger bindings here

  // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
  wpi::cmd::Trigger([this] {
    return subsystem.ExampleCondition();
  }).OnTrue(ExampleCommand(&#x26;subsystem).ToPtr());

<strong>  // Hold an angle while the bottom (A) or right (B) face button is held.
</strong><strong>  driverController.FaceDown().WhileTrue(arm.Run(-5_deg));
</strong><strong>  driverController.FaceRight().WhileTrue(arm.Run(15_deg));
</strong><strong>  // Drive the arm up or down while the left (X) or top (Y) face button is held.
</strong><strong>  driverController.FaceLeft().WhileTrue(arm.Set(0.3));
</strong><strong>  driverController.FaceUp().WhileTrue(arm.Set(-0.3));
</strong>}

wpi::cmd::CommandPtr RobotContainer::GetAutonomousCommand() {
  // An example command will be run in autonomous
  return autos::ExampleAuto(&#x26;subsystem);
}
</code></pre>
{% endtab %}
{% endtabs %}

{% hint style="info" %}
These commands are helpers that start and stop the closed loop for you. If you would rather set setpoints from your own command or state machine, see [How do I write my own mechanism command?](../../how-to-mechanisms/how-do-i-write-my-own-mechanism-command.md).
{% endhint %}

## run or runTo?

<figure><img src="../../.gitbook/assets/howto-arm-run-vs-runto.gif" alt="Animation of the same arm moving to 45 degrees with two commands, with a default command of run to 0 degrees. With run the command keeps running after the arm arrives, and the arm stays at 45 degrees. With runTo and a tolerance of 3 degrees the command ends once the arm has stayed inside the tolerance for 0.1 seconds, then the default command takes over and moves the arm back to 0 degrees."><figcaption><p><code>run()</code> keeps running until something interrupts it. <code>runTo()</code> ends at the setpoint, and the default command takes over.</p></figcaption></figure>

The buttons here use the command that never ends, because a button that is held should keep holding the arm. Use the one that ends when you need to know the arm has arrived, such as in an autonomous sequence. [When mechanism commands end](../../understanding/run-vs-runto.md) covers the difference, and [How do I set an arm's target angle?](../../how-to-mechanisms/how-to-set-an-arms-target-angle.md) has more examples.

## Check it

Build the code. If it builds, every button is bound. Next you run it and press them.
