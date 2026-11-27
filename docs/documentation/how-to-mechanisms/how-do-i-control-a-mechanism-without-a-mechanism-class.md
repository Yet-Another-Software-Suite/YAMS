---
description: >-
  Control each SmartMotorController yourself, such as when the motors of one
  mechanism need different setpoints.
icon: microchip
---

# How do I control a mechanism without a mechanism class?

**Goal:** drive two motors that belong to one mechanism, but need **different** setpoints, from one subsystem with your own commands.

The example is a shooter with a top wheel and a bottom wheel. Spinning the bottom wheel faster than the top one puts backspin on the game piece, so the two wheels need different speeds at the same time.

## Do you need this page?

YAMS mechanism classes (`Arm`, `Elevator`, `FlyWheel` and so on) drive one motor and its followers with one setpoint. Use them when you can:

* **Motors on one shaft or gearbox** that always do the same thing: make the others [followers](../reference/editor/config/followers.md#withfollowers).
* **Separate motors that must move together**, such as one on each side of an intake arm: make the others [loosely coupled followers](../reference/editor/config/followers.md#withlooselycoupledfollowers).
* **Two different mechanisms**, such as a shooter and a feeder: give each its own subsystem and mechanism class. One YAMS mechanism per subsystem.

<figure><img src="../.gitbook/assets/smc-followers.png" alt="Two panels. Tightly coupled, withFollowers: a leader and a follower are on one shaft or gearbox and the follower copies the leader&#x27;s output. The motors are mechanically locked, there is one closed loop and one config, the followers are the same vendor as the leader, and voltage and duty cycle reach every motor. Loosely coupled, withLooselyCoupledFollowers: two separate motors joined only by the mechanism get the same setpoint and each runs its own PID. They are not mechanically locked, each has its own closed loop and config, vendors can be mixed, and voltage and duty cycle are not forwarded, so the leader moves alone."><figcaption><p>Tightly coupled followers share a shaft. Loosely coupled followers are separate motors.</p></figcaption></figure>

If none of those fit, control each `SmartMotorController` yourself, as below. The [Double FlyWheel tutorial](../tutorials/double-flywheel/) builds this same shooter step by step, with C++ as well; this page is the short version. You still get YAMS's telemetry, live tuning, current limiting, closed loop control and simulation for each motor. Without a mechanism class, your subsystem does three of its jobs: call `setupTelemetry()` on each motor once, `updateTelemetry()` every loop, and `simIterate()` every loop in simulation. You do not get the mechanism's drawing in the dashboard, or its ready-made commands and triggers.

## Why not two `FlyWheel`s in one subsystem?

Every YAMS-generated command (such as `FlyWheel.run` or `Arm.setVoltage`) requires the subsystem you passed to `new SmartMotorControllerConfig(this)`. A subsystem runs one command at a time. So if `topWheel.run(...)` and `bottomWheel.run(...)` both require the same subsystem, scheduling the second **cancels the first**, and only one wheel spins.

The fix is one command that sets both motors, which is what this page writes.

## The subsystem

The code uses two SPARK MAXes (CAN IDs 4 and 5), each driving a NEO 1:1 to a 4 inch wheel. The gains are **placeholders**: with them the wheels reach roughly the right speed in simulation, but tune them for your robot with [Step 5: Tune it](../tutorials/arm/tune.md) (for a flywheel, find `kV` first, then `kP`).

{% tabs %}
{% tab title="Commands v2" %}
{% code title="subsystems/ShooterSubsystem.java" lineNumbers="true" %}
```java
package first.robot.subsystems;

import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.Inches;
import static org.wpilib.units.Units.Pounds;
import static org.wpilib.units.Units.RPM;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import org.wpilib.command2.Command;
import org.wpilib.command2.SubsystemBase;
import org.wpilib.command2.button.Trigger;
import org.wpilib.hardware.bus.CANPort;
import org.wpilib.math.controller.SimpleMotorFeedforward;
import org.wpilib.math.system.DCMotor;
import org.wpilib.units.measure.AngularVelocity;
import yams.commands2.config.SmartMotorControllerConfig;
import yams.core.gearing.MechanismGearing;
import yams.core.motorcontrollers.SmartMotorController;
import yams.core.motorcontrollers.enums.ControlMode;
import yams.core.motorcontrollers.enums.MotorMode;
import yams.core.motorcontrollers.local.SparkWrapper;
import yams.core.telemetry.enums.TelemetryVerbosity;

public class ShooterSubsystem extends SubsystemBase {

  private final SmartMotorController top = new SparkWrapper(
      new SparkMax(CANPort.CAN_S0, 4, MotorType.kBrushless), DCMotor.getNEO(1), wheelConfig("TopWheel"));

  private final SmartMotorController bottom = new SparkWrapper(
      new SparkMax(CANPort.CAN_S0, 5, MotorType.kBrushless), DCMotor.getNEO(1), wheelConfig("BottomWheel"));

  /** Both wheels are built the same way, so one method makes both configs. */
  private SmartMotorControllerConfig wheelConfig(String name) {
    return new SmartMotorControllerConfig(this)
        .withControlMode(ControlMode.CLOSED_LOOP)
        .withGearing(new MechanismGearing(1))
        // PLACEHOLDER gains. kV is volts per rotation per second.
        .withClosedLoopController(0.1, 0, 0)
        .withFeedforward(new SimpleMotorFeedforward(0, 0.12, 0))
        // Wheel radius and mass, used by the standalone simulation.
        .withMomentOfInertia(Inches.of(2), Pounds.of(1))
        .withZeroPower(MotorMode.COAST)
        .withStatorCurrentLimit(Amps.of(40))
        .withTelemetry(name, TelemetryVerbosity.HIGH);
  }

  public ShooterSubsystem() {
    // A mechanism class would do this for you. Without one, set up the telemetry
    // (and live tuning) of each motor controller once.
    top.setupTelemetry();
    bottom.setupTelemetry();
  }

  /** Spin each wheel at its own speed. Runs until interrupted. */
  public Command spin(AngularVelocity topSpeed, AngularVelocity bottomSpeed) {
    return startRun(
        () -> {
          // A stop() command may have turned the closed loop off.
          top.startClosedLoopController();
          bottom.startClosedLoopController();
        },
        () -> {
          top.setVelocity(topSpeed);
          bottom.setVelocity(bottomSpeed);
        })
        .withName("Shooter Spin");
  }

  /** Let both wheels coast. For the default command. */
  public Command stop() {
    return startRun(
        () -> {
          top.stopClosedLoopController();
          bottom.stopClosedLoopController();
        },
        () -> {
          top.setDutyCycle(0);
          bottom.setDutyCycle(0);
        })
        .withName("Shooter Stop");
  }

  /** True while both wheels are within 100 RPM of their speeds. */
  public Trigger atSpeed(AngularVelocity topSpeed, AngularVelocity bottomSpeed) {
    return new Trigger(() ->
        top.getMechanismVelocity().isNear(topSpeed, RPM.of(100))
            && bottom.getMechanismVelocity().isNear(bottomSpeed, RPM.of(100)));
  }

  @Override
  public void periodic() {
    top.updateTelemetry();
    bottom.updateTelemetry();
  }

  @Override
  public void simulationPeriodic() {
    top.simIterate();
    bottom.simIterate();
  }
}
```
{% endcode %}

Bind it in `RobotContainer`:

{% code title="RobotContainer.java (fragment)" %}
```java
shooter.setDefaultCommand(shooter.stop());
// Backspin: bottom wheel faster than the top.
m_driverController.rightTrigger().whileTrue(shooter.spin(RPM.of(2500), RPM.of(3500)));
```
{% endcode %}
{% endtab %}

{% tab title="Commands v3" %}
{% code title="mechanisms/ShooterMechanism.java" lineNumbers="true" %}
```java
package first.robot.mechanisms;

import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.Inches;
import static org.wpilib.units.Units.Pounds;
import static org.wpilib.units.Units.RPM;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;
import org.wpilib.command3.Trigger;
import org.wpilib.hardware.bus.CANPort;
import org.wpilib.math.controller.SimpleMotorFeedforward;
import org.wpilib.math.system.DCMotor;
import org.wpilib.units.measure.AngularVelocity;
import yams.commands3.config.SmartMotorControllerConfig;
import yams.core.gearing.MechanismGearing;
import yams.core.motorcontrollers.SmartMotorController;
import yams.core.motorcontrollers.enums.ControlMode;
import yams.core.motorcontrollers.enums.MotorMode;
import yams.core.motorcontrollers.local.SparkWrapper;
import yams.core.telemetry.enums.TelemetryVerbosity;

public class ShooterMechanism implements Mechanism {

  private final SmartMotorController top = new SparkWrapper(
      new SparkMax(CANPort.CAN_S0, 4, MotorType.kBrushless), DCMotor.getNEO(1), wheelConfig("TopWheel"));

  private final SmartMotorController bottom = new SparkWrapper(
      new SparkMax(CANPort.CAN_S0, 5, MotorType.kBrushless), DCMotor.getNEO(1), wheelConfig("BottomWheel"));

  /** Both wheels are built the same way, so one method makes both configs. */
  private SmartMotorControllerConfig wheelConfig(String name) {
    return new SmartMotorControllerConfig(this)
        .withControlMode(ControlMode.CLOSED_LOOP)
        .withGearing(new MechanismGearing(1))
        // PLACEHOLDER gains. kV is volts per rotation per second.
        .withClosedLoopController(0.1, 0, 0)
        .withFeedforward(new SimpleMotorFeedforward(0, 0.12, 0))
        // Wheel radius and mass, used by the standalone simulation.
        .withMomentOfInertia(Inches.of(2), Pounds.of(1))
        .withZeroPower(MotorMode.COAST)
        .withStatorCurrentLimit(Amps.of(40))
        .withTelemetry(name, TelemetryVerbosity.HIGH);
  }

  public ShooterMechanism() {
    // A mechanism class would do this for you. Without one, set up the telemetry
    // (and live tuning) of each motor controller once.
    top.setupTelemetry();
    bottom.setupTelemetry();
  }

  /** Spin each wheel at its own speed. Runs until interrupted. */
  public Command spin(AngularVelocity topSpeed, AngularVelocity bottomSpeed) {
    return run(coroutine -> {
      // An idle() command may have turned the closed loop off.
      top.startClosedLoopController();
      bottom.startClosedLoopController();
      while (true) {
        top.setVelocity(topSpeed);
        bottom.setVelocity(bottomSpeed);
        coroutine.yield();
      }
    }).named("Shooter Spin");
  }

  /** Let both wheels coast at the lowest priority, so any other shooter command can take over. */
  public Command idle() {
    return run(coroutine -> {
      top.stopClosedLoopController();
      bottom.stopClosedLoopController();
      while (true) {
        top.setDutyCycle(0);
        bottom.setDutyCycle(0);
        coroutine.yield();
      }
    }).withPriority(Command.LOWEST_PRIORITY).named("Shooter Idle");
  }

  /** True while both wheels are within 100 RPM of their speeds. */
  public Trigger atSpeed(AngularVelocity topSpeed, AngularVelocity bottomSpeed) {
    return new Trigger(() ->
        top.getMechanismVelocity().isNear(topSpeed, RPM.of(100))
            && bottom.getMechanismVelocity().isNear(bottomSpeed, RPM.of(100)));
  }

  /** Call this from Robot.robotPeriodic(), before the scheduler runs. */
  public void periodic() {
    top.updateTelemetry();
    bottom.updateTelemetry();
  }

  /** Robot.simulationInit() runs this in its own Notifier, every robot loop. */
  public void simulationPeriodic() {
    top.simIterate();
    bottom.simIterate();
  }
}
```
{% endcode %}

In `Robot`, set `shooter.setDefaultCommand(shooter.idle())`, call `periodic()` from `robotPeriodic()` and start the simulation `Notifier` in `simulationInit()`, as in the [Shooter/FlyWheels tutorial](../tutorials/shooter-flywheels/commands.md). Then in your opmode:

{% code title="ExampleTeleop.java (fragment)" %}
```java
// Backspin: bottom wheel faster than the top.
controller.rightTrigger().whileTrue(robot.shooter.spin(RPM.of(2500), RPM.of(3500)));
```
{% endcode %}
{% endtab %}
{% endtabs %}

Each motor's simulation runs on its own, from the moment of inertia in its config, so `simIterate()` works without a mechanism class.

## Check it

1. Start the simulation and open **NetworkTables > Mechanisms** in your dashboard. There is one entry per motor: `TopWheel` and `BottomWheel`.
2. Hold the right trigger. `TopWheel`'s `mechanism/velocity` should rise to about 41.7 rotations per second (2500 RPM), and `BottomWheel`'s to about 58.3 (3500 RPM), **at the same time**.
3. Release it. Both coast down.

## If it did not work

* **Only one wheel spins.** Two commands that require this subsystem are fighting, so the second cancels the first. Set both motors from one command, as `spin` does.
* **The wheels do not start again after `stop`.** `stop` turned the closed loop off. Call `startClosedLoopController()` when a velocity command starts, as `spin` does.
* **The wheels settle well below their speeds.** `kV` is too low; see [How do I shoot with a shooter?](how-to-shoot-with-a-shooter.md)

## Next

Feed the game piece only once both wheels are at speed, with `atSpeed(...)` and a feeder: [How do I shoot with a shooter?](how-to-shoot-with-a-shooter.md)
