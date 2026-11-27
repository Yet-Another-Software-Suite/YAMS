---
icon: cubes
---

# Steps 3 and 4: The subsystem and RobotContainer

## Step 3: Create the subsystem

The subsystem (a `Mechanism` in Commands v3) uses the IO interface, and doesn't know whether it is running on real hardware or in simulation. It only reads the logged inputs, so in replay it makes the same decisions it made on the robot.

It builds its own IO class: the IO class needs the subsystem for its `SmartMotorControllerConfig`, and the subsystem does not exist yet while it is still being created. So the constructor takes a factory, a function that builds the IO class from the subsystem, such as `ArmIOTalonFX::new`.

Create it in the same `akit` folder as the IO classes.

{% tabs %}
{% tab title="Java, Commands v2" %}
`ArmSubsystem.java`:

```java
package first.robot.subsystems.akit;

import static org.wpilib.units.Units.*;

import java.util.function.Function;
import org.wpilib.command2.Command;
import org.wpilib.command2.SubsystemBase;
import org.wpilib.units.measure.Angle;
import org.littletonrobotics.junction.Logger;

public class ArmSubsystem extends SubsystemBase {

  private final ArmIO io;
  private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();

  /**
   * @param ioFactory Builds the IO class for this subsystem, for example ArmIOTalonFX::new.
   *                  The subsystem passes itself in, because the IO class's
   *                  SmartMotorControllerConfig needs the subsystem.
   */
  public ArmSubsystem(Function<SubsystemBase, ArmIO> ioFactory) {
    this.io = ioFactory.apply(this);
  }

  @Override
  public void periodic() {
    // Update the arm's telemetry
    io.updateTelemetry();
    // Update and log inputs every cycle
    io.updateInputs(inputs);
    Logger.processInputs("Arm", inputs);
  }

  @Override
  public void simulationPeriodic() {
    io.simIterate();
  }

  /**
   * Command to move the arm to a target angle and hold it.
   * This command does not end on its own.
   */
  public Command setAngle(Angle angle) {
    return run(() -> io.setTargetAngle(angle))
        .withName("Arm.setAngle(" + angle + ")");
  }

  /**
   * Command to move the arm to a target angle and finish when reached.
   * Be careful with default commands, see the hint below.
   */
  public Command goToAngle(Angle angle, Angle tolerance) {
    return run(() -> io.setTargetAngle(angle))
        .until(() -> isNear(angle, tolerance))
        .withName("Arm.goToAngle(" + angle + ")");
  }

  /** Returns true if the logged arm position is within tolerance of a target, using WPILib's isNear(). */
  public boolean isNear(Angle target, Angle tolerance) {
    return getAngle().isNear(target, tolerance);
  }

  /** The logged arm position, so it replays correctly. */
  public Angle getAngle() {
    return Rotations.of(inputs.positionRotations);
  }

  /** Command to stop the arm. */
  public Command stop() {
    return runOnce(() -> io.stop()).withName("Arm.stop");
  }

}
```
{% endtab %}

{% tab title="Java, Commands v3" %}
`ArmMechanism.java`. A Commands v3 `Mechanism` has no `periodic()` that the scheduler calls, so `Robot` calls `periodic()` and `simulationPeriodic()` itself in Step 4. Commands are coroutines that set the target and `yield()` every loop.

```java
package first.robot.mechanisms.akit;

import static org.wpilib.units.Units.*;

import java.util.function.Function;
import org.littletonrobotics.junction.Logger;
import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;
import org.wpilib.command3.Trigger;
import org.wpilib.units.measure.Angle;

public class ArmMechanism implements Mechanism {

  private final ArmIO io;
  private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();

  /**
   * @param ioFactory Builds the IO class for this mechanism, for example ArmIOTalonFX::new.
   *                  The mechanism passes itself in, because the IO class's
   *                  SmartMotorControllerConfig needs the mechanism.
   */
  public ArmMechanism(Function<Mechanism, ArmIO> ioFactory) {
    this.io = ioFactory.apply(this);
  }

  /** Robot.robotPeriodic() calls this every loop, before the scheduler runs. */
  public void periodic() {
    // Update the arm's telemetry
    io.updateTelemetry();
    // Update and log inputs every cycle
    io.updateInputs(inputs);
    Logger.processInputs("Arm", inputs);
  }

  /** Robot.simulationPeriodic() calls this every loop in simulation. */
  public void simulationPeriodic() {
    io.simIterate();
  }

  /**
   * Command to move the arm to a target angle and hold it.
   * This command does not end on its own.
   */
  public Command setAngle(Angle angle) {
    return run(coroutine -> {
      while (true) {
        io.setTargetAngle(angle);
        coroutine.yield();
      }
    }).named("Arm.setAngle(" + angle + ")");
  }

  /** Command to move the arm to a target angle and finish when it is within tolerance. */
  public Command goToAngle(Angle angle, Angle tolerance) {
    return run(coroutine -> {
      while (!isNear(angle, tolerance)) {
        io.setTargetAngle(angle);
        coroutine.yield();
      }
    }).named("Arm.goToAngle(" + angle + ")");
  }

  /** A Trigger built from the logged arm position, so it replays correctly. */
  public Trigger near(Angle target, Angle tolerance) {
    return new Trigger(() -> isNear(target, tolerance));
  }

  /** Returns true if the logged arm position is within tolerance of a target, using WPILib's isNear(). */
  public boolean isNear(Angle target, Angle tolerance) {
    return getAngle().isNear(target, tolerance);
  }

  /** The logged arm position, so it replays correctly. */
  public Angle getAngle() {
    return Rotations.of(inputs.positionRotations);
  }

  /** Command to stop the arm. */
  public Command stop() {
    return run(coroutine -> io.stop()).named("Arm.stop");
  }
}
```
{% endtab %}
{% endtabs %}

`isNear()` and `getAngle()` read the logged inputs, not the motor, so anything that waits on them (such as `goToAngle`) waits the same way in replay. `isNear()` is WPILib's, and works with any unit type (`Angle`, `Distance`, `AngularVelocity`).

{% hint style="warning" %}
See [When mechanism commands end](../../understanding/run-vs-runto.md) for important information about when commands end and how that interacts with default commands.
{% endhint %}

## Step 4: Wire it up

With YAMS's simulation, you don't need conditional logic for real vs sim. The same IO class works for both.

{% tabs %}
{% tab title="Java, Commands v2" %}
Replace `RobotContainer.java` with this. `getAutonomousCommand()` stays, because `Robot` calls it.

```java
package first.robot;

import static org.wpilib.units.Units.Degrees;

import first.robot.Constants.OperatorConstants;
import first.robot.subsystems.akit.ArmIOTalonFX;
import first.robot.subsystems.akit.ArmSubsystem;
import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import org.wpilib.command2.button.CommandGamepad;

public class RobotContainer {

  // Same IO class for the real robot and simulation. YAMS handles the difference.
  private final ArmSubsystem arm = new ArmSubsystem(ArmIOTalonFX::new);

  private final CommandGamepad driverController =
      new CommandGamepad(OperatorConstants.DRIVER_CONTROLLER_PORT);

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    // Commands work identically on the real robot and in simulation
    driverController.faceDown().whileTrue(arm.setAngle(Degrees.of(45)));
    driverController.faceRight().whileTrue(arm.setAngle(Degrees.of(-45)));
  }

  /** The command Robot runs in autonomous. */
  public Command getAutonomousCommand() {
    // No autonomous yet.
    return Commands.none();
  }
}
```
{% endtab %}

{% tab title="Java, Commands v3" %}
In `Robot.java`, create the arm after `Logger.start()`, and call its `periodic()` before the scheduler runs. This is the whole file:

```java
package first.robot;

import first.robot.constants.DriverConstants;
import first.robot.mechanisms.ExampleMechanism;
import first.robot.mechanisms.akit.ArmIOTalonFX;
import first.robot.mechanisms.akit.ArmMechanism;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedOpModeRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import org.wpilib.command3.Scheduler;
import org.wpilib.command3.button.CommandGamepad;
import org.wpilib.framework.RobotBase;

public class Robot extends LoggedOpModeRobot {
  public static enum Mode {
    /** Running on a real robot. */
    REAL,
    /** Running a physics simulator. */
    SIM,
    /** Replaying from a log file. */
    REPLAY
  }

  /** Change to Mode.REPLAY to replay a log in simulation. */
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  // Created in the constructor, after the logger starts.
  final ExampleMechanism exampleMechanism;
  final ArmMechanism arm;
  final CommandGamepad exampleController =
      new CommandGamepad(DriverConstants.DRIVER_CONTROLLER_PORT);

  public Robot() {
    switch (currentMode) {
      case REAL:
        // On the SystemCore: write a log file and publish to NetworkTables.
        Logger.addDataReceiver(new WPILOGWriter());
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case SIM:
        // In simulation: publish to NetworkTables.
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case REPLAY:
        // Replaying a log: read it, and write the replayed values to a new log next to it.
        setUseTiming(false); // Run as fast as possible
        String logPath = LogFileUtil.findReplayLog();
        Logger.setReplaySource(new WPILOGReader(logPath));
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
        break;
    }

    // Start the AdvantageKit logger before creating the mechanisms.
    Logger.start();

    exampleMechanism = new ExampleMechanism();
    // Same IO class for the real robot and simulation. YAMS handles the difference.
    arm = new ArmMechanism(ArmIOTalonFX::new);
  }

  @Override
  public void robotPeriodic() {
    // Mechanisms have no periodic() of their own: log their inputs before the scheduler runs.
    arm.periodic();
    Scheduler.getDefault().run();
  }

  @Override
  public void simulationPeriodic() {
    arm.simulationPeriodic();
  }
}
```

Bind the buttons in `ExampleTeleop.java`:

```java
package first.robot;

import static org.wpilib.units.Units.Degrees;

import org.wpilib.opmode.OpMode;
import org.wpilib.opmode.Teleop;

@Teleop
public class ExampleTeleop implements OpMode {
  public ExampleTeleop(Robot robot) {
    // Commands work identically on the real robot and in simulation
    robot.exampleController.faceDown().whileTrue(robot.arm.setAngle(Degrees.of(45)));
    robot.exampleController.faceRight().whileTrue(robot.arm.setAngle(Degrees.of(-45)));
  }
}
```
{% endtab %}
{% endtabs %}

**Check:** run the code in simulation, and drive it as in [Simulate it](../arm/simulate.md) from the Arm tutorial: put a controller (or **Keyboard 0**) on the template's controller port, which is **Joystick\[0]** for Commands v2 and **Joystick\[1]** for Commands v3. For Commands v3, pick the `ExampleTeleop` op mode. Enable **Teleoperated** and hold A (`faceDown`).

* The arm swings to 45 degrees in **NetworkTables > Mechanisms > Arm > mechanism**.
* In AdvantageScope (**File > Connect to Simulator**), `AdvantageKit/Arm/PositionRotations` rises to about `0.125` (45 degrees) within about a second, and `AdvantageKit/Arm/TargetPositionRotations` is `0.125`.

**If it did not work:**

* **`ArmIOInputsAutoLogged` cannot be found:** the AutoLog code generator is not set up. See [Add the AutoLog code generator](setup.md#add-the-autolog-code-generator).
* **There is no `AdvantageKit` table, or nothing under `Arm`:** `Robot` does not call `Logger.start()`, or it is not a `LoggedRobot` (`LoggedOpModeRobot` for Commands v3). See [Make Robot a logged robot](setup.md#make-robot-a-logged-robot).
* **`Arm` has inputs, but the arm does not move:** check the controller is on the right joystick port, and the robot is enabled in **Teleoperated**. For Commands v3, check `Robot.robotPeriodic()` calls `Scheduler.getDefault().run()`.
* **The arm moves on the controller, but the logged values stay at 0:** the subsystem's `periodic()` is not running. For Commands v3, check `Robot.robotPeriodic()` calls `arm.periodic()`.

{% hint style="info" %}
**Compare to Traditional AdvantageKit**: Without YAMS, you'd need separate `ArmIOReal` and `ArmIOSim` classes with duplicate configuration. YAMS eliminates this duplication entirely.
{% endhint %}
