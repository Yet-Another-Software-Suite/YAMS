---
icon: arrow-right-to-bracket
---

# How do I set up an intake roller?

**Goal:** spin an intake, roller, feeder or indexer in and out from buttons, with YAMS telemetry and simulation.

In YAMS a roller is a `FlyWheel`, even when it only ever needs a fixed voltage. Using `FlyWheel` gets you telemetry, simulation and live tuning for free, and lets you switch to speed control later without rewriting the subsystem. Put the motor in `ControlMode.OPEN_LOOP` and drive it with `set` (duty cycle, -1 to 1) or `setVoltage`.

Give each roller its own subsystem (Commands v2) or `Mechanism` (Commands v3): **one YAMS mechanism per subsystem**, so live tuning works and commands for the intake never block the shooter. If you are new to the parts used here, the [Shooter/FlyWheels tutorial](../tutorials/shooter-flywheels/) builds the same kind of class step by step.

## The intake class

This example uses a SPARK MAX with CAN ID 9 driving a NEO through a 3:1 reduction to a 2 inch roller. Change the CAN ID, gearing and diameter to match your robot.

{% tabs %}
{% tab title="Commands v2" %}
{% code title="subsystems/IntakeSubsystem.java" lineNumbers="true" %}
```java
package first.robot.subsystems;

import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.Inches;
import static org.wpilib.units.Units.Pounds;
import static org.wpilib.units.Units.Volts;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import org.wpilib.command2.Command;
import org.wpilib.command2.SubsystemBase;
import org.wpilib.hardware.bus.CANPort;
import org.wpilib.math.system.DCMotor;
import yams.commands2.config.SmartMotorControllerConfig;
import yams.commands2.mechanisms.FlyWheel;
import yams.core.gearing.MechanismGearing;
import yams.core.mechanisms.config.FlyWheelConfig;
import yams.core.motorcontrollers.SmartMotorController;
import yams.core.motorcontrollers.enums.ControlMode;
import yams.core.motorcontrollers.enums.MotorMode;
import yams.core.motorcontrollers.local.SparkWrapper;
import yams.core.telemetry.enums.TelemetryVerbosity;

public class IntakeSubsystem extends SubsystemBase {

  private final SmartMotorControllerConfig smcConfig = new SmartMotorControllerConfig(this)
      // Open loop: no PID or feedforward, the motor just gets a duty cycle or voltage.
      .withControlMode(ControlMode.OPEN_LOOP)
      // 3:1 from the motor to the roller.
      .withGearing(new MechanismGearing(3))
      // Roller size and mass, used by the simulation.
      .withMomentOfInertia(Inches.of(1), Pounds.of(0.5))
      .withMotorInverted(false)
      .withZeroPower(MotorMode.COAST)
      .withStatorCurrentLimit(Amps.of(40))
      .withTelemetry("IntakeMotor", TelemetryVerbosity.HIGH);

  private final SparkMax spark = new SparkMax(CANPort.CAN_S0, 9, MotorType.kBrushless);

  private final SmartMotorController motor = new SparkWrapper(spark, DCMotor.getNEO(1), smcConfig);

  private final FlyWheelConfig intakeConfig = new FlyWheelConfig()
      .withDiameter(Inches.of(2))
      .withTelemetry("Intake", TelemetryVerbosity.HIGH);

  private final FlyWheel intake = new FlyWheel(intakeConfig, motor);

  /** Pull a game piece in. Runs until interrupted. */
  public Command intake() {
    return intake.setVoltage(Volts.of(8));
  }

  /** Push a game piece out. Runs until interrupted. */
  public Command eject() {
    return intake.setVoltage(Volts.of(-6));
  }

  /** Stop the roller. For the default command. */
  public Command stop() {
    return intake.set(0);
  }

  @Override
  public void periodic() {
    intake.updateTelemetry();
  }

  @Override
  public void simulationPeriodic() {
    intake.simIterate();
  }
}
```
{% endcode %}

Bind it in `RobotContainer`:

{% code title="RobotContainer.java (fragment)" %}
```java
private final IntakeSubsystem intake = new IntakeSubsystem();

// In the constructor or configureBindings():
intake.setDefaultCommand(intake.stop());
m_driverController.leftBumper().whileTrue(intake.intake());
m_driverController.rightBumper().whileTrue(intake.eject());
```
{% endcode %}
{% endtab %}

{% tab title="Commands v3" %}
{% code title="mechanisms/IntakeMechanism.java" lineNumbers="true" %}
```java
package first.robot.mechanisms;

import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.Inches;
import static org.wpilib.units.Units.Pounds;
import static org.wpilib.units.Units.Volts;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;
import org.wpilib.hardware.bus.CANPort;
import org.wpilib.math.system.DCMotor;
import yams.commands3.config.SmartMotorControllerConfig;
import yams.commands3.mechanisms.FlyWheel;
import yams.core.gearing.MechanismGearing;
import yams.core.mechanisms.config.FlyWheelConfig;
import yams.core.motorcontrollers.SmartMotorController;
import yams.core.motorcontrollers.enums.ControlMode;
import yams.core.motorcontrollers.enums.MotorMode;
import yams.core.motorcontrollers.local.SparkWrapper;
import yams.core.telemetry.enums.TelemetryVerbosity;

public class IntakeMechanism implements Mechanism {

  private final SmartMotorControllerConfig smcConfig = new SmartMotorControllerConfig(this)
      // Open loop: no PID or feedforward, the motor just gets a duty cycle or voltage.
      .withControlMode(ControlMode.OPEN_LOOP)
      // 3:1 from the motor to the roller.
      .withGearing(new MechanismGearing(3))
      // Roller size and mass, used by the simulation.
      .withMomentOfInertia(Inches.of(1), Pounds.of(0.5))
      .withMotorInverted(false)
      .withZeroPower(MotorMode.COAST)
      .withStatorCurrentLimit(Amps.of(40))
      .withTelemetry("IntakeMotor", TelemetryVerbosity.HIGH);

  private final SparkMax spark = new SparkMax(CANPort.CAN_S0, 9, MotorType.kBrushless);

  private final SmartMotorController motor = new SparkWrapper(spark, DCMotor.getNEO(1), smcConfig);

  private final FlyWheelConfig intakeConfig = new FlyWheelConfig()
      .withDiameter(Inches.of(2))
      .withTelemetry("Intake", TelemetryVerbosity.HIGH);

  private final FlyWheel intake = new FlyWheel(intakeConfig, motor);

  /** Pull a game piece in. Runs until interrupted. */
  public Command intake() {
    return intake.setVoltage(Volts.of(8));
  }

  /** Push a game piece out. Runs until interrupted. */
  public Command eject() {
    return intake.setVoltage(Volts.of(-6));
  }

  /** Stop the roller at the lowest priority, so any other intake command can take over. */
  public Command idle() {
    return run(coroutine -> coroutine.await(intake.set(0)))
        .withPriority(Command.LOWEST_PRIORITY)
        .named("Intake Idle");
  }

  /** Call this from Robot.robotPeriodic(), before the scheduler runs. */
  public void periodic() {
    intake.updateTelemetry();
  }

  /** Robot.simulationInit() runs this in its own Notifier, every robot loop. */
  public void simulationPeriodic() {
    intake.simIterate();
  }
}
```
{% endcode %}

In `Robot`, create it, set the default command, call `periodic()` from `robotPeriodic()` and start its simulation `Notifier` in `simulationInit()`, as in the [Shooter/FlyWheels tutorial](../tutorials/shooter-flywheels/commands.md). Then bind it in your opmode:

{% code title="ExampleTeleop.java (fragment)" %}
```java
// In Robot(): intake.setDefaultCommand(intake.idle());
controller.leftBumper().whileTrue(robot.intake.intake());
controller.rightBumper().whileTrue(robot.intake.eject());
```
{% endcode %}
{% endtab %}
{% endtabs %}

`setVoltage` is usually better than `set` for a roller: 8 volts is 8 volts whether the battery is at 12.5 or 11.5 volts, so the intake behaves the same all match.

## Check it

1. Start the simulation (or enable the robot) and open **NetworkTables > Mechanisms > Intake** in your dashboard.
2. Hold the left bumper. The roller's speed should rise and settle, and the motor's output voltage should read about 8 volts.
3. Release it. The default command takes over and the roller coasts to a stop.

## If it did not work

* **The roller spins the wrong way.** Flip `withMotorInverted`, so that `intake()` pulls the game piece in.
* **The roller does nothing in teleop.** Another command is using the intake, or the default command was never set. In Commands v3, check that the bindings are in the opmode you selected.
* **Something else.** See [Troubleshooting](../tutorials/troubleshooting.md).

## Next

Stop the intake by itself when a game piece is in: [How do I detect a game piece from motor current?](../how-to-motor-controllers-and-sensors/how-do-i-sense-activity-based-off-of-the-current.md)
