# Building an elevator subsystem

**Goal:** Create an `Elevator` subsystem with height-based position control and physics simulation.

The gains in this guide are placeholders, as [the overview](guides.md#the-gains-in-these-guides) explains. The setup matches [Building an arm subsystem](building-an-arm.md) apart from the linear units, the drum and the carriage, so this guide spends its words on those.

***

{% stepper %}
{% step %}
#### Configure the motor for linear travel

The motor config must account for how rotations convert to linear distance. Either:

* Call `.withMechanismCircumference(distancePerRotation)` where the value equals `2 * pi * drumRadius`.
* Or call `.withDrumRadius(radius)` which computes the circumference for you.

Set the circumference before calling `.withSoftLimits(Distance, Distance)`, because the linear limits are converted to rotations using it.

```java
// "this" is the Subsystem (Commands v2) or Mechanism (Commands v3) that owns the motor.
SmartMotorControllerConfig motorConfig = new SmartMotorControllerConfig(this)
    .withMotorInverted(false)
    .withZeroPower(MotorMode.BRAKE)
    .withGearing(new MechanismGearing(GearBox.fromReductionStages(9.0)))
    .withMechanismCircumference(Meters.of(2 * Math.PI * 0.025))  // drum circumference
    .withCascadingElevatorStages(2)                               // 2-stage cascade
    .withClosedLoopController(1.2, 0.0, 0.05)                  // Placeholder PID gains.
    .withFeedforward(new ElevatorFeedforward(0.1, 0.4, 0.02)) // Placeholder kS, kG, kV.
    .withSoftLimits(Meters.of(0.0), Meters.of(1.2))
    .withStartingPosition(Meters.of(0.0))
    .withStatorCurrentLimit(Amps.of(60))
    .withTelemetry("ElevatorMotor", TelemetryVerbosity.HIGH);

SmartMotorController motor = new TalonFXWrapper(
    new TalonFX(2, new CANBus(CANPort.CAN_S0)),
    DCMotor.getKrakenX60(1),
    motorConfig
);
```

**Check it:** the project builds.
{% endstep %}

{% step %}
#### Create an `ElevatorConfig`

Carriage weight is required for simulation to model gravity correctly. Drum radius and cascade stage count are configured on `SmartMotorControllerConfig`, as in [Configure the motor for linear travel](building-an-elevator.md#configure-the-motor-for-linear-travel).

```java
ElevatorConfig elevatorConfig = new ElevatorConfig()
    .withCarriageWeight(Kilograms.of(4.5))
    .withHardLimits(Meters.of(0.0), Meters.of(1.2))
    .withTelemetry("Elevator", TelemetryVerbosity.HIGH);
```

{% hint style="info" %}
Carriage weight, hard limits, and a starting position (`SmartMotorControllerConfig.withStartingPosition(Distance)`) are required for simulation. Without them, the `Elevator` constructor throws when running in simulation: an `ElevatorConfigurationException` for the carriage weight or hard limits, and a `SmartMotorControllerConfigurationException` for the starting position. Drum circumference and cascade stages belong on `SmartMotorControllerConfig` via `.withMechanismCircumference()` and `.withCascadingElevatorStages()`.
{% endhint %}

**Check it:** the project builds.
{% endstep %}

{% step %}
#### Construct the `Elevator`

Import `Elevator` from `yams.commands2.mechanisms` (Commands v2) or `yams.commands3.mechanisms` (Commands v3).

```java
Elevator elevator = new Elevator(elevatorConfig, motor);
```

**Check it:** run the simulation. The code starts without an exception. If it throws, the message names the missing setting, as the hint above lists.
{% endstep %}

{% step %}
#### Integrate into a subsystem

{% tabs %}
{% tab title="Commands v2" %}
```java
package first.robot.subsystems;

import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.Kilograms;
import static org.wpilib.units.Units.Meters;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.TalonFX;
import org.wpilib.command2.Command;
import org.wpilib.command2.SubsystemBase;
import org.wpilib.command2.button.Trigger;
import org.wpilib.hardware.bus.CANPort;
import org.wpilib.math.controller.ElevatorFeedforward;
import org.wpilib.math.system.DCMotor;
import org.wpilib.units.measure.Distance;
import yams.commands2.config.SmartMotorControllerConfig;
import yams.commands2.mechanisms.Elevator;
import yams.core.gearing.GearBox;
import yams.core.gearing.MechanismGearing;
import yams.core.mechanisms.config.ElevatorConfig;
import yams.core.motorcontrollers.SmartMotorController;
import yams.core.motorcontrollers.enums.MotorMode;
import yams.core.motorcontrollers.remote.TalonFXWrapper;
import yams.core.telemetry.enums.TelemetryVerbosity;

public class ElevatorSubsystem extends SubsystemBase {

    private final SmartMotorController motor;
    private final Elevator elevator;

    public ElevatorSubsystem() {
        SmartMotorControllerConfig motorConfig = new SmartMotorControllerConfig(this)
            .withMotorInverted(false)
            .withZeroPower(MotorMode.BRAKE)
            .withGearing(new MechanismGearing(GearBox.fromReductionStages(9.0)))
            .withMechanismCircumference(Meters.of(2 * Math.PI * 0.025))  // drum circumference
            .withCascadingElevatorStages(2)                               // 2-stage cascade
            .withClosedLoopController(1.2, 0.0, 0.05)                  // Placeholder PID gains.
            .withFeedforward(new ElevatorFeedforward(0.1, 0.4, 0.02)) // Placeholder kS, kG, kV.
            .withSoftLimits(Meters.of(0.0), Meters.of(1.2))
            .withStartingPosition(Meters.of(0.0))
            .withStatorCurrentLimit(Amps.of(60))
            .withTelemetry("ElevatorMotor", TelemetryVerbosity.HIGH);

        motor = new TalonFXWrapper(
            new TalonFX(2, new CANBus(CANPort.CAN_S0)),
            DCMotor.getKrakenX60(1),
            motorConfig
        );

        ElevatorConfig elevatorConfig = new ElevatorConfig()
            .withCarriageWeight(Kilograms.of(4.5))
            .withHardLimits(Meters.of(0.0), Meters.of(1.2))
            .withTelemetry("Elevator", TelemetryVerbosity.HIGH);

        elevator = new Elevator(elevatorConfig, motor);
    }

    /** Move to a fixed height and hold it. */
    public Command setHeightCommand(Distance height) {
        return elevator.setHeight(height);
    }

    /** Move to height, finish when within tolerance. */
    public Command runToHeight(Distance height, Distance tolerance) {
        return elevator.runTo(height, tolerance);
    }

    /** Trigger that is true while the elevator is at or below a height. */
    public Trigger lte(Distance height) {
        return elevator.lte(height);
    }

    /** Trigger that is true while the elevator is at or above a height. */
    public Trigger gte(Distance height) {
        return elevator.gte(height);
    }

    public Distance getHeight() {
        return elevator.getHeight();
    }

    @Override
    public void periodic() {
        elevator.updateTelemetry();
    }

    @Override
    public void simulationPeriodic() {
        elevator.simIterate();
    }
}
```
{% endtab %}

{% tab title="Commands v3" %}
The configuration is identical. The class implements `Mechanism`, the YAMS imports come from `yams.commands3`, and the command factories use coroutines.

```java
package first.robot.mechanisms;

import static org.wpilib.units.Units.Meters;

import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;
import org.wpilib.command3.Trigger;
import org.wpilib.units.measure.Distance;
import yams.commands3.config.SmartMotorControllerConfig;
import yams.commands3.mechanisms.Elevator;
// ... the yams.core, org.wpilib.math, and vendor imports are the same as Commands v2

public class ElevatorMechanism implements Mechanism {

    public static final Distance STOW_HEIGHT = Meters.of(0);
    public static final Distance TOLERANCE   = Meters.of(0.02);

    private final SmartMotorController motor;
    private final Elevator elevator;

    public ElevatorMechanism() {
        SmartMotorControllerConfig motorConfig = new SmartMotorControllerConfig(this)
            // ... same builder calls as Commands v2
            .withTelemetry("ElevatorMotor", TelemetryVerbosity.HIGH);

        motor = new TalonFXWrapper(
            new TalonFX(2, new CANBus(CANPort.CAN_S0)),
            DCMotor.getKrakenX60(1),
            motorConfig
        );

        ElevatorConfig elevatorConfig = new ElevatorConfig()
            .withCarriageWeight(Kilograms.of(4.5))
            .withHardLimits(Meters.of(0.0), Meters.of(1.2))
            .withTelemetry("Elevator", TelemetryVerbosity.HIGH);

        elevator = new Elevator(elevatorConfig, motor);
    }

    /** Hold a height at the lowest priority, so any other elevator command can take over. */
    public Command hold(Distance height) {
        return run(coroutine -> coroutine.await(elevator.setHeight(height)))
            .withPriority(Command.LOWEST_PRIORITY)
            .named("Elevator Hold " + height.in(Meters) + " m");
    }

    /** Default command: hold the stow height. */
    public Command stow() {
        return hold(STOW_HEIGHT);
    }

    /** Move to height, finish when within tolerance. */
    public Command moveTo(Distance height) {
        return elevator.runTo(height, TOLERANCE);
    }

    public Trigger gte(Distance height) {
        return elevator.gte(height);
    }

    public Distance getHeight() {
        return elevator.getHeight();
    }

    /** Call from Robot.robotPeriodic(), before the scheduler runs. */
    public void periodic() {
        elevator.updateTelemetry();
    }

    /** Run in its own Notifier, started from Robot.simulationInit(). */
    public void simulationPeriodic() {
        elevator.simIterate();
    }
}
```

In `Robot`, set the default command with `elevator.setDefaultCommand(elevator.stow())`, call `elevator.periodic()` from `robotPeriodic()` before `scheduler.run()`, and start the simulation in `simulationInit()` with `startSimulation("Elevator", elevator::simulationPeriodic)`, as in [Building an arm subsystem](building-an-arm.md#integrate-into-a-subsystem).
{% endtab %}
{% endtabs %}

**Check it:** run the simulation and open **NetworkTables -> Mechanisms -> Elevator -> mechanism** in the Sim GUI. The carriage is drawn at 0 m. Schedule `setHeightCommand(Meters.of(1.0))` (Commands v2) or `moveTo(Meters.of(1.0))` (Commands v3) from a button, and the carriage rises to 1 m.
{% endstep %}

{% step %}
#### Wire up triggers

`Elevator` exposes `gte(height)`, `lte(height)`, `between(start, end)`, `near(height, tolerance)`, `max()`, and `min()` triggers. Use them to gate game piece actions.

```java
Trigger atBottom = elevatorSubsystem.lte(Meters.of(0.05));
Trigger atScore  = elevatorSubsystem.gte(Meters.of(1.0));
```

**Check it:** bind `atScore` to a command that prints, such as `atScore.onTrue(Commands.print("At score"))`. Raise the elevator to 1 m in simulation and the message prints.
{% endstep %}
{% endstepper %}

## If it did not work

* **The elevator moves the wrong distance.** The circumference or the gearing is wrong. The circumference is the distance the carriage moves for one turn of the drum or sprocket; for a cascade, set the stage count too.
* **It throws when the code starts in simulation.** A carriage weight, the hard limits, or a starting position inside the hard limits is missing.
* **The carriage falls or never reaches the setpoint.** Expected with the placeholder gains, especially `kG`. Tune them as in [Tune it](https://yams.yassrobotics.com/documentation/tutorials/elevator/tune).

[Troubleshooting](https://yams.yassrobotics.com/documentation/tutorials/troubleshooting) covers more.

***

## Examples

{% @github-files/github-code-block url="https://github.com/Yet-Another-Software-Suite/YAMS/blob/master/examples/commands2/simple_robot-2027/src/main/java/first/robot/subsystems/ElevatorSubsystem.java" %}

{% @github-files/github-code-block url="https://github.com/Yet-Another-Software-Suite/YAMS/blob/master/examples/commands2/simple_robot-2027/src/main/java/first/robot/subsystems/ExponentiallyProfiledElevatorSubsystem.java" %}

Commands v3 version of the elevator:

{% @github-files/github-code-block url="https://github.com/Yet-Another-Software-Suite/YAMS/blob/master/examples/commands3/simple_robot-2027/src/main/java/first/robot/mechanisms/ElevatorMechanism.java" %}

***

## Related pages

* [Elevator](../java-reference/mechanisms/elevator.md)
* [ElevatorConfig](../java-reference/config/elevator-config.md)
* [Configuring a Motor Controller](configuring-a-motor.md)
* [Simulation](simulation.md)
