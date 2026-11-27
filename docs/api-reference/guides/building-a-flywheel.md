# Building a flywheel subsystem

**Goal:** Create a `FlyWheel` subsystem with velocity control and physics simulation.

The gains in this guide are placeholders, as [the overview](guides.md#the-gains-in-these-guides) explains. The [Shooter/FlyWheels tutorial](https://yams.yassrobotics.com/documentation/tutorials/shooter-flywheels) explains each setting.

***

{% stepper %}
{% step %}
#### Configure the motor for velocity control

Use `withClosedLoopController` for the closed loop PID and `withFeedforward(SimpleMotorFeedforward)` to reduce steady-state error. Optionally add an exponential profile for smooth spin-up.

```java
// "this" is the Subsystem (Commands v2) or Mechanism (Commands v3) that owns the motor.
SmartMotorControllerConfig motorConfig = new SmartMotorControllerConfig(this)
    .withMotorInverted(false)
    .withZeroPower(MotorMode.COAST)
    .withGearing(new MechanismGearing(GearBox.fromReductionStages(1.5)))
    .withClosedLoopController(0.0003, 0.0, 0.0)                 // Placeholder PID gains.
    .withFeedforward(new SimpleMotorFeedforward(0.1, 0.002, 0.0)) // Placeholder kS, kV, kA.
    .withMomentOfInertia(Meters.of(0.0508), Kilograms.of(0.18))
    .withStatorCurrentLimit(Amps.of(80))
    .withTelemetry("ShooterMotor", TelemetryVerbosity.HIGH);

SmartMotorController motor = new TalonFXWrapper(
    new TalonFX(3, new CANBus(CANPort.CAN_S0)),
    DCMotor.getKrakenX60(1),
    motorConfig
);
```

**Check it:** the project builds.
{% endstep %}

{% step %}
#### Create a `FlyWheelConfig`

Wheel diameter is used for linear velocity conversion. Moment of inertia (for simulation) is set on `SmartMotorControllerConfig` via `withMomentOfInertia`.

```java
FlyWheelConfig flywheelConfig = new FlyWheelConfig()
    .withDiameter(Meters.of(0.1016))
    .withTelemetry("Shooter", TelemetryVerbosity.HIGH);
```

**Check it:** the project builds.
{% endstep %}

{% step %}
#### Construct the `FlyWheel`

Import `FlyWheel` from `yams.commands2.mechanisms` (Commands v2) or `yams.commands3.mechanisms` (Commands v3).

```java
FlyWheel flyWheel = new FlyWheel(flywheelConfig, motor);
```

**Check it:** run the simulation. The code starts without an exception.
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
import static org.wpilib.units.Units.RPM;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.TalonFX;
import org.wpilib.command2.Command;
import org.wpilib.command2.SubsystemBase;
import org.wpilib.command2.button.Trigger;
import org.wpilib.hardware.bus.CANPort;
import org.wpilib.math.controller.SimpleMotorFeedforward;
import org.wpilib.math.system.DCMotor;
import org.wpilib.units.measure.AngularVelocity;
import yams.commands2.config.SmartMotorControllerConfig;
import yams.commands2.mechanisms.FlyWheel;
import yams.core.gearing.GearBox;
import yams.core.gearing.MechanismGearing;
import yams.core.mechanisms.config.FlyWheelConfig;
import yams.core.motorcontrollers.SmartMotorController;
import yams.core.motorcontrollers.enums.MotorMode;
import yams.core.motorcontrollers.remote.TalonFXWrapper;
import yams.core.telemetry.enums.TelemetryVerbosity;

public class ShooterSubsystem extends SubsystemBase {

    private final SmartMotorController motor;
    private final FlyWheel flyWheel;

    private static final AngularVelocity SHOOT_SPEED     = RPM.of(4500);
    private static final AngularVelocity SPEED_TOLERANCE = RPM.of(100);

    public ShooterSubsystem() {
        SmartMotorControllerConfig motorConfig = new SmartMotorControllerConfig(this)
            .withMotorInverted(false)
            .withZeroPower(MotorMode.COAST)
            .withGearing(new MechanismGearing(GearBox.fromReductionStages(1.5)))
            .withClosedLoopController(0.0003, 0.0, 0.0)                 // Placeholder PID gains.
            .withFeedforward(new SimpleMotorFeedforward(0.1, 0.002, 0.0)) // Placeholder kS, kV, kA.
            .withMomentOfInertia(Meters.of(0.0508), Kilograms.of(0.18))
            .withStatorCurrentLimit(Amps.of(80))
            .withTelemetry("ShooterMotor", TelemetryVerbosity.HIGH);

        motor = new TalonFXWrapper(
            new TalonFX(3, new CANBus(CANPort.CAN_S0)),
            DCMotor.getKrakenX60(1),
            motorConfig
        );

        FlyWheelConfig flywheelConfig = new FlyWheelConfig()
            .withDiameter(Meters.of(0.1016))
            .withTelemetry("Shooter", TelemetryVerbosity.HIGH);

        flyWheel = new FlyWheel(flywheelConfig, motor);
    }

    /** Spin to a target velocity and hold indefinitely. */
    public Command spinUpCommand(AngularVelocity velocity) {
        return flyWheel.run(velocity);
    }

    /** Spin up and finish once within tolerance; use to gate shooting. */
    public Command spinUpAndWait(AngularVelocity velocity) {
        return flyWheel.runTo(velocity, SPEED_TOLERANCE);
    }

    /** Trigger that is true whenever the wheel is near the shoot speed. */
    public Trigger readyToShoot() {
        return flyWheel.near(SHOOT_SPEED, SPEED_TOLERANCE);
    }

    @Override
    public void periodic() {
        flyWheel.updateTelemetry();
    }

    @Override
    public void simulationPeriodic() {
        flyWheel.simIterate();
    }
}
```
{% endtab %}

{% tab title="Commands v3" %}
The configuration is identical. The class implements `Mechanism`, the YAMS imports come from `yams.commands3`, and the command factories use coroutines.

```java
package first.robot.mechanisms;

import static org.wpilib.units.Units.RPM;

import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;
import org.wpilib.command3.Trigger;
import org.wpilib.units.measure.AngularVelocity;
import yams.commands3.config.SmartMotorControllerConfig;
import yams.commands3.mechanisms.FlyWheel;
// ... the yams.core, org.wpilib.math, and vendor imports are the same as Commands v2

public class ShooterMechanism implements Mechanism {

    private final SmartMotorController motor;
    private final FlyWheel flyWheel;

    public static final AngularVelocity SHOOT_SPEED     = RPM.of(4500);
    public static final AngularVelocity SPEED_TOLERANCE = RPM.of(100);

    public ShooterMechanism() {
        SmartMotorControllerConfig motorConfig = new SmartMotorControllerConfig(this)
            // ... same builder calls as Commands v2
            .withTelemetry("ShooterMotor", TelemetryVerbosity.HIGH);

        motor = new TalonFXWrapper(
            new TalonFX(3, new CANBus(CANPort.CAN_S0)),
            DCMotor.getKrakenX60(1),
            motorConfig
        );

        FlyWheelConfig flywheelConfig = new FlyWheelConfig()
            .withDiameter(Meters.of(0.1016))
            .withTelemetry("Shooter", TelemetryVerbosity.HIGH);

        flyWheel = new FlyWheel(flywheelConfig, motor);
    }

    /** Hold a velocity at the lowest priority, so any other shooter command can take over. */
    public Command hold(AngularVelocity velocity) {
        return run(coroutine -> coroutine.await(flyWheel.run(velocity)))
            .withPriority(Command.LOWEST_PRIORITY)
            .named("Shooter Hold " + velocity.in(RPM) + " RPM");
    }

    /** Default command: hold 0 RPM. */
    public Command stop() {
        return hold(RPM.of(0));
    }

    /** Spin up and finish once within tolerance; use to gate shooting. */
    public Command spinUpAndWait(AngularVelocity velocity) {
        return flyWheel.runTo(velocity, SPEED_TOLERANCE);
    }

    /** Trigger that is true whenever the wheel is near the shoot speed. */
    public Trigger readyToShoot() {
        return flyWheel.near(SHOOT_SPEED, SPEED_TOLERANCE);
    }

    /** Call from Robot.robotPeriodic(), before the scheduler runs. */
    public void periodic() {
        flyWheel.updateTelemetry();
    }

    /** Run in its own Notifier, started from Robot.simulationInit(). */
    public void simulationPeriodic() {
        flyWheel.simIterate();
    }
}
```

In `Robot`, set the default command with `shooter.setDefaultCommand(shooter.stop())`, call `shooter.periodic()` from `robotPeriodic()` before `scheduler.run()`, and start the simulation in `simulationInit()` with `startSimulation("Shooter", shooter::simulationPeriodic)`, as in [Building an arm subsystem](building-an-arm.md#integrate-into-a-subsystem).
{% endtab %}
{% endtabs %}

**Check it:** run the simulation and open **NetworkTables -> Mechanisms -> Shooter -> mechanism** in the Sim GUI. Schedule `spinUpCommand(RPM.of(4500))` (Commands v2) or `hold(RPM.of(4500))` (Commands v3) from a button, and the wheel's line spins. Its velocity, in the telemetry next to it, climbs toward 4500 RPM.
{% endstep %}

{% step %}
#### Sequence shooter commands

Use `runTo` to block until the wheel is at speed before feeding a game piece. The indexer is its own subsystem with its own YAMS `FlyWheel` (rollers, feeders, and indexers are flywheels too, even when run open loop).

{% tabs %}
{% tab title="Commands v2" %}
```java
Command shoot = shooter.spinUpAndWait(SHOOT_SPEED)
    .andThen(indexer.feedCommand());
```
{% endtab %}

{% tab title="Commands v3" %}
```java
Command shoot = Command.noRequirements(coroutine -> {
    coroutine.await(shooter.spinUpAndWait(ShooterMechanism.SHOOT_SPEED));
    coroutine.await(indexer.feedCommand());
}).named("Shoot");
```
{% endtab %}
{% endtabs %}

Or bind the `readyToShoot()` trigger to allow feeding at any time once the wheel stabilizes:

```java
shooter.readyToShoot().whileTrue(indexer.feedCommand());
```

**Check it:** in simulation, the indexer only runs once the shooter is within 100 RPM of the shoot speed.
{% endstep %}
{% endstepper %}

## If it did not work

* **The wheel never spins in simulation.** Commands v2: check that `simulationPeriodic()` calls `flyWheel.simIterate()`. Commands v3: check that `simulationInit()` calls `startSimulation(...)` for the mechanism.
* **The wheel spins up very slowly or never reaches speed.** Expected with the placeholder gains: most of a flywheel's voltage comes from `kV`. Tune them as in [Tune it](https://yams.yassrobotics.com/documentation/tutorials/shooter-flywheels/tune).
* **`spinUpAndWait` never ends.** The wheel never gets within the tolerance. Check the velocity in telemetry, then the gains or the tolerance.

[Troubleshooting](https://yams.yassrobotics.com/documentation/tutorials/troubleshooting) covers more.

***

## Notes

{% hint style="info" %}
`runTo(velocity, tolerance)` ends once the wheel reaches the target. Use it in command sequences where the next step should not begin until the wheel is up to speed. `near(velocity, tolerance)` (exposed as a `Trigger`, formerly `isNear`) stays true as long as the wheel remains within tolerance; use it for continuously gated logic like a conveyor that feeds whenever the shooter is ready.
{% endhint %}

***

## Examples

{% @github-files/github-code-block url="https://github.com/Yet-Another-Software-Suite/YAMS/blob/master/examples/commands2/simple_robot-2027/src/main/java/first/robot/subsystems/ShooterSubsystem.java" %}

Commands v3 version of the shooter:

{% @github-files/github-code-block url="https://github.com/Yet-Another-Software-Suite/YAMS/blob/master/examples/commands3/simple_robot-2027/src/main/java/first/robot/mechanisms/ShooterMechanism.java" %}

***

## Related pages

* [FlyWheel](../java-reference/mechanisms/flywheel.md)
* [FlyWheelConfig](../java-reference/config/flywheel-config.md)
* [Configuring a Motor Controller](configuring-a-motor.md)
* [Simulation](simulation.md)
