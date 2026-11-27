---
icon: layer-group
---

# Elevator and shooter

An elevator and a shooter follow the same steps as the arm: an IO interface, one IO class for the real robot and simulation, an empty replay IO class, and a subsystem that only reads the logged inputs. This page has the code for each. Work through the arm first: the steps and checks are the same. Tune the placeholder gains as in the [Elevator](../elevator/) and [Shooter/FlyWheels](../shooter-flywheels/) tutorials.

The differences from the arm:

* **The elevator** logs its height in meters, takes a `Distance`, and needs a drum radius and a carriage weight for its simulation.
* **The shooter** is a `FlyWheel`: it logs a velocity, takes an `AngularVelocity`, and has no position.

The code below is for Commands v2. For Commands v3, make the same changes as in the arm's Commands v3 tabs:

* The interface and IO classes go in `mechanisms/akit` with `package first.robot.mechanisms.akit;`.
* The IO classes import `org.wpilib.command3.Mechanism`, `yams.commands3.config.SmartMotorControllerConfig` and `yams.commands3.mechanisms.Elevator` (or `FlyWheel`), and their constructor takes a `Mechanism`.
* The subsystem is a `Mechanism`: use the **Java, Commands v3** tab.

## Elevator

### IO interface

```java
package first.robot.subsystems.akit;

import org.littletonrobotics.junction.AutoLog;
import org.wpilib.units.measure.Distance;

public interface ElevatorIO {

  @AutoLog
  public static class ElevatorIOInputs {
    public double positionMeters = 0.0;
    public double velocityMetersPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double supplyCurrentAmps = 0.0;
    public double statorCurrentAmps = 0.0;
    public double temperatureCelsius = 0.0;
    public double targetPositionMeters = 0.0;
  }

  default void updateInputs(ElevatorIOInputs inputs) {}

  default void setTargetHeight(Distance height) {}

  default void stop() {}

  /** Run the elevator's simulation for one loop. */
  default void simIterate() {}

  /** Publish the elevator's YAMS telemetry. */
  default void updateTelemetry() {}
}
```

### IO class (real and sim)

```java
package first.robot.subsystems.akit;

import static org.wpilib.units.Units.*;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.TalonFX;
import org.wpilib.hardware.bus.CANPort;
import org.wpilib.math.controller.ElevatorFeedforward;
import org.wpilib.math.system.DCMotor;
import org.wpilib.command2.SubsystemBase;
import org.wpilib.units.measure.Distance;
import yams.commands2.config.SmartMotorControllerConfig;
import yams.core.gearing.GearBox;
import yams.core.gearing.MechanismGearing;
import yams.core.mechanisms.config.ElevatorConfig;
import yams.commands2.mechanisms.Elevator;
import yams.core.motorcontrollers.SmartMotorController;
import yams.core.motorcontrollers.remote.TalonFXWrapper;
import yams.core.telemetry.enums.TelemetryVerbosity;

/**
 * The elevator's IO class, used on the real robot and in simulation.
 * It builds a YAMS Elevator, and reads the logged inputs from its SmartMotorController.
 */
public class ElevatorIOTalonFX implements ElevatorIO {

  private static final int CAN_ID = 2;

  private final Elevator elevator;
  private final SmartMotorController motor;

  public ElevatorIOTalonFX(SubsystemBase subsystem) {
    TalonFX talonFX = new TalonFX(CAN_ID, new CANBus(CANPort.CAN_S0));

    // The motor controller's config: gearing, drum, gains, limits and starting height.
    SmartMotorControllerConfig smcConfig = new SmartMotorControllerConfig(subsystem)
        .withGearing(new MechanismGearing(GearBox.fromReductionStages(5, 4)))
        .withDrumRadius(Inches.of(0.75))
        .withStartingPosition(Meters.of(0.5))
        // Placeholder gains: tune them for your elevator as in the Elevator tutorial.
        .withClosedLoopController(50, 0, 0)
        .withFeedforward(new ElevatorFeedforward(0, 0, 0))
        .withSoftLimits(Meters.of(0.02), Meters.of(1.2))
        .withTrapezoidalProfile(MetersPerSecond.of(1.0), MetersPerSecondPerSecond.of(2.0));

    // The motor controller, a Kraken X60 on a TalonFX.
    SmartMotorController smc = new TalonFXWrapper(talonFX, DCMotor.getKrakenX60(1), smcConfig);

    // The elevator: carriage weight and hard stops, used by the simulation.
    ElevatorConfig elevatorConfig = new ElevatorConfig()
        .withCarriageWeight(Pounds.of(10))
        .withHardLimits(Meters.of(0), Meters.of(1.5))
        .withTelemetry("Elevator", TelemetryVerbosity.HIGH);

    // The YAMS elevator. In simulation it simulates itself.
    this.elevator = new Elevator(elevatorConfig, smc);
    this.motor = elevator.getMotor();
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    // Read everything the subsystem needs from the SmartMotorController
    inputs.positionMeters = motor.getMeasurementPosition().in(Meters);
    inputs.velocityMetersPerSec = motor.getMeasurementVelocity().in(MetersPerSecond);
    inputs.appliedVolts = motor.getVoltage().in(Volts);
    inputs.supplyCurrentAmps = motor.getSupplyCurrent().map(c -> c.in(Amps)).orElse(0.0);
    inputs.statorCurrentAmps = motor.getStatorCurrent().in(Amps);
    inputs.temperatureCelsius = motor.getTemperature().in(Celsius);
    // The setpoint is stored as a mechanism angle; convert it to a height with the config
    inputs.targetPositionMeters = motor.getMechanismPositionSetpoint()
        .map(a -> motor.getConfig().convertFromMechanism(a).in(Meters)).orElse(0.0);
  }

  @Override
  public void setTargetHeight(Distance height) {
    motor.setPosition(height);
  }

  @Override
  public void stop() {
    motor.setVoltage(Volts.of(0));
  }

  @Override
  public void simIterate() {
    elevator.simIterate();
  }

  @Override
  public void updateTelemetry() {
    elevator.updateTelemetry();
  }
}
```

### Replay IO class

```java
package first.robot.subsystems.akit;

/** Empty IO implementation for log replay. */
public class ElevatorIOReplay implements ElevatorIO {}
```

### Subsystem

{% tabs %}
{% tab title="Java, Commands v2" %}
```java
package first.robot.subsystems.akit;

import static org.wpilib.units.Units.Meters;

import java.util.function.Function;
import org.littletonrobotics.junction.Logger;
import org.wpilib.command2.Command;
import org.wpilib.command2.SubsystemBase;
import org.wpilib.units.measure.Distance;

public class ElevatorSubsystem extends SubsystemBase {

  private final ElevatorIO io;
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

  public ElevatorSubsystem(Function<SubsystemBase, ElevatorIO> ioFactory) {
    this.io = ioFactory.apply(this);
  }

  @Override
  public void periodic() {
    io.updateTelemetry();
    io.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);
  }

  @Override
  public void simulationPeriodic() {
    io.simIterate();
  }

  /** Command to move the elevator to a height and hold it. This command does not end on its own. */
  public Command setHeight(Distance height) {
    return run(() -> io.setTargetHeight(height))
        .withName("Elevator.setHeight(" + height + ")");
  }

  /** The logged elevator height, so it replays correctly. */
  public Distance getHeight() {
    return Meters.of(inputs.positionMeters);
  }
}
```
{% endtab %}

{% tab title="Java, Commands v3" %}
```java
package first.robot.mechanisms.akit;

import static org.wpilib.units.Units.Meters;

import java.util.function.Function;
import org.littletonrobotics.junction.Logger;
import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;
import org.wpilib.units.measure.Distance;

public class ElevatorMechanism implements Mechanism {

  private final ElevatorIO io;
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

  public ElevatorMechanism(Function<Mechanism, ElevatorIO> ioFactory) {
    this.io = ioFactory.apply(this);
  }

  /** Robot.robotPeriodic() calls this every loop, before the scheduler runs. */
  public void periodic() {
    io.updateTelemetry();
    io.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);
  }

  /** Robot.simulationPeriodic() calls this every loop in simulation. */
  public void simulationPeriodic() {
    io.simIterate();
  }

  /** Command to move the elevator to a height and hold it. This command does not end on its own. */
  public Command setHeight(Distance height) {
    return run(coroutine -> {
      while (true) {
        io.setTargetHeight(height);
        coroutine.yield();
      }
    }).named("Elevator.setHeight(" + height + ")");
  }

  /** The logged elevator height, so it replays correctly. */
  public Distance getHeight() {
    return Meters.of(inputs.positionMeters);
  }
}
```
{% endtab %}
{% endtabs %}

Create it with `new ElevatorSubsystem(ElevatorIOTalonFX::new)` (or `new ElevatorMechanism(...)`), with the same replay check as the arm.

**Check:** in simulation, run `setHeight(Meters.of(1))`. `AdvantageKit/Elevator/PositionMeters` rises from `0.5` to about `1.0` within about two seconds.

## Shooter

### IO interface

```java
package first.robot.subsystems.akit;

import org.littletonrobotics.junction.AutoLog;
import org.wpilib.units.measure.AngularVelocity;

public interface ShooterIO {

  @AutoLog
  public static class ShooterIOInputs {
    public double velocityRotationsPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double supplyCurrentAmps = 0.0;
    public double statorCurrentAmps = 0.0;
    public double temperatureCelsius = 0.0;
    public double targetVelocityRotationsPerSec = 0.0;
  }

  default void updateInputs(ShooterIOInputs inputs) {}

  default void setTargetVelocity(AngularVelocity velocity) {}

  default void stop() {}

  /** Run the shooter's simulation for one loop. */
  default void simIterate() {}

  /** Publish the shooter's YAMS telemetry. */
  default void updateTelemetry() {}
}
```

### IO class (real and sim)

```java
package first.robot.subsystems.akit;

import static org.wpilib.units.Units.*;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.TalonFX;
import org.wpilib.hardware.bus.CANPort;
import org.wpilib.math.controller.SimpleMotorFeedforward;
import org.wpilib.math.system.DCMotor;
import org.wpilib.command2.SubsystemBase;
import org.wpilib.units.measure.AngularVelocity;
import yams.commands2.config.SmartMotorControllerConfig;
import yams.core.gearing.GearBox;
import yams.core.gearing.MechanismGearing;
import yams.core.mechanisms.config.FlyWheelConfig;
import yams.commands2.mechanisms.FlyWheel;
import yams.core.motorcontrollers.SmartMotorController;
import yams.core.motorcontrollers.remote.TalonFXWrapper;
import yams.core.telemetry.enums.TelemetryVerbosity;

/**
 * The shooter's IO class, used on the real robot and in simulation.
 * It builds a YAMS FlyWheel, and reads the logged inputs from its SmartMotorController.
 */
public class ShooterIOTalonFX implements ShooterIO {

  private static final int CAN_ID = 3;

  private final FlyWheel flywheel;
  private final SmartMotorController motor;

  public ShooterIOTalonFX(SubsystemBase subsystem) {
    TalonFX talonFX = new TalonFX(CAN_ID, new CANBus(CANPort.CAN_S0));

    // The motor controller's config: direct drive, and placeholder gains to tune
    // as in the Shooter tutorial.
    SmartMotorControllerConfig smcConfig = new SmartMotorControllerConfig(subsystem)
        .withGearing(new MechanismGearing(GearBox.fromReductionStages(1)))
        .withClosedLoopController(0.5, 0, 0)
        .withFeedforward(new SimpleMotorFeedforward(0.1, 0.12, 0.01));

    // The motor controller, a Kraken X60 on a TalonFX.
    SmartMotorController smc = new TalonFXWrapper(talonFX, DCMotor.getKrakenX60(1), smcConfig);

    // The flywheel: its diameter, used by the simulation.
    FlyWheelConfig flywheelConfig = new FlyWheelConfig()
        .withDiameter(Inches.of(4))
        .withTelemetry("Shooter", TelemetryVerbosity.HIGH);

    // The YAMS flywheel. In simulation it simulates itself.
    this.flywheel = new FlyWheel(flywheelConfig, smc);
    this.motor = flywheel.getMotor();
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    // Read everything the subsystem needs from the SmartMotorController
    inputs.velocityRotationsPerSec = motor.getMechanismVelocity().in(RotationsPerSecond);
    inputs.appliedVolts = motor.getVoltage().in(Volts);
    inputs.supplyCurrentAmps = motor.getSupplyCurrent().map(c -> c.in(Amps)).orElse(0.0);
    inputs.statorCurrentAmps = motor.getStatorCurrent().in(Amps);
    inputs.temperatureCelsius = motor.getTemperature().in(Celsius);
    inputs.targetVelocityRotationsPerSec = motor.getMechanismSetpointVelocity()
        .map(v -> v.in(RotationsPerSecond)).orElse(0.0);
  }

  @Override
  public void setTargetVelocity(AngularVelocity velocity) {
    motor.setVelocity(velocity);
  }

  @Override
  public void stop() {
    motor.setVoltage(Volts.of(0));
  }

  @Override
  public void simIterate() {
    flywheel.simIterate();
  }

  @Override
  public void updateTelemetry() {
    flywheel.updateTelemetry();
  }
}
```

### Replay IO class

```java
package first.robot.subsystems.akit;

/** Empty IO implementation for log replay. */
public class ShooterIOReplay implements ShooterIO {}
```

### Subsystem

{% tabs %}
{% tab title="Java, Commands v2" %}
```java
package first.robot.subsystems.akit;

import static org.wpilib.units.Units.RotationsPerSecond;

import java.util.function.Function;
import org.littletonrobotics.junction.Logger;
import org.wpilib.command2.Command;
import org.wpilib.command2.SubsystemBase;
import org.wpilib.units.measure.AngularVelocity;

public class ShooterSubsystem extends SubsystemBase {

  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

  public ShooterSubsystem(Function<SubsystemBase, ShooterIO> ioFactory) {
    this.io = ioFactory.apply(this);
  }

  @Override
  public void periodic() {
    io.updateTelemetry();
    io.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);
  }

  @Override
  public void simulationPeriodic() {
    io.simIterate();
  }

  /** Command to spin the shooter at a velocity. This command does not end on its own. */
  public Command setVelocity(AngularVelocity velocity) {
    return run(() -> io.setTargetVelocity(velocity))
        .withName("Shooter.setVelocity(" + velocity + ")");
  }

  /** The logged shooter velocity, so it replays correctly. */
  public AngularVelocity getVelocity() {
    return RotationsPerSecond.of(inputs.velocityRotationsPerSec);
  }
}
```
{% endtab %}

{% tab title="Java, Commands v3" %}
```java
package first.robot.mechanisms.akit;

import static org.wpilib.units.Units.RotationsPerSecond;

import java.util.function.Function;
import org.littletonrobotics.junction.Logger;
import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;
import org.wpilib.units.measure.AngularVelocity;

public class ShooterMechanism implements Mechanism {

  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

  public ShooterMechanism(Function<Mechanism, ShooterIO> ioFactory) {
    this.io = ioFactory.apply(this);
  }

  /** Robot.robotPeriodic() calls this every loop, before the scheduler runs. */
  public void periodic() {
    io.updateTelemetry();
    io.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);
  }

  /** Robot.simulationPeriodic() calls this every loop in simulation. */
  public void simulationPeriodic() {
    io.simIterate();
  }

  /** Command to spin the shooter at a velocity. This command does not end on its own. */
  public Command setVelocity(AngularVelocity velocity) {
    return run(coroutine -> {
      while (true) {
        io.setTargetVelocity(velocity);
        coroutine.yield();
      }
    }).named("Shooter.setVelocity(" + velocity + ")");
  }

  /** The logged shooter velocity, so it replays correctly. */
  public AngularVelocity getVelocity() {
    return RotationsPerSecond.of(inputs.velocityRotationsPerSec);
  }
}
```
{% endtab %}
{% endtabs %}

Create it with `new ShooterSubsystem(ShooterIOTalonFX::new)` (or `new ShooterMechanism(...)`), with the same replay check as the arm.

**Check:** in simulation, run `setVelocity(RPM.of(3000))`. `AdvantageKit/Shooter/VelocityRotationsPerSec` rises to about `50` (3000 RPM) within about four seconds.
