---
icon: code
---

# Step 2: Implement the IO class

With YAMS, you only need **one IO implementation** that works for both real hardware and simulation. The `SmartMotorController` detects when it is running in simulation, and simulates the motor and the arm instead of talking to hardware.

The IO class builds the same YAMS arm as the [Arm tutorial](../arm/): a `SmartMotorControllerConfig`, a `TalonFXWrapper` and an `Arm`. It takes the subsystem (or mechanism) that owns it in its constructor, because the `SmartMotorControllerConfig` needs it. That class builds its own IO class in [Step 3](subsystem-and-robotcontainer.md), passing itself in.

## Arm IO implementation (works for real and sim)

Create `ArmIOTalonFX.java` in the same `akit` folder as `ArmIO.java`. Set `CAN_ID` to your arm motor's CAN ID.

{% tabs %}
{% tab title="Java, Commands v2" %}
```java
package first.robot.subsystems.akit;

import static org.wpilib.units.Units.*;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.TalonFX;
import org.wpilib.hardware.bus.CANPort;
import org.wpilib.math.controller.ArmFeedforward;
import org.wpilib.math.system.DCMotor;
import org.wpilib.command2.SubsystemBase;
import org.wpilib.units.measure.Angle;
import yams.commands2.config.SmartMotorControllerConfig;
import yams.core.gearing.GearBox;
import yams.core.gearing.MechanismGearing;
import yams.core.mechanisms.config.ArmConfig;
import yams.commands2.mechanisms.Arm;
import yams.core.motorcontrollers.SmartMotorController;
import yams.core.motorcontrollers.remote.TalonFXWrapper;
import yams.core.telemetry.enums.TelemetryVerbosity;

/**
 * The arm's IO class, used on the real robot and in simulation.
 * It builds a YAMS Arm, and reads the logged inputs from its SmartMotorController.
 */
public class ArmIOTalonFX implements ArmIO {

  private static final int CAN_ID = 1;

  private final Arm arm;
  private final SmartMotorController motor;

  public ArmIOTalonFX(SubsystemBase subsystem) {
    TalonFX talonFX = new TalonFX(CAN_ID, new CANBus(CANPort.CAN_S0));

    // The motor controller's config: gearing, gains, limits and starting position.
    SmartMotorControllerConfig smcConfig = new SmartMotorControllerConfig(subsystem)
        .withGearing(new MechanismGearing(GearBox.fromReductionStages(5, 4, 3)))
        // Placeholder gains: tune them for your arm as in the Arm tutorial.
        .withClosedLoopController(50, 0, 0)
        .withFeedforward(new ArmFeedforward(0, 0, 0))
        .withSoftLimits(Rotations.of(-0.25), Rotations.of(0.25))
        .withTrapezoidalProfile(RotationsPerSecond.of(1.0), RotationsPerSecondPerSecond.of(2.0))
        .withStartingPosition(Rotations.of(0));

    // The motor controller, a Kraken X60 on a TalonFX.
    SmartMotorController smc = new TalonFXWrapper(talonFX, DCMotor.getKrakenX60(1), smcConfig);

    // The arm: its length and hard stops, used by the simulation.
    ArmConfig armConfig = new ArmConfig()
        .withLength(Inches.of(18))
        .withHardLimits(Rotations.of(-0.3), Rotations.of(0.3))
        .withTelemetry("Arm", TelemetryVerbosity.HIGH);

    // The YAMS arm. In simulation it simulates itself.
    this.arm = new Arm(armConfig, smc);
    this.motor = arm.getMotor();
  }

  @Override
  public void updateInputs(ArmIOInputs inputs) {
    // Read everything the subsystem needs from the SmartMotorController
    inputs.positionRotations = motor.getMechanismPosition().in(Rotations);
    inputs.velocityRotationsPerSec = motor.getMechanismVelocity().in(RotationsPerSecond);
    inputs.appliedVolts = motor.getVoltage().in(Volts);
    inputs.supplyCurrentAmps = motor.getSupplyCurrent().map(c -> c.in(Amps)).orElse(0.0);
    inputs.statorCurrentAmps = motor.getStatorCurrent().in(Amps);
    inputs.temperatureCelsius = motor.getTemperature().in(Celsius);
    inputs.targetPositionRotations = motor.getMechanismPositionSetpoint()
        .map(a -> a.in(Rotations)).orElse(0.0);
  }

  @Override
  public void setTargetAngle(Angle angle) {
    motor.setPosition(angle);
  }

  @Override
  public void stop() {
    motor.setVoltage(Volts.of(0));
  }

  @Override
  public void simIterate() {
    arm.simIterate();
  }

  @Override
  public void updateTelemetry() {
    arm.updateTelemetry();
  }
}
```
{% endtab %}

{% tab title="Java, Commands v3" %}
The same class as Commands v2, with the `yams.commands3` imports, and a `Mechanism` in the constructor instead of a `SubsystemBase`.

```java
package first.robot.mechanisms.akit;

import static org.wpilib.units.Units.*;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.TalonFX;
import org.wpilib.hardware.bus.CANPort;
import org.wpilib.math.controller.ArmFeedforward;
import org.wpilib.math.system.DCMotor;
import org.wpilib.command3.Mechanism;
import org.wpilib.units.measure.Angle;
import yams.commands3.config.SmartMotorControllerConfig;
import yams.core.gearing.GearBox;
import yams.core.gearing.MechanismGearing;
import yams.core.mechanisms.config.ArmConfig;
import yams.commands3.mechanisms.Arm;
import yams.core.motorcontrollers.SmartMotorController;
import yams.core.motorcontrollers.remote.TalonFXWrapper;
import yams.core.telemetry.enums.TelemetryVerbosity;

/**
 * The arm's IO class, used on the real robot and in simulation.
 * It builds a YAMS Arm, and reads the logged inputs from its SmartMotorController.
 */
public class ArmIOTalonFX implements ArmIO {

  private static final int CAN_ID = 1;

  private final Arm arm;
  private final SmartMotorController motor;

  public ArmIOTalonFX(Mechanism mechanism) {
    TalonFX talonFX = new TalonFX(CAN_ID, new CANBus(CANPort.CAN_S0));

    // The motor controller's config: gearing, gains, limits and starting position.
    SmartMotorControllerConfig smcConfig = new SmartMotorControllerConfig(mechanism)
        .withGearing(new MechanismGearing(GearBox.fromReductionStages(5, 4, 3)))
        // Placeholder gains: tune them for your arm as in the Arm tutorial.
        .withClosedLoopController(50, 0, 0)
        .withFeedforward(new ArmFeedforward(0, 0, 0))
        .withSoftLimits(Rotations.of(-0.25), Rotations.of(0.25))
        .withTrapezoidalProfile(RotationsPerSecond.of(1.0), RotationsPerSecondPerSecond.of(2.0))
        .withStartingPosition(Rotations.of(0));

    // The motor controller, a Kraken X60 on a TalonFX.
    SmartMotorController smc = new TalonFXWrapper(talonFX, DCMotor.getKrakenX60(1), smcConfig);

    // The arm: its length and hard stops, used by the simulation.
    ArmConfig armConfig = new ArmConfig()
        .withLength(Inches.of(18))
        .withHardLimits(Rotations.of(-0.3), Rotations.of(0.3))
        .withTelemetry("Arm", TelemetryVerbosity.HIGH);

    // The YAMS arm. In simulation it simulates itself.
    this.arm = new Arm(armConfig, smc);
    this.motor = arm.getMotor();
  }

  @Override
  public void updateInputs(ArmIOInputs inputs) {
    // Read everything the mechanism needs from the SmartMotorController
    inputs.positionRotations = motor.getMechanismPosition().in(Rotations);
    inputs.velocityRotationsPerSec = motor.getMechanismVelocity().in(RotationsPerSecond);
    inputs.appliedVolts = motor.getVoltage().in(Volts);
    inputs.supplyCurrentAmps = motor.getSupplyCurrent().map(c -> c.in(Amps)).orElse(0.0);
    inputs.statorCurrentAmps = motor.getStatorCurrent().in(Amps);
    inputs.temperatureCelsius = motor.getTemperature().in(Celsius);
    inputs.targetPositionRotations = motor.getMechanismPositionSetpoint()
        .map(a -> a.in(Rotations)).orElse(0.0);
  }

  @Override
  public void setTargetAngle(Angle angle) {
    motor.setPosition(angle);
  }

  @Override
  public void stop() {
    motor.setVoltage(Volts.of(0));
  }

  @Override
  public void simIterate() {
    arm.simIterate();
  }

  @Override
  public void updateTelemetry() {
    arm.updateTelemetry();
  }
}
```
{% endtab %}
{% endtabs %}

The subsystem only talks to the arm through this class, and only reads the logged inputs. Don't call the YAMS `Arm`'s own commands (`run()`, `runTo()`) from the subsystem: they read the motor directly, so replay could not reproduce them.

{% hint style="success" %}
**Why this works**: YAMS's simulation is built into every wrapper. When `Robot.isSimulation()` is true, the `TalonFXWrapper` (and every other wrapper) uses the `DCMotor`, the gearing and the arm's length to simulate the physics. You get a simulation without writing any simulation-specific code.
{% endhint %}

**Check:** the project builds. Nothing runs yet: no subsystem creates this class until the next step.
