---
icon: link-slash
---

# Without an IO layer

For simpler projects, you can use YAMS mechanisms directly as shown in the official examples ([Commands v2 subsystems](../../../../examples/commands2/advantage_kit/java/first/robot/subsystems), [Commands v3 mechanisms](../../../../examples/commands3/advantage_kit/java/first/robot/mechanisms)):

{% tabs %}
{% tab title="Commands v2" %}
```java
package first.robot.subsystems;

import static org.wpilib.units.Units.*;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.TalonFX;
import org.wpilib.hardware.bus.CANPort;
import org.wpilib.math.controller.ArmFeedforward;
import org.wpilib.math.system.DCMotor;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Current;
import org.wpilib.units.measure.Voltage;
import org.wpilib.command2.Command;
import org.wpilib.command2.SubsystemBase;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;
import yams.commands2.config.SmartMotorControllerConfig;
import yams.commands2.mechanisms.Arm;
import yams.core.gearing.GearBox;
import yams.core.gearing.MechanismGearing;
import yams.core.mechanisms.config.ArmConfig;
import yams.core.motorcontrollers.SmartMotorController;
import yams.core.motorcontrollers.enums.ControlMode;
import yams.core.motorcontrollers.enums.MotorMode;
import yams.core.motorcontrollers.remote.TalonFXWrapper;
import yams.core.telemetry.enums.TelemetryVerbosity;

public class ArmSubsystem extends SubsystemBase {

  @AutoLog
  public static class ArmInputs {
    public Angle pivotPosition = Degrees.of(0);
    public AngularVelocity pivotVelocity = DegreesPerSecond.of(0);
    public Voltage pivotAppliedVolts = Volts.of(0);
    public Current pivotCurrent = Amps.of(0);
  }

  private final ArmInputsAutoLogged armInputs = new ArmInputsAutoLogged();
  private final SmartMotorController armSMC;
  private final Arm arm;

  public ArmSubsystem() {
    SmartMotorControllerConfig smcConfig = new SmartMotorControllerConfig(this)
        .withControlMode(ControlMode.CLOSED_LOOP)
        .withClosedLoopController(18, 0, 0.2)
        .withTrapezoidalProfile(DegreesPerSecond.of(458), DegreesPerSecondPerSecond.of(688))
        .withFeedforward(new ArmFeedforward(0, 0.89, 0, 0))
        .withGearing(new MechanismGearing(GearBox.fromReductionStages(12.5, 1)))
        .withZeroPower(MotorMode.BRAKE)
        .withStatorCurrentLimit(Amps.of(120))
        .withStartingPosition(Degrees.of(141));

    armSMC = new TalonFXWrapper(new TalonFX(40, new CANBus(CANPort.CAN_S0)), DCMotor.getFalcon500(1), smcConfig);

    ArmConfig armCfg = new ArmConfig()
        .withHardLimits(Degrees.of(-25), Degrees.of(141))
        .withLength(Feet.of(14.0 / 12))
        .withTelemetry("Arm", TelemetryVerbosity.HIGH);

    arm = new Arm(armCfg, armSMC);
  }

  public void updateInputs() {
    armInputs.pivotPosition = arm.getAngle();
    armInputs.pivotVelocity = armSMC.getMechanismVelocity();
    armInputs.pivotAppliedVolts = armSMC.getVoltage();
    armInputs.pivotCurrent = armSMC.getStatorCurrent();
  }

  public Command setAngle(Angle angle) {
    return arm.setAngle(angle);
  }

  @Override
  public void periodic() {
    updateInputs();
    Logger.processInputs("Arm", armInputs);
    arm.updateTelemetry();
  }

  @Override
  public void simulationPeriodic() {
    arm.simIterate();
  }
}
```
{% endtab %}

{% tab title="Commands v3" %}
In Commands v3 the class implements `Mechanism`, imports the `yams.commands3` bindings, and `Robot` calls `periodic()` and `simulationPeriodic()` itself. The inputs, configuration and `updateInputs()` are the same as the Commands v2 tab.

```java
package first.robot.mechanisms;

// Same static units, CTRE, WPILib math/units, AdvantageKit and yams.core imports as Commands v2, plus:
import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;
import org.wpilib.command3.Trigger;
import yams.commands3.config.SmartMotorControllerConfig;
import yams.commands3.mechanisms.Arm;

public class ArmMechanism implements Mechanism {

  // ArmInputs, armInputs, smcConfig, armSMC, armCfg, arm and updateInputs() as in Commands v2

  public Command setAngle(Angle angle) {
    return arm.setAngle(angle);
  }

  /** Trigger built from the logged angle, so it replays correctly. */
  public Trigger near(Angle angle, Angle within) {
    return new Trigger(() -> armInputs.pivotPosition.isNear(angle, within));
  }

  public void periodic() {
    updateInputs();
    Logger.processInputs("Arm", armInputs);
    arm.updateTelemetry();
  }

  public void simulationPeriodic() {
    arm.simIterate();
  }
}
```

```java
// Robot.java (extends LoggedRobot)
@Override
public void robotPeriodic() {
  drive.periodic();
  arm.periodic();
  elevator.periodic();
  shooter.periodic();
  indexer.periodic();
  scheduler.run();
}

@Override
public void simulationPeriodic() {
  drive.simulationPeriodic();
  arm.simulationPeriodic();
  elevator.simulationPeriodic();
  shooter.simulationPeriodic();
  indexer.simulationPeriodic();
}
```
{% endtab %}
{% endtabs %}

This pattern is simpler and **does support log replay**, but with a key difference: during replay, the full YAMS simulation runs alongside the logged data. This means:

* **Pro**: You get full simulation behavior during replay, which can help debug physics-related issues
* **Con**: Replay requires more processing power since every `SmartMotorController` and Mechanism simulates its physics

Choose the IO layer pattern if you need lightweight replay; choose this direct pattern if you prefer simplicity and don't mind the extra processing during replay.

**Check:** run the code in simulation and command the arm. In AdvantageScope, `AdvantageKit/Arm/PivotPosition` follows the arm.

Back to [AdvantageKit integration](./).
