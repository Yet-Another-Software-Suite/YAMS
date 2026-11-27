---
description: >-
  A spin ratio between the two flywheels, and what to change when two motors
  drive one wheel.
icon: people-arrows
---

# Step 3: Spin control and followers

## A spin ratio

Three presets are enough to start with. For finer control, let the caller choose a base speed and how much spin to put on it:

* A ratio of **0** runs both wheels at the base speed: no spin.
* A ratio of **1** runs the lower wheel 25% faster and the upper wheel 25% slower: the most backspin.
* A ratio of **-1** does the opposite: the most topspin.

The average of the two speeds stays at the base speed, so changing the spin does not change how fast the ball leaves.

{% tabs %}
{% tab title="Java, Commands v2" %}
{% code title="subsystems/DoubleFlywheelSubsystem.java" %}
```java
/**
 * Spin both flywheels around a base speed, with a spin ratio between them.
 *
 * @param baseSpeed The average speed of the two flywheels.
 * @param spinRatio From -1 (the most topspin) to 1 (the most backspin). 0 is no spin.
 */
public Command setSpinShot(AngularVelocity baseSpeed, double spinRatio) {
  double ratio = Math.max(-1.0, Math.min(1.0, spinRatio));
  // At a ratio of 1 the lower flywheel runs 25% faster and the upper one 25% slower.
  double upperMultiplier = 1.0 - ratio * 0.25;
  double lowerMultiplier = 1.0 + ratio * 0.25;
  return setFlywheelSpeeds(baseSpeed.times(upperMultiplier), baseSpeed.times(lowerMultiplier));
}
```
{% endcode %}
{% endtab %}

{% tab title="Java, Commands v3" %}
{% code title="mechanisms/DoubleFlywheelMechanism.java" %}
```java
/**
 * Spin both flywheels around a base speed, with a spin ratio between them.
 *
 * @param baseSpeed The average speed of the two flywheels.
 * @param spinRatio From -1 (the most topspin) to 1 (the most backspin). 0 is no spin.
 */
public Command setSpinShot(AngularVelocity baseSpeed, double spinRatio) {
  double ratio = Math.max(-1.0, Math.min(1.0, spinRatio));
  // At a ratio of 1 the lower flywheel runs 25% faster and the upper one 25% slower.
  double upperMultiplier = 1.0 - ratio * 0.25;
  double lowerMultiplier = 1.0 + ratio * 0.25;
  return setFlywheelSpeeds(baseSpeed.times(upperMultiplier), baseSpeed.times(lowerMultiplier));
}
```
{% endcode %}
{% endtab %}

{% tab title="C++" %}
{% code title="include/subsystems/DoubleFlywheelSubsystem.hpp" %}
```cpp
/**
 * Spin both flywheels around a base speed, with a spin ratio between them.
 *
 * @param baseSpeed The average speed of the two flywheels.
 * @param spinRatio From -1 (the most topspin) to 1 (the most backspin). 0 is no spin.
 */
wpi::cmd::CommandPtr SetSpinShot(wpi::units::revolutions_per_minute_t baseSpeed, double spinRatio);
```
{% endcode %}

{% code title="cpp/subsystems/DoubleFlywheelSubsystem.cpp" %}
```cpp
wpi::cmd::CommandPtr DoubleFlywheelSubsystem::SetSpinShot(wpi::units::revolutions_per_minute_t baseSpeed,
                                                          double spinRatio) {
  double ratio = std::clamp(spinRatio, -1.0, 1.0);
  // At a ratio of 1 the lower flywheel runs 25% faster and the upper one 25% slower.
  double upperMultiplier = 1.0 - ratio * 0.25;
  double lowerMultiplier = 1.0 + ratio * 0.25;
  return SetFlywheelSpeeds(baseSpeed * upperMultiplier, baseSpeed * lowerMultiplier);
}
```
{% endcode %}
{% endtab %}
{% endtabs %}

**For your robot:** the 25% is how much the two wheels may differ at most. Raise it for more spin.

### Try it

Bind it to a button to see it work. The right bumper here shoots at 3000 RPM with half the most backspin: the upper wheel at 2625 RPM and the lower at 3375 RPM.

{% tabs %}
{% tab title="Java, Commands v2" %}
{% code title="RobotContainer.java" %}
```java
// Right bumper: 3000 RPM with some backspin.
driverController.rightBumper().onTrue(doubleFlywheel.setSpinShot(RPM.of(3000), 0.5));
```
{% endcode %}
{% endtab %}

{% tab title="Java, Commands v3" %}
{% code title="ExampleTeleop.java" %}
```java
// Right bumper: 3000 RPM with some backspin.
robot.exampleController.rightBumper().onTrue(robot.doubleFlywheel.setSpinShot(RPM.of(3000), 0.5));
```
{% endcode %}
{% endtab %}

{% tab title="C++" %}
{% code title="cpp/RobotContainer.cpp" %}
```cpp
// Right bumper: 3000 RPM with some backspin.
driverController.RightBumper().OnTrue(doubleFlywheel.SetSpinShot(3000_rpm, 0.5));
```
{% endcode %}
{% endtab %}
{% endtabs %}

**Check it:** in simulation, graph the two speeds as in [step 2](commands.md#check-it-in-simulation) and press the right bumper. The upper line settles at about 44 rotations per second (2625 RPM) and the lower at about 56 (3375 RPM).

### The complete file

{% tabs %}
{% tab title="Java, Commands v2" %}
The highlighted lines are new in this step.

<pre class="language-java" data-title="subsystems/DoubleFlywheelSubsystem.java" data-line-numbers><code class="lang-java">package first.robot.subsystems;

import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.KilogramSquareMeters;
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
import yams.core.gearing.GearBox;
import yams.core.gearing.MechanismGearing;
import yams.core.motorcontrollers.SmartMotorController;
import yams.core.motorcontrollers.enums.ControlMode;
import yams.core.motorcontrollers.enums.MotorMode;
import yams.core.motorcontrollers.remote.TalonFXWrapper;
import yams.core.telemetry.enums.TelemetryVerbosity;

public class DoubleFlywheelSubsystem extends SubsystemBase {

  // Each flywheel gets its own config, so that each can be given its own speed.
  private final SmartMotorControllerConfig upperConfig = new SmartMotorControllerConfig(this)
      .withControlMode(ControlMode.CLOSED_LOOP)
      // Feedback and feedforward constants, on the real robot and in simulation.
      .withClosedLoopController(0.1, 0, 0)
      .withSimClosedLoopController(0.1, 0, 0)
      .withFeedforward(new SimpleMotorFeedforward(0.1, 0.12, 0))
      .withSimFeedforward(new SimpleMotorFeedforward(0.1, 0.12, 0))
      // Gearing from the motor rotor to the flywheel: 1:1, the wheel is on the motor's shaft.
      .withGearing(new MechanismGearing(GearBox.fromReductionStages(1)))
      // There is no FlyWheel mechanism, so the simulation gets the wheel's moment of
      // inertia from here: a 4 inch, 1 lb (0.45 kg) wheel as a solid disc is
      // 0.5 x 0.45 kg x (0.0508 m)^2, about 0.0006 kg m^2.
      .withMomentOfInertia(KilogramSquareMeters.of(0.0006))
      .withMotorInverted(false)
      .withZeroPower(MotorMode.COAST)
      .withStatorCurrentLimit(Amps.of(40))
      .withTelemetry("UpperFlyWheel", TelemetryVerbosity.HIGH);

  // The lower flywheel is the same, except that its motor faces the other way.
  private final SmartMotorControllerConfig lowerConfig = new SmartMotorControllerConfig(this)
      .withControlMode(ControlMode.CLOSED_LOOP)
      .withClosedLoopController(0.1, 0, 0)
      .withSimClosedLoopController(0.1, 0, 0)
      .withFeedforward(new SimpleMotorFeedforward(0.1, 0.12, 0))
      .withSimFeedforward(new SimpleMotorFeedforward(0.1, 0.12, 0))
      .withGearing(new MechanismGearing(GearBox.fromReductionStages(1)))
      .withMomentOfInertia(KilogramSquareMeters.of(0.0006))
      .withMotorInverted(true)
      .withZeroPower(MotorMode.COAST)
      .withStatorCurrentLimit(Amps.of(40))
      .withTelemetry("LowerFlyWheel", TelemetryVerbosity.HIGH);

  // The vendor motor controller objects: two TalonFX, with CAN IDs 1 and 2.
  private final TalonFX upperTalon = new TalonFX(1, new CANBus(CANPort.CAN_S0));
  private final TalonFX lowerTalon = new TalonFX(2, new CANBus(CANPort.CAN_S0));

  // One SmartMotorController for each flywheel, each driving one Kraken X60.
  private final SmartMotorController upperMotor = new TalonFXWrapper(upperTalon, DCMotor.getKrakenX60(1), upperConfig);
  private final SmartMotorController lowerMotor = new TalonFXWrapper(lowerTalon, DCMotor.getKrakenX60(1), lowerConfig);

  public DoubleFlywheelSubsystem() {
    // A mechanism class would do this for us. Without one, set up the telemetry
    // (and live tuning) of each motor controller ourselves.
    upperMotor.setupTelemetry();
    lowerMotor.setupTelemetry();
  }

  /** Spin both flywheels. One command sets both speeds, so the two always change together. */
  public Command setFlywheelSpeeds(AngularVelocity upperSpeed, AngularVelocity lowerSpeed) {
    return run(() -> {
      upperMotor.setVelocity(upperSpeed);
      lowerMotor.setVelocity(lowerSpeed);
    }).withName("Set FlyWheel Speeds");
  }

  /** Both flywheels at the same speed: no spin. */
  public Command neutralSpinShot() {
    return setFlywheelSpeeds(RPM.of(3000), RPM.of(3000));
  }

  /** The lower flywheel faster: backspin. */
  public Command backspinShot() {
    return setFlywheelSpeeds(RPM.of(2500), RPM.of(3500));
  }

  /** The upper flywheel faster: topspin. */
  public Command topspinShot() {
    return setFlywheelSpeeds(RPM.of(3500), RPM.of(2500));
  }

  /** Let both flywheels coast to a stop. */
  public Command stopFlywheels() {
    return runOnce(() -> {
      // A duty cycle of 0 lets the wheels coast down. A 0 RPM setpoint would brake them hard.
      upperMotor.setDutyCycle(0);
      lowerMotor.setDutyCycle(0);
    }).withName("Stop FlyWheels");
  }

  /** A Trigger that is true while both flywheels are within the tolerance of their target speeds. */
  public Trigger atTargetVelocity(AngularVelocity upperTarget, AngularVelocity lowerTarget, AngularVelocity tolerance) {
    return new Trigger(() -> upperMotor.getMechanismVelocity().isNear(upperTarget, tolerance)
        &#x26;&#x26; lowerMotor.getMechanismVelocity().isNear(lowerTarget, tolerance));
  }

<strong>  /**
</strong><strong>   * Spin both flywheels around a base speed, with a spin ratio between them.
</strong><strong>   *
</strong><strong>   * @param baseSpeed The average speed of the two flywheels.
</strong><strong>   * @param spinRatio From -1 (the most topspin) to 1 (the most backspin). 0 is no spin.
</strong><strong>   */
</strong><strong>  public Command setSpinShot(AngularVelocity baseSpeed, double spinRatio) {
</strong><strong>    double ratio = Math.max(-1.0, Math.min(1.0, spinRatio));
</strong><strong>    // At a ratio of 1 the lower flywheel runs 25% faster and the upper one 25% slower.
</strong><strong>    double upperMultiplier = 1.0 - ratio * 0.25;
</strong><strong>    double lowerMultiplier = 1.0 + ratio * 0.25;
</strong><strong>    return setFlywheelSpeeds(baseSpeed.times(upperMultiplier), baseSpeed.times(lowerMultiplier));
</strong><strong>  }
</strong>
  @Override
  public void periodic() {
    // Publishes each flywheel's telemetry, and applies anything changed through live tuning.
    upperMotor.updateTelemetry();
    lowerMotor.updateTelemetry();
  }

  @Override
  public void simulationPeriodic() {
    // Runs each flywheel's physics simulation.
    upperMotor.simIterate();
    lowerMotor.simIterate();
  }
}
</code></pre>
{% endtab %}

{% tab title="Java, Commands v3" %}
The highlighted lines are new in this step.

<pre class="language-java" data-title="mechanisms/DoubleFlywheelMechanism.java" data-line-numbers><code class="lang-java">package first.robot.mechanisms;

import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.KilogramSquareMeters;
import static org.wpilib.units.Units.RPM;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.TalonFX;
import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;
import org.wpilib.command3.Trigger;
import org.wpilib.hardware.bus.CANPort;
import org.wpilib.math.controller.SimpleMotorFeedforward;
import org.wpilib.math.system.DCMotor;
import org.wpilib.units.measure.AngularVelocity;
import yams.commands3.config.SmartMotorControllerConfig;
import yams.core.gearing.GearBox;
import yams.core.gearing.MechanismGearing;
import yams.core.motorcontrollers.SmartMotorController;
import yams.core.motorcontrollers.enums.ControlMode;
import yams.core.motorcontrollers.enums.MotorMode;
import yams.core.motorcontrollers.remote.TalonFXWrapper;
import yams.core.telemetry.enums.TelemetryVerbosity;

public class DoubleFlywheelMechanism implements Mechanism {

  // Each flywheel gets its own config, so that each can be given its own speed.
  private final SmartMotorControllerConfig upperConfig = new SmartMotorControllerConfig(this)
      .withControlMode(ControlMode.CLOSED_LOOP)
      // Feedback and feedforward constants, on the real robot and in simulation.
      .withClosedLoopController(0.1, 0, 0)
      .withSimClosedLoopController(0.1, 0, 0)
      .withFeedforward(new SimpleMotorFeedforward(0.1, 0.12, 0))
      .withSimFeedforward(new SimpleMotorFeedforward(0.1, 0.12, 0))
      // Gearing from the motor rotor to the flywheel: 1:1, the wheel is on the motor's shaft.
      .withGearing(new MechanismGearing(GearBox.fromReductionStages(1)))
      // There is no FlyWheel mechanism, so the simulation gets the wheel's moment of
      // inertia from here: a 4 inch, 1 lb (0.45 kg) wheel as a solid disc is
      // 0.5 x 0.45 kg x (0.0508 m)^2, about 0.0006 kg m^2.
      .withMomentOfInertia(KilogramSquareMeters.of(0.0006))
      .withMotorInverted(false)
      .withZeroPower(MotorMode.COAST)
      .withStatorCurrentLimit(Amps.of(40))
      .withTelemetry("UpperFlyWheel", TelemetryVerbosity.HIGH);

  // The lower flywheel is the same, except that its motor faces the other way.
  private final SmartMotorControllerConfig lowerConfig = new SmartMotorControllerConfig(this)
      .withControlMode(ControlMode.CLOSED_LOOP)
      .withClosedLoopController(0.1, 0, 0)
      .withSimClosedLoopController(0.1, 0, 0)
      .withFeedforward(new SimpleMotorFeedforward(0.1, 0.12, 0))
      .withSimFeedforward(new SimpleMotorFeedforward(0.1, 0.12, 0))
      .withGearing(new MechanismGearing(GearBox.fromReductionStages(1)))
      .withMomentOfInertia(KilogramSquareMeters.of(0.0006))
      .withMotorInverted(true)
      .withZeroPower(MotorMode.COAST)
      .withStatorCurrentLimit(Amps.of(40))
      .withTelemetry("LowerFlyWheel", TelemetryVerbosity.HIGH);

  // The vendor motor controller objects: two TalonFX, with CAN IDs 1 and 2.
  private final TalonFX upperTalon = new TalonFX(1, new CANBus(CANPort.CAN_S0));
  private final TalonFX lowerTalon = new TalonFX(2, new CANBus(CANPort.CAN_S0));

  // One SmartMotorController for each flywheel, each driving one Kraken X60.
  private final SmartMotorController upperMotor = new TalonFXWrapper(upperTalon, DCMotor.getKrakenX60(1), upperConfig);
  private final SmartMotorController lowerMotor = new TalonFXWrapper(lowerTalon, DCMotor.getKrakenX60(1), lowerConfig);

  public DoubleFlywheelMechanism() {
    // A mechanism class would do this for us. Without one, set up the telemetry
    // (and live tuning) of each motor controller ourselves.
    upperMotor.setupTelemetry();
    lowerMotor.setupTelemetry();
  }

  /** Spin both flywheels. One command sets both speeds, so the two always change together. */
  public Command setFlywheelSpeeds(AngularVelocity upperSpeed, AngularVelocity lowerSpeed) {
    return run(coroutine -> {
      while (true) {
        upperMotor.setVelocity(upperSpeed);
        lowerMotor.setVelocity(lowerSpeed);
        coroutine.yield();
      }
    }).named("Set FlyWheel Speeds");
  }

  /** Both flywheels at the same speed: no spin. */
  public Command neutralSpinShot() {
    return setFlywheelSpeeds(RPM.of(3000), RPM.of(3000));
  }

  /** The lower flywheel faster: backspin. */
  public Command backspinShot() {
    return setFlywheelSpeeds(RPM.of(2500), RPM.of(3500));
  }

  /** The upper flywheel faster: topspin. */
  public Command topspinShot() {
    return setFlywheelSpeeds(RPM.of(3500), RPM.of(2500));
  }

  /** Let both flywheels coast to a stop. */
  public Command stopFlywheels() {
    return run(coroutine -> {
      // A duty cycle of 0 lets the wheels coast down. A 0 RPM setpoint would brake them hard.
      upperMotor.setDutyCycle(0);
      lowerMotor.setDutyCycle(0);
    }).named("Stop FlyWheels");
  }

  /** A Trigger that is true while both flywheels are within the tolerance of their target speeds. */
  public Trigger atTargetVelocity(AngularVelocity upperTarget, AngularVelocity lowerTarget, AngularVelocity tolerance) {
    return new Trigger(() -> upperMotor.getMechanismVelocity().isNear(upperTarget, tolerance)
        &#x26;&#x26; lowerMotor.getMechanismVelocity().isNear(lowerTarget, tolerance));
  }

<strong>  /**
</strong><strong>   * Spin both flywheels around a base speed, with a spin ratio between them.
</strong><strong>   *
</strong><strong>   * @param baseSpeed The average speed of the two flywheels.
</strong><strong>   * @param spinRatio From -1 (the most topspin) to 1 (the most backspin). 0 is no spin.
</strong><strong>   */
</strong><strong>  public Command setSpinShot(AngularVelocity baseSpeed, double spinRatio) {
</strong><strong>    double ratio = Math.max(-1.0, Math.min(1.0, spinRatio));
</strong><strong>    // At a ratio of 1 the lower flywheel runs 25% faster and the upper one 25% slower.
</strong><strong>    double upperMultiplier = 1.0 - ratio * 0.25;
</strong><strong>    double lowerMultiplier = 1.0 + ratio * 0.25;
</strong><strong>    return setFlywheelSpeeds(baseSpeed.times(upperMultiplier), baseSpeed.times(lowerMultiplier));
</strong><strong>  }
</strong>
  /** Call this from Robot.robotPeriodic(), before the scheduler runs. */
  public void periodic() {
    // Publishes each flywheel's telemetry, and applies anything changed through live tuning.
    upperMotor.updateTelemetry();
    lowerMotor.updateTelemetry();
  }

  /** Robot.simulationInit() runs this in its own Notifier, every robot loop. */
  public void simulationPeriodic() {
    // Runs each flywheel's physics simulation.
    upperMotor.simIterate();
    lowerMotor.simIterate();
  }
}
</code></pre>
{% endtab %}

{% tab title="C++" %}
The highlighted lines are new in this step.

<pre class="language-cpp" data-title="include/subsystems/DoubleFlywheelSubsystem.hpp" data-line-numbers><code class="lang-cpp">#pragma once

#include &#x3C;optional>

#include &#x3C;ctre/phoenix6/TalonFX.hpp>
#include &#x3C;wpi/commands2/CommandPtr.hpp>
#include &#x3C;wpi/commands2/SubsystemBase.hpp>
#include &#x3C;wpi/commands2/button/Trigger.hpp>
#include &#x3C;wpi/units/angular_velocity.hpp>

#include "yams/motorcontrollers/SmartMotorControllerConfig.hpp"
#include "yams/motorcontrollers/remote/TalonFXWrapper.hpp"

class DoubleFlywheelSubsystem : public wpi::cmd::SubsystemBase {
 public:
  DoubleFlywheelSubsystem();

  /** Spin both flywheels. One command sets both speeds, so the two always change together. */
  wpi::cmd::CommandPtr SetFlywheelSpeeds(wpi::units::revolutions_per_minute_t upperSpeed,
                                         wpi::units::revolutions_per_minute_t lowerSpeed);

  /** Both flywheels at the same speed: no spin. */
  wpi::cmd::CommandPtr NeutralSpinShot();

  /** The lower flywheel faster: backspin. */
  wpi::cmd::CommandPtr BackspinShot();

  /** The upper flywheel faster: topspin. */
  wpi::cmd::CommandPtr TopspinShot();

  /** Let both flywheels coast to a stop. */
  wpi::cmd::CommandPtr StopFlywheels();

  /** A Trigger that is true while both flywheels are within the tolerance of their target speeds. */
  wpi::cmd::Trigger AtTargetVelocity(wpi::units::revolutions_per_minute_t upperTarget,
                                     wpi::units::revolutions_per_minute_t lowerTarget,
                                     wpi::units::revolutions_per_minute_t tolerance);

<strong>  /**
</strong><strong>   * Spin both flywheels around a base speed, with a spin ratio between them.
</strong><strong>   *
</strong><strong>   * @param baseSpeed The average speed of the two flywheels.
</strong><strong>   * @param spinRatio From -1 (the most topspin) to 1 (the most backspin). 0 is no spin.
</strong><strong>   */
</strong><strong>  wpi::cmd::CommandPtr SetSpinShot(wpi::units::revolutions_per_minute_t baseSpeed, double spinRatio);
</strong>
  void Periodic() override;
  void SimulationPeriodic() override;

 private:
  // The vendor motor controller objects: two TalonFX, with CAN IDs 1 and 2.
  ctre::phoenix6::hardware::TalonFX m_upperTalon{1, ctre::phoenix6::CANBus{}};
  ctre::phoenix6::hardware::TalonFX m_lowerTalon{2, ctre::phoenix6::CANBus{}};

  // Each flywheel gets its own config, so that each can be given its own speed.
  yams::motorcontrollers::SmartMotorControllerConfig m_upperConfig;
  yams::motorcontrollers::SmartMotorControllerConfig m_lowerConfig;

  // One SmartMotorController for each flywheel, created in the constructor.
  std::optional&#x3C;yams::motorcontrollers::remote::TalonFXWrapper> m_upperMotor;
  std::optional&#x3C;yams::motorcontrollers::remote::TalonFXWrapper> m_lowerMotor;
};
</code></pre>

<pre class="language-cpp" data-title="cpp/subsystems/DoubleFlywheelSubsystem.cpp" data-line-numbers><code class="lang-cpp">#include "subsystems/DoubleFlywheelSubsystem.hpp"

<strong>#include &#x3C;algorithm>
</strong>#include &#x3C;wpi/math/controller/SimpleMotorFeedforward.hpp>
#include &#x3C;wpi/math/system/DCMotor.hpp>
#include &#x3C;wpi/units/angle.hpp>
#include &#x3C;wpi/units/current.hpp>
#include &#x3C;wpi/units/moment_of_inertia.hpp>
#include &#x3C;wpi/units/math.hpp>
#include &#x3C;wpi/units/voltage.hpp>

#include "yams/gearing/GearBox.hpp"
#include "yams/gearing/MechanismGearing.hpp"

using namespace yams::gearing;
using Cfg = yams::motorcontrollers::SmartMotorControllerConfig;
using Feedforward = wpi::math::SimpleMotorFeedforward&#x3C;wpi::units::turns>;

DoubleFlywheelSubsystem::DoubleFlywheelSubsystem() {
  // The feedforward constants of both flywheels: kS and kV.
  const Feedforward feedforward{0.1_V, wpi::units::unit_t&#x3C;Feedforward::kv_unit>{0.12}};

  m_upperConfig
      .WithSubsystem(this)
      .WithClosedLoopMode()
      // Feedback and feedforward constants, on the real robot and in simulation.
      .WithFeedback(0.1, 0, 0)
      .WithSimClosedLoopController(0.1, 0, 0)
      .WithFeedforward(feedforward)
      .WithSimFeedforward(feedforward)
      // Gearing from the motor rotor to the flywheel: 1:1, the wheel is on the motor's shaft.
      .WithMotorGearing(MechanismGearing{GearBox::FromReductionStages({1.0})})
      // There is no FlyWheel mechanism, so the simulation gets the wheel's moment of
      // inertia from here: a 4 inch, 1 lb (0.45 kg) wheel as a solid disc is
      // 0.5 x 0.45 kg x (0.0508 m)^2, about 0.0006 kg m^2.
      .WithMOI(wpi::units::kilogram_square_meter_t{0.0006})
      .WithMotorInverted(false)
      .WithZeroPower(Cfg::MotorMode::COAST)
      .WithStatorCurrentLimit(40_A)
      .WithTelemetry("UpperFlyWheel", Cfg::TelemetryVerbosity::HIGH);

  // The lower flywheel is the same, except that its motor faces the other way.
  m_lowerConfig
      .WithSubsystem(this)
      .WithClosedLoopMode()
      .WithFeedback(0.1, 0, 0)
      .WithSimClosedLoopController(0.1, 0, 0)
      .WithFeedforward(feedforward)
      .WithSimFeedforward(feedforward)
      .WithMotorGearing(MechanismGearing{GearBox::FromReductionStages({1.0})})
      .WithMOI(wpi::units::kilogram_square_meter_t{0.0006})
      .WithMotorInverted(true)
      .WithZeroPower(Cfg::MotorMode::COAST)
      .WithStatorCurrentLimit(40_A)
      .WithTelemetry("LowerFlyWheel", Cfg::TelemetryVerbosity::HIGH);

  // One SmartMotorController for each flywheel, each driving one Kraken X60.
  m_upperMotor.emplace(&#x26;m_upperTalon, wpi::math::DCMotor::KrakenX60(1), &#x26;m_upperConfig);
  m_lowerMotor.emplace(&#x26;m_lowerTalon, wpi::math::DCMotor::KrakenX60(1), &#x26;m_lowerConfig);

  // A mechanism class would do this for us. Without one, set up the telemetry
  // (and live tuning) of each motor controller ourselves.
  m_upperMotor->SetupTelemetry();
  m_lowerMotor->SetupTelemetry();
}

wpi::cmd::CommandPtr DoubleFlywheelSubsystem::SetFlywheelSpeeds(
    wpi::units::revolutions_per_minute_t upperSpeed, wpi::units::revolutions_per_minute_t lowerSpeed) {
  return Run([this, upperSpeed, lowerSpeed] {
           m_upperMotor->SetVelocity(wpi::units::turns_per_second_t{upperSpeed});
           m_lowerMotor->SetVelocity(wpi::units::turns_per_second_t{lowerSpeed});
         })
      .WithName("Set FlyWheel Speeds");
}

wpi::cmd::CommandPtr DoubleFlywheelSubsystem::NeutralSpinShot() {
  return SetFlywheelSpeeds(3000_rpm, 3000_rpm);
}

wpi::cmd::CommandPtr DoubleFlywheelSubsystem::BackspinShot() {
  return SetFlywheelSpeeds(2500_rpm, 3500_rpm);
}

wpi::cmd::CommandPtr DoubleFlywheelSubsystem::TopspinShot() {
  return SetFlywheelSpeeds(3500_rpm, 2500_rpm);
}

wpi::cmd::CommandPtr DoubleFlywheelSubsystem::StopFlywheels() {
  return RunOnce([this] {
    // A duty cycle of 0 lets the wheels coast down. A 0 RPM setpoint would brake them hard.
    m_upperMotor->SetDutyCycle(0);
    m_lowerMotor->SetDutyCycle(0);
  }).WithName("Stop FlyWheels");
}

wpi::cmd::Trigger DoubleFlywheelSubsystem::AtTargetVelocity(wpi::units::revolutions_per_minute_t upperTarget,
                                                            wpi::units::revolutions_per_minute_t lowerTarget,
                                                            wpi::units::revolutions_per_minute_t tolerance) {
  return wpi::cmd::Trigger([this, upperTarget, lowerTarget, tolerance] {
    return wpi::units::math::abs(m_upperMotor->GetMechanismVelocity() - upperTarget) &#x3C;= tolerance &#x26;&#x26;
           wpi::units::math::abs(m_lowerMotor->GetMechanismVelocity() - lowerTarget) &#x3C;= tolerance;
  });
}

<strong>wpi::cmd::CommandPtr DoubleFlywheelSubsystem::SetSpinShot(wpi::units::revolutions_per_minute_t baseSpeed,
</strong><strong>                                                          double spinRatio) {
</strong><strong>  double ratio = std::clamp(spinRatio, -1.0, 1.0);
</strong><strong>  // At a ratio of 1 the lower flywheel runs 25% faster and the upper one 25% slower.
</strong><strong>  double upperMultiplier = 1.0 - ratio * 0.25;
</strong><strong>  double lowerMultiplier = 1.0 + ratio * 0.25;
</strong><strong>  return SetFlywheelSpeeds(baseSpeed * upperMultiplier, baseSpeed * lowerMultiplier);
</strong><strong>}
</strong>
void DoubleFlywheelSubsystem::Periodic() {
  // Publishes each flywheel's telemetry, and applies anything changed through live tuning.
  m_upperMotor->UpdateTelemetry();
  m_lowerMotor->UpdateTelemetry();
}

void DoubleFlywheelSubsystem::SimulationPeriodic() {
  // Runs each flywheel's physics simulation.
  m_upperMotor->SimIterate();
  m_lowerMotor->SimIterate();
}
</code></pre>
{% endtab %}
{% endtabs %}

## Two motors on one wheel

So far each wheel has one motor. If a wheel is driven by two motors on the same shaft or gearbox, the second motor must always turn with the first. Make it a **follower**: it copies the leader's output, and you do not give it a config or a `SmartMotorController` of its own.

{% tabs %}
{% tab title="Java, Commands v2" %}
```java
private final SmartMotorControllerConfig upperConfig = new SmartMotorControllerConfig(this)
    // ... config options from step 1
    // A second Kraken X60 on the upper wheel's shaft, CAN ID 3. true: it faces the other way.
    .withFollowers(Pair.of(new TalonFX(3, new CANBus(CANPort.CAN_S0)), true));
```
{% endtab %}

{% tab title="Java, Commands v3" %}
The same as Commands v2, with `SmartMotorControllerConfig` from `yams.commands3.config`.

```java
private final SmartMotorControllerConfig upperConfig = new SmartMotorControllerConfig(this)
    // ... config options from step 1
    // A second Kraken X60 on the upper wheel's shaft, CAN ID 3. true: it faces the other way.
    .withFollowers(Pair.of(new TalonFX(3, new CANBus(CANPort.CAN_S0)), true));
```
{% endtab %}

{% tab title="C++" %}
Keep the follower's `TalonFX` as a member, next to the leader's, and name it in the leader's config:

```cpp
// In the header: a second Kraken X60 on the upper wheel's shaft, CAN ID 3.
ctre::phoenix6::hardware::TalonFX m_upperFollowerTalon{3, ctre::phoenix6::CANBus{}};

// In the constructor. true: it faces the other way.
m_upperConfig
    // ... config options from step 1
    .WithFollowers({{&m_upperFollowerTalon, true}});
```
{% endtab %}
{% endtabs %}

`Pair` is `org.wpilib.util.Pair`. Change the `DCMotor` in that wheel's `SmartMotorController` to two motors, `DCMotor.getKrakenX60(2)`, so the simulation knows. The follower must be the same vendor as the leader. [Followers](../../reference/editor/config/followers.md#withfollowers) has the details.

## Both wheels at the same speed

If you want both wheels to always get the **same** setpoint, you do not need to command each one yourself. Make one a **loosely coupled follower** of the other, and command only the leader. Unlike a follower above, it keeps its own config and runs its own closed loop, because it is on a different shaft:

{% tabs %}
{% tab title="Java, Commands v2" %}
```java
SmartMotorControllerConfig lowerConfig = new SmartMotorControllerConfig(this)
    // ... config options
    ;

SmartMotorController lowerMotor = new TalonFXWrapper(lowerTalon, DCMotor.getKrakenX60(1), lowerConfig);

SmartMotorControllerConfig upperConfig = new SmartMotorControllerConfig(this)
    // ... config options
    .withLooselyCoupledFollowers(lowerMotor); // the lower motor is given the upper motor's setpoints

SmartMotorController upperMotor = new TalonFXWrapper(upperTalon, DCMotor.getKrakenX60(1), upperConfig);
```
{% endtab %}

{% tab title="Java, Commands v3" %}
The same as Commands v2, with `SmartMotorControllerConfig` from `yams.commands3.config`.

```java
SmartMotorControllerConfig lowerConfig = new SmartMotorControllerConfig(this)
    // ... config options
    ;

SmartMotorController lowerMotor = new TalonFXWrapper(lowerTalon, DCMotor.getKrakenX60(1), lowerConfig);

SmartMotorControllerConfig upperConfig = new SmartMotorControllerConfig(this)
    // ... config options
    .withLooselyCoupledFollowers(lowerMotor); // the lower motor is given the upper motor's setpoints

SmartMotorController upperMotor = new TalonFXWrapper(upperTalon, DCMotor.getKrakenX60(1), upperConfig);
```
{% endtab %}

{% tab title="C++" %}
In the constructor, create the lower motor first, then name it in the upper motor's config:

```cpp
m_lowerMotor.emplace(&m_lowerTalon, wpi::math::DCMotor::KrakenX60(1), &m_lowerConfig);

m_upperConfig
    // ... config options
    .WithLooselyCoupledFollowers({&m_lowerMotor.value()});  // the lower motor is given the upper motor's setpoints

m_upperMotor.emplace(&m_upperTalon, wpi::math::DCMotor::KrakenX60(1), &m_upperConfig);
```
{% endtab %}
{% endtabs %}

{% hint style="warning" %}
Loosely coupled followers are only given velocity and position requests, **NOT** duty cycle or voltage requests. Command each `SmartMotorController` yourself when you need fully independent control, as the rest of this tutorial does.
{% endhint %}

[Followers](../../reference/editor/config/followers.md) explains the difference between loosely and tightly coupled followers.

## Summary

You now have a double flywheel shooter with:

* Independent velocity control of the upper and lower flywheels
* Preset shots for no spin, backspin and topspin, and a spin ratio for anything in between
* Simulation of both wheels, which you can graph
* Telemetry and live tuning for both flywheels

The differences from a single flywheel are:

1. Two `SmartMotorControllerConfig`s and two `SmartMotorController`s, controlled directly instead of through a `FlyWheel` mechanism
2. The subsystem calls `setupTelemetry()`, `updateTelemetry()` and `simIterate()` on each `SmartMotorController` itself
3. One command sets both velocity setpoints
