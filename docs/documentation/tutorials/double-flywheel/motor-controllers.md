---
description: One config and one SmartMotorController for each flywheel.
icon: microchip
---

# Step 1: Two motor controllers

{% hint style="info" %}
This page assumes a project with YAMS and Phoenix 6 installed. If you do not have one yet, start at [Create a project and install YAMS](../project-setup/).
{% endhint %}

Each flywheel gets its own `SmartMotorControllerConfig` and its own `SmartMotorController`, so that each can be given its own speed. The two configs are the same except for the telemetry name and the direction of the motor.

There is no `FlyWheel` mechanism in this tutorial, so three jobs a mechanism class would do fall to the subsystem:

* **Describe the wheel.** `withMomentOfInertia(...)` on the config tells the simulation how hard the wheel is to spin up. Without it the simulated wheel is far too heavy.
* **Set up the telemetry.** `setupTelemetry()` on each `SmartMotorController`, once, in the constructor. This is also what makes live tuning work.
* **Update every loop.** `updateTelemetry()` on each, and `simIterate()` on each in simulation.

<figure><img src="../../.gitbook/assets/smc-followers.png" alt="Two panels. Tightly coupled, withFollowers: a leader and a follower are on one shaft or gearbox and the follower copies the leader&#x27;s output. The motors are mechanically locked, there is one closed loop and one config, the followers are the same vendor as the leader, and voltage and duty cycle reach every motor. Loosely coupled, withLooselyCoupledFollowers: two separate motors joined only by the mechanism get the same setpoint and each runs its own PID. They are not mechanically locked, each has its own closed loop and config, vendors can be mixed, and voltage and duty cycle are not forwarded, so the leader moves alone."><figcaption><p>Tightly coupled followers share a shaft. Loosely coupled followers are separate motors.</p></figcaption></figure>

The two wheels here are separate motors on separate shafts, each given its own setpoint. If two motors drive the same wheel, see [step 3](spin-and-followers.md#two-motors-on-one-wheel).

{% hint style="success" %}
**In a hurry?** Jump to [the complete file](motor-controllers.md#the-complete-file), copy it, and change the values called out under **For your robot**. The numbered parts explain each piece in a sentence or two. You do not need to understand all of it to get moving: the pages under **Understanding** go deeper when you want them.
{% endhint %}

Create `DoubleFlywheelSubsystem` in your `subsystems` folder, or `DoubleFlywheelMechanism` in your `mechanisms` folder for Commands v3. In C++ that is a header in `src/main/include/subsystems` and a source file in `src/main/cpp/subsystems`.

{% stepper %}
{% step %}
### The upper flywheel's config

One `SmartMotorControllerConfig` says what the upper flywheel's motor controller should do: closed loop control, its gains, its gearing, and the size and weight of the wheel for the simulation.

{% tabs %}
{% tab title="Java, Commands v2" %}
{% code title="subsystems/DoubleFlywheelSubsystem.java" %}
```java
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
```
{% endcode %}
{% endtab %}

{% tab title="Java, Commands v3" %}
{% code title="mechanisms/DoubleFlywheelMechanism.java" %}
```java
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
```
{% endcode %}
{% endtab %}

{% tab title="C++" %}
{% code title="include/subsystems/DoubleFlywheelSubsystem.hpp" %}
```cpp
// Each flywheel gets its own config, so that each can be given its own speed.
yams::motorcontrollers::SmartMotorControllerConfig m_upperConfig;
yams::motorcontrollers::SmartMotorControllerConfig m_lowerConfig;
```
{% endcode %}

{% code title="cpp/subsystems/DoubleFlywheelSubsystem.cpp" %}
```cpp
// The feedforward constants of both flywheels: kS and kV.
const Feedforward feedforward{0.1_V, wpi::units::unit_t<Feedforward::kv_unit>{0.12}};

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
```
{% endcode %}
{% endtab %}
{% endtabs %}

The gains are **placeholders** worked out from the motor. A Kraken X60 turns at most about 6000 RPM, 100 rotations per second, on 12 volts. With no gearbox the wheel does too, so `kV` is 12 / 100 = `0.12` volts for each rotation per second. `kS` of `0.1` is a guess at the voltage friction takes. The [Shooter/FlyWheels](../shooter-flywheels/motor-controller.md) tutorial has a different `kV` (0.19) because it uses a different motor (a NEO, about 5676 RPM) through 1.5:1 gearing: its wheel tops out near 63 rotations per second, and 12 / 63 = 0.19.

**What to expect with them:** in simulation each wheel reaches its preset in well under a second and holds it close to the setpoint. Tune them for your robot with [the shooter tutorial's tune step](../shooter-flywheels/tune.md).

`withMomentOfInertia` takes the wheel's moment of inertia in kg m². For a solid wheel it is 0.5 x mass x radius squared: 0.5 x 0.45 kg x (0.0508 m)² for a 4 inch, 1 pound wheel. (There is also a `withMomentOfInertia(length, mass)` overload, but it treats the length as an arm swinging from one end, 1/3 x mass x length², so it does not fit a wheel.)

**For your robot:** Set your own gearing (top speed = motor free speed / reduction, and keep your fastest shot below it), and your wheel's moment of inertia from its mass and radius. Work out a starting `kV` as 12 divided by the wheel's top speed in rotations per second.

Want the details? [SmartMotorControllerConfig](../../reference/editor/config/)
{% endstep %}

{% step %}
### The lower flywheel's config

The same again for the lower flywheel, with its own telemetry name and the motor inverted.

{% tabs %}
{% tab title="Java, Commands v2" %}
{% code title="subsystems/DoubleFlywheelSubsystem.java" %}
```java
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
```
{% endcode %}
{% endtab %}

{% tab title="Java, Commands v3" %}
{% code title="mechanisms/DoubleFlywheelMechanism.java" %}
```java
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
```
{% endcode %}
{% endtab %}

{% tab title="C++" %}
{% code title="cpp/subsystems/DoubleFlywheelSubsystem.cpp" %}
```cpp
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
```
{% endcode %}
{% endtab %}
{% endtabs %}

**For your robot:** Flip the inversion of whichever wheel spins the wrong way. Both wheels should throw the game piece out when given a positive speed.
{% endstep %}

{% step %}
### The motor controller objects

These are the vendor's own objects, exactly as you would create them without YAMS.

{% tabs %}
{% tab title="Java, Commands v2" %}
{% code title="subsystems/DoubleFlywheelSubsystem.java" %}
```java
// The vendor motor controller objects: two TalonFX, with CAN IDs 1 and 2.
private final TalonFX upperTalon = new TalonFX(1, new CANBus(CANPort.CAN_S0));
private final TalonFX lowerTalon = new TalonFX(2, new CANBus(CANPort.CAN_S0));
```
{% endcode %}
{% endtab %}

{% tab title="Java, Commands v3" %}
{% code title="mechanisms/DoubleFlywheelMechanism.java" %}
```java
// The vendor motor controller objects: two TalonFX, with CAN IDs 1 and 2.
private final TalonFX upperTalon = new TalonFX(1, new CANBus(CANPort.CAN_S0));
private final TalonFX lowerTalon = new TalonFX(2, new CANBus(CANPort.CAN_S0));
```
{% endcode %}
{% endtab %}

{% tab title="C++" %}
{% code title="include/subsystems/DoubleFlywheelSubsystem.hpp" %}
```cpp
// The vendor motor controller objects: two TalonFX, with CAN IDs 1 and 2.
ctre::phoenix6::hardware::TalonFX m_upperTalon{1, ctre::phoenix6::CANBus{}};
ctre::phoenix6::hardware::TalonFX m_lowerTalon{2, ctre::phoenix6::CANBus{}};
```
{% endcode %}
{% endtab %}
{% endtabs %}

**For your robot:** Change the CAN IDs to yours.
{% endstep %}

{% step %}
### The SmartMotorControllers

One wrapper for each flywheel brings together its motor controller object, its motor and its config.

{% tabs %}
{% tab title="Java, Commands v2" %}
{% code title="subsystems/DoubleFlywheelSubsystem.java" %}
```java
// One SmartMotorController for each flywheel, each driving one Kraken X60.
private final SmartMotorController upperMotor = new TalonFXWrapper(upperTalon, DCMotor.getKrakenX60(1), upperConfig);
private final SmartMotorController lowerMotor = new TalonFXWrapper(lowerTalon, DCMotor.getKrakenX60(1), lowerConfig);
```
{% endcode %}
{% endtab %}

{% tab title="Java, Commands v3" %}
{% code title="mechanisms/DoubleFlywheelMechanism.java" %}
```java
// One SmartMotorController for each flywheel, each driving one Kraken X60.
private final SmartMotorController upperMotor = new TalonFXWrapper(upperTalon, DCMotor.getKrakenX60(1), upperConfig);
private final SmartMotorController lowerMotor = new TalonFXWrapper(lowerTalon, DCMotor.getKrakenX60(1), lowerConfig);
```
{% endcode %}
{% endtab %}

{% tab title="C++" %}
{% code title="include/subsystems/DoubleFlywheelSubsystem.hpp" %}
```cpp
// One SmartMotorController for each flywheel, created in the constructor.
std::optional<yams::motorcontrollers::remote::TalonFXWrapper> m_upperMotor;
std::optional<yams::motorcontrollers::remote::TalonFXWrapper> m_lowerMotor;
```
{% endcode %}

{% code title="cpp/subsystems/DoubleFlywheelSubsystem.cpp" %}
```cpp
// One SmartMotorController for each flywheel, each driving one Kraken X60.
m_upperMotor.emplace(&m_upperTalon, wpi::math::DCMotor::KrakenX60(1), &m_upperConfig);
m_lowerMotor.emplace(&m_lowerTalon, wpi::math::DCMotor::KrakenX60(1), &m_lowerConfig);
```
{% endcode %}
{% endtab %}
{% endtabs %}

**For your robot:** Change the motor to the kind and the number of motors on each wheel.

Want the details? [Smart motor controllers](../../reference/editor/)
{% endstep %}

{% step %}
### Set up the telemetry

A mechanism class would do this for you. Without one, call `setupTelemetry()` on each `SmartMotorController` once. This is also what makes live tuning work.

{% tabs %}
{% tab title="Java, Commands v2" %}
{% code title="subsystems/DoubleFlywheelSubsystem.java" %}
```java
public DoubleFlywheelSubsystem() {
  // A mechanism class would do this for us. Without one, set up the telemetry
  // (and live tuning) of each motor controller ourselves.
  upperMotor.setupTelemetry();
  lowerMotor.setupTelemetry();
}
```
{% endcode %}
{% endtab %}

{% tab title="Java, Commands v3" %}
{% code title="mechanisms/DoubleFlywheelMechanism.java" %}
```java
public DoubleFlywheelMechanism() {
  // A mechanism class would do this for us. Without one, set up the telemetry
  // (and live tuning) of each motor controller ourselves.
  upperMotor.setupTelemetry();
  lowerMotor.setupTelemetry();
}
```
{% endcode %}
{% endtab %}

{% tab title="C++" %}
{% code title="cpp/subsystems/DoubleFlywheelSubsystem.cpp" %}
```cpp
// A mechanism class would do this for us. Without one, set up the telemetry
// (and live tuning) of each motor controller ourselves.
m_upperMotor->SetupTelemetry();
m_lowerMotor->SetupTelemetry();
```
{% endcode %}
{% endtab %}
{% endtabs %}

Want the details? [Telemetry](../../understanding/telemetry.md)
{% endstep %}

{% step %}
### Update both every loop

`updateTelemetry()` on each wheel publishes its telemetry, and `simIterate()` on each runs its simulation.

In Commands v2 and C++ the scheduler calls a subsystem's periodic methods for you, so there is nothing more to do.

In Commands v3 nothing calls a `Mechanism` every loop, so `Robot.java` does it, the same way as in [the arm tutorial's step 2](../arm/mechanism.md#update-it-every-loop), which explains each part. Every mechanism gets one line in `robotPeriodic()`, before `scheduler.run()`, and one in `simulationInit()`.

If your `Robot.java` already has `simulationInit()` and `startSimulation(...)` from the arm tutorial, add the lines below. If not, copy the whole `Robot.java` from the Commands v3 tab of [the complete file](motor-controllers.md#the-complete-file).

{% tabs %}
{% tab title="Java, Commands v2" %}
{% code title="subsystems/DoubleFlywheelSubsystem.java" %}
```java
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
```
{% endcode %}
{% endtab %}

{% tab title="Java, Commands v3" %}
{% code title="mechanisms/DoubleFlywheelMechanism.java" %}
```java
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
```
{% endcode %}

{% code title="Robot.java" %}
```java
import first.robot.mechanisms.DoubleFlywheelMechanism;

// A field of Robot.
final DoubleFlywheelMechanism doubleFlywheel = new DoubleFlywheelMechanism();

// In robotPeriodic(), before scheduler.run():
doubleFlywheel.periodic();

// In simulationInit():
startSimulation("Double Flywheel", doubleFlywheel::simulationPeriodic);
```
{% endcode %}
{% endtab %}

{% tab title="C++" %}
{% code title="include/subsystems/DoubleFlywheelSubsystem.hpp" %}
```cpp
void Periodic() override;
void SimulationPeriodic() override;
```
{% endcode %}

{% code title="cpp/subsystems/DoubleFlywheelSubsystem.cpp" %}
```cpp
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
```
{% endcode %}
{% endtab %}
{% endtabs %}
{% endstep %}
{% endstepper %}

## The complete file

{% tabs %}
{% tab title="Java, Commands v2" %}
{% code title="subsystems/DoubleFlywheelSubsystem.java" lineNumbers="true" %}
```java
package first.robot.subsystems;

import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.KilogramSquareMeters;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.TalonFX;
import org.wpilib.command2.SubsystemBase;
import org.wpilib.hardware.bus.CANPort;
import org.wpilib.math.controller.SimpleMotorFeedforward;
import org.wpilib.math.system.DCMotor;
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
```
{% endcode %}
{% endtab %}

{% tab title="Java, Commands v3" %}
{% code title="mechanisms/DoubleFlywheelMechanism.java" lineNumbers="true" %}
```java
package first.robot.mechanisms;

import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.KilogramSquareMeters;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.TalonFX;
import org.wpilib.command3.Mechanism;
import org.wpilib.hardware.bus.CANPort;
import org.wpilib.math.controller.SimpleMotorFeedforward;
import org.wpilib.math.system.DCMotor;
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
```
{% endcode %}

<pre class="language-java" data-title="Robot.java" data-line-numbers><code class="lang-java">package first.robot;

<strong>import java.util.ArrayList;
</strong><strong>import java.util.List;
</strong>import org.wpilib.command3.Scheduler;
import org.wpilib.command3.button.CommandGamepad;
import org.wpilib.framework.OpModeRobot;
<strong>import org.wpilib.system.Notifier;
</strong><strong>import org.wpilib.telemetry.Telemetry;
</strong>import first.robot.constants.DriverConstants;
<strong>import first.robot.mechanisms.DoubleFlywheelMechanism;
</strong>import first.robot.mechanisms.ExampleMechanism;

public class Robot extends OpModeRobot {
  final ExampleMechanism exampleMechanism = new ExampleMechanism();
<strong>  final DoubleFlywheelMechanism doubleFlywheel = new DoubleFlywheelMechanism();
</strong>  final CommandGamepad exampleController =
      new CommandGamepad(DriverConstants.DRIVER_CONTROLLER_PORT);

<strong>  // The scheduler that runs every command.
</strong><strong>  private final Scheduler scheduler = Scheduler.getDefault();
</strong><strong>  // Each mechanism's simulation Notifier, kept so that it is not garbage collected.
</strong><strong>  private final List&#x3C;Notifier> simulationNotifiers = new ArrayList&#x3C;>();
</strong>
  public Robot() {}

  @Override
  public void robotPeriodic() {
<strong>    // Runs every loop, on the robot and in simulation.
</strong><strong>    // Update the mechanism before the scheduler runs its commands.
</strong><strong>    doubleFlywheel.periodic();
</strong><strong>    scheduler.run();
</strong><strong>    // Publish the scheduler, so that a dashboard shows which commands are running.
</strong><strong>    Telemetry.log("Scheduler", scheduler, Scheduler.proto);
</strong><strong>  }
</strong>
<strong>  @Override
</strong><strong>  public void simulationInit() {
</strong><strong>    // Runs once, when the simulation starts.
</strong><strong>    // Run each mechanism's simulation in its own Notifier, every robot loop.
</strong><strong>    startSimulation("Double Flywheel", doubleFlywheel::simulationPeriodic);
</strong><strong>  }
</strong>
<strong>  /** Runs a mechanism's simulation in its own Notifier, every robot loop period. */
</strong><strong>  private void startSimulation(String name, Runnable simulationPeriodic) {
</strong><strong>    Notifier notifier = new Notifier(simulationPeriodic);
</strong><strong>    notifier.setName(name + " Simulation");
</strong><strong>    notifier.startPeriodic(getPeriod());
</strong><strong>    simulationNotifiers.add(notifier);
</strong><strong>  }
</strong>}
</code></pre>
{% endtab %}

{% tab title="C++" %}
{% code title="include/subsystems/DoubleFlywheelSubsystem.hpp" lineNumbers="true" %}
```cpp
#pragma once

#include <optional>

#include <ctre/phoenix6/TalonFX.hpp>
#include <wpi/commands2/SubsystemBase.hpp>

#include "yams/motorcontrollers/SmartMotorControllerConfig.hpp"
#include "yams/motorcontrollers/remote/TalonFXWrapper.hpp"

class DoubleFlywheelSubsystem : public wpi::cmd::SubsystemBase {
 public:
  DoubleFlywheelSubsystem();

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
  std::optional<yams::motorcontrollers::remote::TalonFXWrapper> m_upperMotor;
  std::optional<yams::motorcontrollers::remote::TalonFXWrapper> m_lowerMotor;
};
```
{% endcode %}

{% code title="cpp/subsystems/DoubleFlywheelSubsystem.cpp" lineNumbers="true" %}
```cpp
#include "subsystems/DoubleFlywheelSubsystem.hpp"

#include <wpi/math/controller/SimpleMotorFeedforward.hpp>
#include <wpi/math/system/DCMotor.hpp>
#include <wpi/units/angle.hpp>
#include <wpi/units/current.hpp>
#include <wpi/units/moment_of_inertia.hpp>
#include <wpi/units/voltage.hpp>

#include "yams/gearing/GearBox.hpp"
#include "yams/gearing/MechanismGearing.hpp"

using namespace yams::gearing;
using Cfg = yams::motorcontrollers::SmartMotorControllerConfig;
using Feedforward = wpi::math::SimpleMotorFeedforward<wpi::units::turns>;

DoubleFlywheelSubsystem::DoubleFlywheelSubsystem() {
  // The feedforward constants of both flywheels: kS and kV.
  const Feedforward feedforward{0.1_V, wpi::units::unit_t<Feedforward::kv_unit>{0.12}};

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
  m_upperMotor.emplace(&m_upperTalon, wpi::math::DCMotor::KrakenX60(1), &m_upperConfig);
  m_lowerMotor.emplace(&m_lowerTalon, wpi::math::DCMotor::KrakenX60(1), &m_lowerConfig);

  // A mechanism class would do this for us. Without one, set up the telemetry
  // (and live tuning) of each motor controller ourselves.
  m_upperMotor->SetupTelemetry();
  m_lowerMotor->SetupTelemetry();
}

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
```
{% endcode %}
{% endtab %}
{% endtabs %}

## Read more

Every option used here has its own section in the `SmartMotorControllerConfig` pages:

* [Motor basics](../../reference/editor/config/motor-basics.md): control mode, inversion, what the motor does at zero power
* [Gearing and units](../../reference/editor/config/gearing-and-units.md): gearing, and how rotations become distance
* [Closed loop control](../../reference/editor/config/closed-loop-control.md), [Feedforward](../../reference/editor/config/feedforward.md) and [Motion profiles](../../reference/editor/config/motion-profiles.md)
* [Limits and protection](../../reference/editor/config/limits-and-protection.md): soft limits and current limits
* [Simulation](../../reference/editor/config/simulation.md): the simulation only gains

The PID and feedforward numbers on this page are a place to start, not tuned values. Tune them with [the shooter tutorial's tune step](../shooter-flywheels/tune.md), once for each wheel.

## Check it

Open the command palette with <kbd>Ctrl+Shift+P</kbd> and run **WPILib: Build Robot Code**. It should build. Nothing moves yet, because nothing gives the wheels a speed.

* **`TalonFX` or `TalonFXWrapper` cannot be found:** Phoenix 6 is not installed. See [Install your motor controller's library](../project-setup/#install-your-motor-controllers-library).
* **`SmartMotorControllerConfig` cannot be found:** YAMS is not installed, or the import is from the other command framework (`yams.commands2` or `yams.commands3`).
