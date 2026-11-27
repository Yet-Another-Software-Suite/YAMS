---
description: Configure the SparkMax and wrap it in a SmartMotorController.
icon: microchip
---

# Step 1: The motor controller

{% hint style="info" %}
This page assumes a project with YAMS and REVLib installed. If you do not have one yet, start at [Create a project and install YAMS](../project-setup/).
{% endhint %}

A `SmartMotorController` is how YAMS talks to a motor controller. It takes three things:

1. **The vendor's motor controller object**, here a `SparkMax`.
2. **The motor** that is plugged into it, here one NEO.
3. **A `SmartMotorControllerConfig`** that says what the motor controller should do: its control mode, its PID and feedforward gains, the gearing between the motor and the flywheel, and how heavy the wheel is for the simulation.

YAMS applies the config to the `SparkMax` for you, and uses the same config to build the simulation.

{% hint style="success" %}
**In a hurry?** Jump to [the complete file](motor-controller.md#the-complete-file), copy it, and change the values called out under **For your robot**. The numbered parts explain each piece in a sentence or two. You do not need to understand all of it to get moving: the pages under **Understanding** go deeper when you want them.
{% endhint %}

## Create the file

{% tabs %}
{% tab title="Java, Commands v2" %}
Create `ShooterSubsystem.java` in the `subsystems` folder, next to the template's `ExampleSubsystem.java`.
{% endtab %}

{% tab title="Java, Commands v3" %}
Create `ShooterMechanism.java` in the `mechanisms` folder, next to the template's `ExampleMechanism.java`.

The config comes from `yams.commands3.config`, and it takes the `Mechanism` where Commands v2 passes the subsystem.
{% endtab %}

{% tab title="C++" %}
Create `ShooterSubsystem.hpp` in `src/main/include/subsystems` and `ShooterSubsystem.cpp` in `src/main/cpp/subsystems`.

YAMS keeps a pointer to the config and to the `SparkMax`, so in C++ both are members of the subsystem. The constructor fills in the config and then creates the `SmartMotorController`, which is why that one is a `std::optional`.

`WithFeedback` is the C++ name of `withClosedLoopController`, and `WithMotorGearing` of `withGearing`.
{% endtab %}
{% endtabs %}

{% stepper %}
{% step %}
### Start the config

Everything the motor controller should do goes into one `SmartMotorControllerConfig`. Start it, and put it in closed loop mode so that it can hold a speed. A flywheel holds a speed and has no position to protect, so this config has no motion profile, no starting position and no soft limits. It coasts at zero power, so that the wheel is not braked to a stop every time a command ends.

{% tabs %}
{% tab title="Java, Commands v2" %}
{% code title="subsystems/ShooterSubsystem.java" %}
```java
// What the motor controller should do: its control mode, gains and gearing.
private final SmartMotorControllerConfig smcConfig = new SmartMotorControllerConfig(this)
    .withControlMode(ControlMode.CLOSED_LOOP)
```
{% endcode %}
{% endtab %}

{% tab title="Java, Commands v3" %}
{% code title="mechanisms/ShooterMechanism.java" %}
```java
// What the motor controller should do: its control mode, gains and gearing.
private final SmartMotorControllerConfig smcConfig = new SmartMotorControllerConfig(this)
    .withControlMode(ControlMode.CLOSED_LOOP)
```
{% endcode %}
{% endtab %}

{% tab title="C++" %}
{% code title="include/subsystems/ShooterSubsystem.hpp" %}
```cpp
// What the motor controller should do. It is filled in by the constructor.
yams::motorcontrollers::SmartMotorControllerConfig m_smcConfig;
```
{% endcode %}

{% code title="cpp/subsystems/ShooterSubsystem.cpp" %}
```cpp
// What the motor controller should do: its control mode, gains and gearing.
m_smcConfig
    .WithSubsystem(this)
    .WithClosedLoopMode()
```
{% endcode %}
{% endtab %}
{% endtabs %}

Want the details? [Motor basics](../../reference/editor/config/motor-basics.md)
{% endstep %}

{% step %}
### PID and feedforward

An arm or an elevator uses **position control**: you ask for an angle or a height, and the motor stops once it gets there. A flywheel uses **velocity control**: you ask for a speed, and the motor has to keep pushing the whole time to hold it. Most of that push is predictable, because a motor's speed is close to proportional to its voltage. So the feedforward gain `kV` (volts for each rotation per second) does most of the work, and the PID gain `kP` only fixes the small error left over, such as the drop when a game piece goes through. There is no motion profile: the wheel can spin up as fast as the motor allows.

The `Sim` options are used only in simulation, so the simulation can be tuned without touching the real robot's gains.

{% tabs %}
{% tab title="Java, Commands v2" %}
{% code title="subsystems/ShooterSubsystem.java" %}
```java
// Feedback constants (PID), on the real robot and in simulation.
// A flywheel holds a speed, so it gets no motion profile.
.withClosedLoopController(0.2, 0, 0)
.withSimClosedLoopController(0.2, 0, 0)
// Feedforward constants, on the real robot and in simulation.
.withFeedforward(new SimpleMotorFeedforward(0, 0.19, 0))
.withSimFeedforward(new SimpleMotorFeedforward(0, 0.19, 0))
```
{% endcode %}
{% endtab %}

{% tab title="Java, Commands v3" %}
{% code title="mechanisms/ShooterMechanism.java" %}
```java
// Feedback constants (PID), on the real robot and in simulation.
// A flywheel holds a speed, so it gets no motion profile.
.withClosedLoopController(0.2, 0, 0)
.withSimClosedLoopController(0.2, 0, 0)
// Feedforward constants, on the real robot and in simulation.
.withFeedforward(new SimpleMotorFeedforward(0, 0.19, 0))
.withSimFeedforward(new SimpleMotorFeedforward(0, 0.19, 0))
```
{% endcode %}
{% endtab %}

{% tab title="C++" %}
{% code title="cpp/subsystems/ShooterSubsystem.cpp" %}
```cpp
// Feedback constants (PID), on the real robot and in simulation.
// A flywheel holds a speed, so it gets no motion profile.
.WithFeedback(0.2, 0, 0)
.WithSimClosedLoopController(0.2, 0, 0)
// Feedforward constants (kS and kV), on the real robot and in simulation.
.WithFeedforward(Feedforward{0_V, wpi::units::unit_t<Feedforward::kv_unit>{0.19}})
.WithSimFeedforward(Feedforward{0_V, wpi::units::unit_t<Feedforward::kv_unit>{0.19}})
```
{% endcode %}
{% endtab %}
{% endtabs %}

These gains are **placeholders**, worked out from the motor rather than tuned. A NEO turns at most about 5676 RPM on 12 volts. Through the 1.5:1 gearing below, that is about 3784 RPM, or 63 rotations per second, at the wheel. 12 volts divided by 63 rotations per second is about `0.19` volts for each rotation per second, which is the `kV` here. `kP` of `0.2` adds 0.2 volts for each rotation per second (60 RPM) the wheel is too slow.

**What to expect with them:** in simulation the wheel reaches a 3000 RPM setpoint in well under a second and holds it within a few tens of RPM, inside the 50 RPM band you use in step 3. A real wheel has friction, so it settles a little below the setpoint until you tune it in [Step 5: Tune it](tune.md).

**For your robot:** Work out your own starting `kV` the same way: 12 divided by your wheel's top speed in rotations per second. Then tune both gains in step 5.

Want the details? [Closed loop control](../../reference/editor/config/closed-loop-control.md) and [Feedforward](../../reference/editor/config/feedforward.md)
{% endstep %}

{% step %}
### Gearing and the wheel's weight

The gearing tells YAMS how many turns of the motor make one turn of the flywheel. A shooter usually has little or no reduction, because it needs speed more than force: here one turn of the wheel is 1.5 turns of the motor.

**Top speed = motor free speed / reduction.** A NEO's free speed is about 5676 RPM, so 1.5:1 gives about 3784 RPM at the wheel. Keep your setpoints below that, with some room spare: this tutorial uses 2000 and 3000 RPM. A 12:1 gearbox would cap the same wheel at about 470 RPM.

The moment of inertia says how hard the wheel is to spin up, and only the simulation uses it. For a solid wheel it is 0.5 x mass x radius squared, in kilograms and meters. With it, the simulated wheel spins up in well under a second, like a real one. Leave it out and YAMS uses 0.02 kg m², more than twenty times this wheel, so the simulated wheel takes several seconds to reach speed.

{% tabs %}
{% tab title="Java, Commands v2" %}
{% code title="subsystems/ShooterSubsystem.java" %}
```java
// Gearing from the motor rotor to the flywheel: one 1.5:1 stage.
.withGearing(new MechanismGearing(GearBox.fromReductionStages(1.5)))
// How hard the wheel is to spin up, used by the simulation: two 4 inch
// wheels, 0.68 kg (1.5 lb) in all, as solid discs: 0.5 x 0.68 kg x (0.0508 m)^2.
.withMomentOfInertia(KilogramSquareMeters.of(0.0009))
```
{% endcode %}
{% endtab %}

{% tab title="Java, Commands v3" %}
{% code title="mechanisms/ShooterMechanism.java" %}
```java
// Gearing from the motor rotor to the flywheel: one 1.5:1 stage.
.withGearing(new MechanismGearing(GearBox.fromReductionStages(1.5)))
// How hard the wheel is to spin up, used by the simulation: two 4 inch
// wheels, 0.68 kg (1.5 lb) in all, as solid discs: 0.5 x 0.68 kg x (0.0508 m)^2.
.withMomentOfInertia(KilogramSquareMeters.of(0.0009))
```
{% endcode %}
{% endtab %}

{% tab title="C++" %}
{% code title="cpp/subsystems/ShooterSubsystem.cpp" %}
```cpp
// Gearing from the motor rotor to the flywheel: one 1.5:1 stage.
.WithMotorGearing(MechanismGearing{GearBox::FromReductionStages({1.5})})
// How hard the wheel is to spin up, used by the simulation: two 4 inch
// wheels, 0.68 kg (1.5 lb) in all, as solid discs: 0.5 x 0.68 kg x (0.0508 m)^2.
.WithMOI(wpi::units::kilogram_square_meter_t{0.0009})
```
{% endcode %}
{% endtab %}
{% endtabs %}

**For your robot:** List your gearbox stages in order, from the motor outward, such as `3, 4` for a 3:1 stage followed by a 4:1 stage. Use `1` if the wheel is on the motor's shaft. Work out your wheel's moment of inertia from its mass and radius, adding the shaft and any pulleys if they are heavy.

Want the details? [Gearing and units](../../reference/editor/config/gearing-and-units.md)
{% endstep %}

{% step %}
### Direction, idle mode and current limit

Which way is positive, what the motor does when it is given no power, and the most current it may draw.

{% tabs %}
{% tab title="Java, Commands v2" %}
{% code title="subsystems/ShooterSubsystem.java" %}
```java
.withMotorInverted(false)
// Coast, so that the wheel spins down by itself instead of being braked to a stop.
.withZeroPower(MotorMode.COAST)
.withStatorCurrentLimit(Amps.of(40))
```
{% endcode %}
{% endtab %}

{% tab title="Java, Commands v3" %}
{% code title="mechanisms/ShooterMechanism.java" %}
```java
.withMotorInverted(false)
// Coast, so that the wheel spins down by itself instead of being braked to a stop.
.withZeroPower(MotorMode.COAST)
.withStatorCurrentLimit(Amps.of(40))
```
{% endcode %}
{% endtab %}

{% tab title="C++" %}
{% code title="cpp/subsystems/ShooterSubsystem.cpp" %}
```cpp
.WithMotorInverted(false)
// Coast, so that the wheel spins down by itself instead of being braked to a stop.
.WithZeroPower(Cfg::MotorMode::COAST)
.WithStatorCurrentLimit(40_A)
```
{% endcode %}
{% endtab %}
{% endtabs %}

**For your robot:** Flip the inversion if a positive output moves your flywheel the wrong way. Keep the current limit within what your wiring and breaker allow.

Want the details? [Motor basics](../../reference/editor/config/motor-basics.md) and [Limits and protection](../../reference/editor/config/limits-and-protection.md)
{% endstep %}

{% step %}
### Telemetry

The name the motor gets on your dashboard, and how much it publishes. `HIGH` publishes everything, which is what you want while tuning.

{% tabs %}
{% tab title="Java, Commands v2" %}
{% code title="subsystems/ShooterSubsystem.java" %}
```java
// Telemetry name and how much to publish.
.withTelemetry("ShooterMotor", TelemetryVerbosity.HIGH);
```
{% endcode %}
{% endtab %}

{% tab title="Java, Commands v3" %}
{% code title="mechanisms/ShooterMechanism.java" %}
```java
// Telemetry name and how much to publish.
.withTelemetry("ShooterMotor", TelemetryVerbosity.HIGH);
```
{% endcode %}
{% endtab %}

{% tab title="C++" %}
{% code title="cpp/subsystems/ShooterSubsystem.cpp" %}
```cpp
// Telemetry name and how much to publish.
.WithTelemetry("ShooterMotor", Cfg::TelemetryVerbosity::HIGH);
```
{% endcode %}
{% endtab %}
{% endtabs %}

Want the details? [Telemetry](../../understanding/telemetry.md)
{% endstep %}

{% step %}
### The motor controller object

This is the vendor's own object, exactly as you would create it without YAMS.

{% tabs %}
{% tab title="Java, Commands v2" %}
{% code title="subsystems/ShooterSubsystem.java" %}
```java
// The vendor motor controller object: a SPARK MAX with CAN ID 4.
private final SparkMax spark = new SparkMax(CANPort.CAN_S0, 4, MotorType.kBrushless);
```
{% endcode %}
{% endtab %}

{% tab title="Java, Commands v3" %}
{% code title="mechanisms/ShooterMechanism.java" %}
```java
// The vendor motor controller object: a SPARK MAX with CAN ID 4.
private final SparkMax spark = new SparkMax(CANPort.CAN_S0, 4, MotorType.kBrushless);
```
{% endcode %}
{% endtab %}

{% tab title="C++" %}
{% code title="include/subsystems/ShooterSubsystem.hpp" %}
```cpp
// The vendor motor controller object: a SPARK MAX with CAN ID 4.
rev::spark::SparkMax m_spark{wpi::CANPort::CAN_S0, 4, rev::spark::SparkMax::MotorType::kBrushless};
```
{% endcode %}
{% endtab %}
{% endtabs %}

**For your robot:** Change the CAN ID to yours. Using a different motor controller? See the [feature matrix](../../reference/editor/feature-matrix.md) for the wrapper that goes with it.
{% endstep %}

{% step %}
### The SmartMotorController

The wrapper brings the three together: the motor controller object, the motor plugged into it, and the config.

{% tabs %}
{% tab title="Java, Commands v2" %}
{% code title="subsystems/ShooterSubsystem.java" %}
```java
// The SmartMotorController applies the config to the SPARK MAX, which drives one NEO.
private final SmartMotorController motor = new SparkWrapper(spark, DCMotor.getNEO(1), smcConfig);
```
{% endcode %}
{% endtab %}

{% tab title="Java, Commands v3" %}
{% code title="mechanisms/ShooterMechanism.java" %}
```java
// The SmartMotorController applies the config to the SPARK MAX, which drives one NEO.
private final SmartMotorController motor = new SparkWrapper(spark, DCMotor.getNEO(1), smcConfig);
```
{% endcode %}
{% endtab %}

{% tab title="C++" %}
{% code title="include/subsystems/ShooterSubsystem.hpp" %}
```cpp
// The SmartMotorController, created in the constructor once the config is ready.
std::optional<yams::motorcontrollers::local::SparkWrapper> m_motor;
```
{% endcode %}

{% code title="cpp/subsystems/ShooterSubsystem.cpp" %}
```cpp
// The SmartMotorController applies the config to the SPARK MAX, which drives one NEO.
m_motor.emplace(&m_spark, wpi::math::DCMotor::NEO(1), &m_smcConfig);
```
{% endcode %}
{% endtab %}
{% endtabs %}

**For your robot:** Change the motor to the kind and the number of motors you have.

Want the details? [Smart motor controllers](../../reference/editor/)
{% endstep %}
{% endstepper %}

## The complete file

Every part above, in one file, with its imports.

{% tabs %}
{% tab title="Java, Commands v2" %}
{% code title="subsystems/ShooterSubsystem.java" lineNumbers="true" %}
```java
package first.robot.subsystems;

import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.KilogramSquareMeters;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
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
import yams.core.motorcontrollers.local.SparkWrapper;
import yams.core.telemetry.enums.TelemetryVerbosity;

public class ShooterSubsystem extends SubsystemBase {

  // What the motor controller should do: its control mode, gains and gearing.
  private final SmartMotorControllerConfig smcConfig = new SmartMotorControllerConfig(this)
      .withControlMode(ControlMode.CLOSED_LOOP)
      // Feedback constants (PID), on the real robot and in simulation.
      // A flywheel holds a speed, so it gets no motion profile.
      .withClosedLoopController(0.2, 0, 0)
      .withSimClosedLoopController(0.2, 0, 0)
      // Feedforward constants, on the real robot and in simulation.
      .withFeedforward(new SimpleMotorFeedforward(0, 0.19, 0))
      .withSimFeedforward(new SimpleMotorFeedforward(0, 0.19, 0))
      // Gearing from the motor rotor to the flywheel: one 1.5:1 stage.
      .withGearing(new MechanismGearing(GearBox.fromReductionStages(1.5)))
      // How hard the wheel is to spin up, used by the simulation: two 4 inch
      // wheels, 0.68 kg (1.5 lb) in all, as solid discs: 0.5 x 0.68 kg x (0.0508 m)^2.
      .withMomentOfInertia(KilogramSquareMeters.of(0.0009))
      .withMotorInverted(false)
      // Coast, so that the wheel spins down by itself instead of being braked to a stop.
      .withZeroPower(MotorMode.COAST)
      .withStatorCurrentLimit(Amps.of(40))
      // Telemetry name and how much to publish.
      .withTelemetry("ShooterMotor", TelemetryVerbosity.HIGH);

  // The vendor motor controller object: a SPARK MAX with CAN ID 4.
  private final SparkMax spark = new SparkMax(CANPort.CAN_S0, 4, MotorType.kBrushless);

  // The SmartMotorController applies the config to the SPARK MAX, which drives one NEO.
  private final SmartMotorController motor = new SparkWrapper(spark, DCMotor.getNEO(1), smcConfig);
}
```
{% endcode %}
{% endtab %}

{% tab title="Java, Commands v3" %}
{% code title="mechanisms/ShooterMechanism.java" lineNumbers="true" %}
```java
package first.robot.mechanisms;

import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.KilogramSquareMeters;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
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
import yams.core.motorcontrollers.local.SparkWrapper;
import yams.core.telemetry.enums.TelemetryVerbosity;

public class ShooterMechanism implements Mechanism {

  // What the motor controller should do: its control mode, gains and gearing.
  private final SmartMotorControllerConfig smcConfig = new SmartMotorControllerConfig(this)
      .withControlMode(ControlMode.CLOSED_LOOP)
      // Feedback constants (PID), on the real robot and in simulation.
      // A flywheel holds a speed, so it gets no motion profile.
      .withClosedLoopController(0.2, 0, 0)
      .withSimClosedLoopController(0.2, 0, 0)
      // Feedforward constants, on the real robot and in simulation.
      .withFeedforward(new SimpleMotorFeedforward(0, 0.19, 0))
      .withSimFeedforward(new SimpleMotorFeedforward(0, 0.19, 0))
      // Gearing from the motor rotor to the flywheel: one 1.5:1 stage.
      .withGearing(new MechanismGearing(GearBox.fromReductionStages(1.5)))
      // How hard the wheel is to spin up, used by the simulation: two 4 inch
      // wheels, 0.68 kg (1.5 lb) in all, as solid discs: 0.5 x 0.68 kg x (0.0508 m)^2.
      .withMomentOfInertia(KilogramSquareMeters.of(0.0009))
      .withMotorInverted(false)
      // Coast, so that the wheel spins down by itself instead of being braked to a stop.
      .withZeroPower(MotorMode.COAST)
      .withStatorCurrentLimit(Amps.of(40))
      // Telemetry name and how much to publish.
      .withTelemetry("ShooterMotor", TelemetryVerbosity.HIGH);

  // The vendor motor controller object: a SPARK MAX with CAN ID 4.
  private final SparkMax spark = new SparkMax(CANPort.CAN_S0, 4, MotorType.kBrushless);

  // The SmartMotorController applies the config to the SPARK MAX, which drives one NEO.
  private final SmartMotorController motor = new SparkWrapper(spark, DCMotor.getNEO(1), smcConfig);
}
```
{% endcode %}
{% endtab %}

{% tab title="C++" %}
{% code title="include/subsystems/ShooterSubsystem.hpp" lineNumbers="true" %}
```cpp
#pragma once

#include <optional>

#include <rev/SparkMax.h>
#include <wpi/commands2/SubsystemBase.hpp>
#include <wpi/hardware/bus/CANPort.hpp>

#include "yams/motorcontrollers/SmartMotorControllerConfig.hpp"
#include "yams/motorcontrollers/local/SparkWrapper.hpp"

class ShooterSubsystem : public wpi::cmd::SubsystemBase {
 public:
  ShooterSubsystem();

 private:
  // The vendor motor controller object: a SPARK MAX with CAN ID 4.
  rev::spark::SparkMax m_spark{wpi::CANPort::CAN_S0, 4, rev::spark::SparkMax::MotorType::kBrushless};

  // What the motor controller should do. It is filled in by the constructor.
  yams::motorcontrollers::SmartMotorControllerConfig m_smcConfig;

  // The SmartMotorController, created in the constructor once the config is ready.
  std::optional<yams::motorcontrollers::local::SparkWrapper> m_motor;
};
```
{% endcode %}

{% code title="cpp/subsystems/ShooterSubsystem.cpp" lineNumbers="true" %}
```cpp
#include "subsystems/ShooterSubsystem.hpp"

#include <wpi/math/controller/SimpleMotorFeedforward.hpp>
#include <wpi/math/system/DCMotor.hpp>
#include <wpi/units/angle.hpp>
#include <wpi/units/current.hpp>
#include <wpi/units/voltage.hpp>

#include "yams/gearing/GearBox.hpp"
#include "yams/gearing/MechanismGearing.hpp"

using namespace yams::gearing;
using Cfg = yams::motorcontrollers::SmartMotorControllerConfig;
using Feedforward = wpi::math::SimpleMotorFeedforward<wpi::units::turns>;

ShooterSubsystem::ShooterSubsystem() {
  // What the motor controller should do: its control mode, gains and gearing.
  m_smcConfig
      .WithSubsystem(this)
      .WithClosedLoopMode()
      // Feedback constants (PID), on the real robot and in simulation.
      // A flywheel holds a speed, so it gets no motion profile.
      .WithFeedback(0.2, 0, 0)
      .WithSimClosedLoopController(0.2, 0, 0)
      // Feedforward constants (kS and kV), on the real robot and in simulation.
      .WithFeedforward(Feedforward{0_V, wpi::units::unit_t<Feedforward::kv_unit>{0.19}})
      .WithSimFeedforward(Feedforward{0_V, wpi::units::unit_t<Feedforward::kv_unit>{0.19}})
      // Gearing from the motor rotor to the flywheel: one 1.5:1 stage.
      .WithMotorGearing(MechanismGearing{GearBox::FromReductionStages({1.5})})
      // How hard the wheel is to spin up, used by the simulation: two 4 inch
      // wheels, 0.68 kg (1.5 lb) in all, as solid discs: 0.5 x 0.68 kg x (0.0508 m)^2.
      .WithMOI(wpi::units::kilogram_square_meter_t{0.0009})
      .WithMotorInverted(false)
      // Coast, so that the wheel spins down by itself instead of being braked to a stop.
      .WithZeroPower(Cfg::MotorMode::COAST)
      .WithStatorCurrentLimit(40_A)
      // Telemetry name and how much to publish.
      .WithTelemetry("ShooterMotor", Cfg::TelemetryVerbosity::HIGH);

  // The SmartMotorController applies the config to the SPARK MAX, which drives one NEO.
  m_motor.emplace(&m_spark, wpi::math::DCMotor::NEO(1), &m_smcConfig);
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

The PID and feedforward numbers on this page are a place to start, not tuned values. You tune them in [Step 5: Tune it](tune.md).

## Check it

Open the command palette with <kbd>Ctrl+Shift+P</kbd> and run **WPILib: Build Robot Code**. It should build. Nothing moves yet, because nothing tells the motor what to do.

* **`SparkMax` or `SparkWrapper` cannot be found:** REVLib is not installed. See [Install your motor controller's library](../project-setup/#install-your-motor-controllers-library).
* **`SmartMotorControllerConfig` cannot be found:** YAMS is not installed, or the import is from the other command framework (`yams.commands2` or `yams.commands3`).
