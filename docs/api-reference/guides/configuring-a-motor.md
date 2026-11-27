# Configuring a motor controller

**Goal:** Build and instantiate a `SmartMotorController` ready for use in a mechanism or standalone.

The gains in this guide are placeholders, as [the overview](guides.md#the-gains-in-these-guides) explains.

***

{% stepper %}
{% step %}
#### Choose your hardware and import vendor libraries

Identify which motor controller you are using:

* **REV SPARK MAX / SPARK FLEX**: requires the REVLib 2027 vendordep. Wrapped by `SparkWrapper`.
* **CTRE TalonFX / TalonFXS**: requires the Phoenix 6 vendordep. Wrapped by `TalonFXWrapper` / `TalonFXSWrapper`.
* **Thrifty Nova**: requires the ThriftyLib 2027 vendordep. Wrapped by `NovaWrapper`.
* **FIRST A301**: requires the REVLib 2027 vendordep. Wrapped by `A301Wrapper`.

Add the appropriate vendordep to your project before proceeding.

**Check it:** the project builds, and the vendor library shows under **Vendor Dependencies** in the WPILib extension.
{% endstep %}

{% step %}
#### Create a `SmartMotorControllerConfig` and chain builder methods

`SmartMotorControllerConfig` follows the builder pattern. Call the methods you need and chain them together. At minimum, set gearing, a closed loop controller (PID), and zero power mode.

In Java, import `SmartMotorControllerConfig` from the command framework package you installed and pass the owning `Subsystem` (Commands v2) or `Mechanism` (Commands v3) to the constructor. Everything else comes from `yams.core`.

{% tabs %}
{% tab title="Commands v2" %}
```java
import yams.commands2.config.SmartMotorControllerConfig;
import yams.core.gearing.GearBox;
import yams.core.gearing.MechanismGearing;
import yams.core.motorcontrollers.enums.MotorMode;
import yams.core.telemetry.enums.TelemetryVerbosity;
```
{% endtab %}

{% tab title="Commands v3" %}
```java
import yams.commands3.config.SmartMotorControllerConfig;
import yams.core.gearing.GearBox;
import yams.core.gearing.MechanismGearing;
import yams.core.motorcontrollers.enums.MotorMode;
import yams.core.telemetry.enums.TelemetryVerbosity;
```
{% endtab %}
{% endtabs %}

**Java: SPARK MAX with arm control**

```java
// "this" is the Subsystem (Commands v2) or Mechanism (Commands v3) that owns the motor.
SmartMotorControllerConfig config = new SmartMotorControllerConfig(this)
    .withMotorInverted(false)
    .withZeroPower(MotorMode.BRAKE)
    .withGearing(new MechanismGearing(GearBox.fromTeeth(14, 72)))
    .withClosedLoopController(0.5, 0.0, 0.01)            // Placeholder PID gains.
    .withFeedforward(new ArmFeedforward(0.1, 0.5, 0.01)) // Placeholder kS, kG, kV.
    .withSoftLimits(Degrees.of(-90), Degrees.of(90))
    .withStatorCurrentLimit(Amps.of(40))
    .withTelemetry("ShoulderMotor", TelemetryVerbosity.HIGH);
```

**C++: TalonFX with arm control**

```cpp
#include "yams/gearing/GearBox.hpp"
#include "yams/gearing/MechanismGearing.hpp"
#include "yams/motorcontrollers/SmartMotorControllerConfig.hpp"

using namespace yams;
using Cfg = motorcontrollers::SmartMotorControllerConfig;

Cfg config;
config.WithSubsystem(this)
      .WithMotorInverted(false)
      .WithZeroPower(Cfg::MotorMode::BRAKE)
      .WithMotorGearing(gearing::MechanismGearing{
          gearing::GearBox::FromTeeth({14, 72})})
      .WithFeedback(0.5, 0.0, 0.01)                      // Placeholder PID gains.
      .WithFeedforward(wpi::math::ArmFeedforward{0.1_V, 0.5_V,  // Placeholder kS, kG, kV, kA.
                                                 wpi::units::unit_t<wpi::math::ArmFeedforward::kv_unit>{0.01},
                                                 wpi::units::unit_t<wpi::math::ArmFeedforward::ka_unit>{0.0}})
      .WithMechanismLimits(-90_deg, 90_deg)
      .WithStatorCurrentLimit(40_A)
      .WithTelemetry("ShoulderMotor", Cfg::TelemetryVerbosity::HIGH);
```

**Check it:** the project still builds. Nothing runs yet, because the config is not attached to a motor controller.
{% endstep %}

{% step %}
#### Construct the concrete motor controller

Pass the vendor hardware object, the WPILib `DCMotor` model, and the config to the appropriate constructor. The Java wrappers live in `yams.core.motorcontrollers.local` (`SparkWrapper`, `NovaWrapper`, `A301Wrapper`) and `yams.core.motorcontrollers.remote` (`TalonFXWrapper`, `TalonFXSWrapper`).

**Java (REV SPARK):**

```java
SmartMotorController motor = new SparkWrapper(
    new SparkMax(CANPorts.fromBusId(1), 1, MotorType.kBrushless),
    DCMotor.getNEO(1),
    config
);
```

**Java (CTRE TalonFX):**

```java
SmartMotorController motor = new TalonFXWrapper(
    new TalonFX(1, new CANBus(CANPort.CAN_S0)),
    DCMotor.getKrakenX60(1),
    config
);
```

**Java (Thrifty Nova):**

```java
// Nova driving a NEO on CAN bus 0, CAN ID 3
SmartMotorController motor = new NovaWrapper(
    new Nova(0, 3, MotorType.NEO),
    DCMotor.getNEO(1),
    config
);
```

**Java (FIRST A301):**

```java
// A301 on CAN bus 1, CAN ID 20. Every A301 has the same motor, so there is no DCMotor.
SmartMotorController motor = new A301Wrapper(
    new A301(CANPorts.fromBusId(1), 20),
    config
);
```

**C++ (REV SPARK):**

```cpp
#include "yams/motorcontrollers/local/SparkWrapper.hpp"

rev::spark::SparkMax spark{wpi::CANPort::CAN_S0, 1, rev::spark::SparkMax::MotorType::kBrushless};
yams::motorcontrollers::local::SparkWrapper motor{&spark, wpi::math::DCMotor::NEO(1), &config};
```

**C++ (CTRE TalonFX):**

```cpp
#include "yams/motorcontrollers/remote/TalonFXWrapper.hpp"

ctre::phoenix6::hardware::TalonFX talon{1, ctre::phoenix6::CANBus{}};
yams::motorcontrollers::remote::TalonFXWrapper motor{&talon, wpi::math::DCMotor::KrakenX60(1), &config};
```

**C++ (CTRE TalonFXS driving a NEO):**

```cpp
#include "yams/motorcontrollers/remote/TalonFXSWrapper.hpp"

using yams::motorcontrollers::remote::TalonFXSWrapper;

ctre::phoenix6::hardware::TalonFXS talonFXS{1, ctre::phoenix6::CANBus{}};
TalonFXSWrapper motor{&talonFXS, wpi::math::DCMotor::NEO(1), TalonFXSWrapper::MotorArrangement::NEO, &config};
```

**C++ (Thrifty Nova):**

```cpp
#include "yams/motorcontrollers/local/NovaWrapper.hpp"

thrifty::Nova nova{0, 3, thrifty::Motor::NEO};
yams::motorcontrollers::local::NovaWrapper motor{&nova, wpi::math::DCMotor::NEO(1), &config};
```

**C++ (FIRST A301):**

```cpp
#include "yams/motorcontrollers/local/A301Wrapper.hpp"

first::a301::A301 a301{wpi::CANPort::CAN_S0, 3};
yams::motorcontrollers::local::A301Wrapper motor{&a301, &config};
```

In C++ the hardware object and the config are passed by pointer and must outlive the wrapper, so declare them as subsystem members.

**Check it:** the code builds and starts in simulation without an exception. With `TelemetryVerbosity.HIGH`, the motor's values show in NetworkTables under **Mechanisms**, named after `withTelemetry("ShoulderMotor", ...)`.
{% endstep %}

{% step %}
#### Pass the result to a mechanism or use it directly

The returned `SmartMotorController` is ready to hand to a mechanism. Continue with the guide for yours: [an arm](building-an-arm.md), [an elevator](building-an-elevator.md), [a flywheel](building-a-flywheel.md) or [a swerve drive](setting-up-swerve.md), or the [C++ mechanisms](building-mechanisms-cpp.md).

```java
Arm arm = new Arm(armConfig, motor);
```

**Check it:** the mechanism guide's own checks, starting with the mechanism showing up in simulation.
{% endstep %}
{% endstepper %}

***

## Notes

{% hint style="info" %}
If you are using an **absolute encoder** and the mechanism works across the point where its reading wraps, call `.withWrapping(min, max)` on the config to set the range it reads (`Rotations.of(-0.5), Rotations.of(0.5)` for \[-0.5, 0.5), `Rotations.of(0), Rotations.of(1)` for \[0, 1)). Without it, a SPARK or Nova absolute encoder reads \[0, 1) and jumps from 1 to 0 rotations at that boundary. For a mechanism that turns all the way round, such as a turret, use `.withContinuousWrapping(min, max)` so the closed loop also takes the shortest path. See [Wrapping](../java-reference/motor-controllers/smart-motor-controller-config.md#wrapping).
{% endhint %}

{% hint style="info" %}
**Live tuning:** with `TelemetryVerbosity.HIGH`, the config automatically registers a "Live Tuning" command for the owning subsystem or mechanism. If you set up the motor controller's telemetry yourself with `setupTelemetry(NetworkTable, NetworkTable)`, call `config.setupLiveTuning()` afterwards (Java). Tunable fields are in degrees and RPM (meters and meters per second for linear mechanisms), while read-only fields are in rotations and rotations per second.
{% endhint %}

{% hint style="info" %}
**Encoder getters:** `getMechanismPosition()` / `getMechanismVelocity()` report the feedback the closed loop uses. Use `getRelativeMechanismPosition()` / `getRelativeMechanismVelocity()` to read only the motor's relative encoder, and `getExternalEncoderMechanismPosition()` / `getExternalEncoderMechanismVelocity()` to read only the external encoder. The C++ equivalents use the same names in `PascalCase`.
{% endhint %}

## If it did not work

* **It does not build.** The vendordep for your motor controller is missing, or an import is. Check step one, then the imports at the top of the arm guide's [complete class](building-an-arm.md#integrate-into-a-subsystem).
* **It throws when the code starts.** Read the exception message: YAMS names the missing setting and the method that sets it.
* **Nothing shows in NetworkTables.** Telemetry needs `withTelemetry(...)` on the config, and the mechanism's `updateTelemetry()` running every loop.

[Troubleshooting](https://yams.yassrobotics.com/documentation/tutorials/troubleshooting) covers more.

***

## Related pages

* [SmartMotorControllerConfig](../java-reference/motor-controllers/smart-motor-controller-config.md)
* [MechanismGearing](../java-reference/gearing/mechanism-gearing.md)
