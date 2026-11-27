# Building a flywheel subsystem (C++)

**Goal:** Create a `FlyWheel` subsystem in C++ with velocity control and physics simulation.

The gains in this guide are placeholders, as [the overview](guides.md#the-gains-in-these-guides) explains. They match the Java [flywheel guide](building-a-flywheel.md).

***

## Prerequisites

Install the Commands v2 YAMS vendordep (`https://cdn.yassrobotics.com/yams_commands2.json`). `YAMS.hpp` does not pull in the vendor wrappers, so include the wrapper for your motor controller explicitly.

```cpp
#include <yams/YAMS.hpp>

// Or individually:
#include <yams/mechanisms/velocity/FlyWheel.hpp>
#include <yams/mechanisms/config/FlyWheelConfig.hpp>
#include <yams/gearing/MechanismGearing.hpp>
#include <yams/motorcontrollers/SmartMotorControllerConfig.hpp>

// Vendor wrapper (always included individually):
#include <yams/motorcontrollers/remote/TalonFXWrapper.hpp>

// WPILib 2027 headers used below:
#include <wpi/commands2/CommandPtr.hpp>
#include <wpi/commands2/SubsystemBase.hpp>
#include <wpi/commands2/button/Trigger.hpp>
#include <wpi/math/controller/SimpleMotorFeedforward.hpp>
#include <wpi/math/system/DCMotor.hpp>
#include <wpi/units/angular_velocity.hpp>
```

***

{% stepper %}
{% step %}
#### Configure the motor for velocity control

Use `WithFeedback` for velocity PID and `WithFeedforward(const wpi::math::SimpleMotorFeedforward<wpi::units::turns>&)` for steady-state tracking. Set the zero power mode to `COAST` so the wheel decelerates freely.

```cpp
yams::motorcontrollers::SmartMotorControllerConfig motorConfig;
motorConfig
    .WithSubsystem(this)
    .WithZeroPower(yams::motorcontrollers::SmartMotorControllerConfig::MotorMode::COAST)
    .WithMotorGearing(yams::gearing::MechanismGearing(1.5))
    .WithFeedback(0.0003, 0.0, 0.0)              // Placeholder PID gains.
    .WithFeedforward(wpi::math::SimpleMotorFeedforward<wpi::units::turns>{   // Placeholder kS, kV, kA.
        0.1_V, wpi::units::unit_t<wpi::math::SimpleMotorFeedforward<wpi::units::turns>::kv_unit>{0.002},
        wpi::units::unit_t<wpi::math::SimpleMotorFeedforward<wpi::units::turns>::ka_unit>{0.0}})    // kS, kV, kA
    .WithMOI(0.0508_m, 0.18_kg)                // roller radius, mass; required for simulation
    .WithStatorCurrentLimit(80_A)
    .WithTelemetry("ShooterMotor");
```

> **Simulation note:** `WithMOI(radius, mass)` sets the moment of inertia for physics simulation. It belongs on `SmartMotorControllerConfig`, not on `FlyWheelConfig`.

**Check it:** the project builds.
{% endstep %}

{% step %}
#### Construct the motor controller

The wrapper takes pointers to the vendor hardware object and to the config. Both must outlive the wrapper, so keep them as subsystem members. The wrapper applies the config when it is constructed, so finish configuring `motorConfig` first.

```cpp
ctre::phoenix6::hardware::TalonFX talon{3, ctre::phoenix6::CANBus{}};

yams::motorcontrollers::remote::TalonFXWrapper motor{
    &talon,
    wpi::math::DCMotor::KrakenX60(1),
    &motorConfig
};
```

**Check it:** the project builds.
{% endstep %}

{% step %}
#### Create a `FlyWheelConfig`

`WithRollerDiameter` enables surface speed calculations. All fields are optional.

```cpp
yams::mechanisms::config::FlyWheelConfig flywheelConfig;
flywheelConfig
    .WithRollerDiameter(0.1016_m)    // 4-inch roller
    .WithTelemetryName("Shooter");
```

{% hint style="info" %}
Moment of inertia is set on `SmartMotorControllerConfig` via `WithMOI()`, not on `FlyWheelConfig`. `FlyWheelConfig` only holds the roller diameter, telemetry name, and sim color.
{% endhint %}

**Check it:** the project builds.
{% endstep %}

{% step %}
#### Construct the `FlyWheel`

```cpp
yams::mechanisms::velocity::FlyWheel flyWheel{&flywheelConfig, &motor};
```

**Check it:** run the simulation. The code starts without an exception.
{% endstep %}

{% step %}
#### Integrate into a subsystem

The wrapper and the mechanism are held in `std::optional` members and built with `emplace` at the end of the constructor, after the configs are filled in.

```cpp
class ShooterSubsystem : public wpi::cmd::SubsystemBase {
public:
    static constexpr wpi::units::degrees_per_second_t kShootSpeed{4500.0 * 6.0};   // RPM to deg/s
    static constexpr wpi::units::degrees_per_second_t kSpeedTolerance{600.0};       // ~100 RPM

    ShooterSubsystem() {
        motorConfig_
            .WithSubsystem(this)
            .WithZeroPower(yams::motorcontrollers::SmartMotorControllerConfig::MotorMode::COAST)
            .WithMotorGearing(yams::gearing::MechanismGearing(1.5))
            .WithFeedback(0.0003, 0.0, 0.0)      // Placeholder PID gains.
            .WithFeedforward(wpi::math::SimpleMotorFeedforward<wpi::units::turns>{   // Placeholder kS, kV, kA.
                0.1_V, wpi::units::unit_t<wpi::math::SimpleMotorFeedforward<wpi::units::turns>::kv_unit>{0.002},
                wpi::units::unit_t<wpi::math::SimpleMotorFeedforward<wpi::units::turns>::ka_unit>{0.0}})
            .WithMOI(0.0508_m, 0.18_kg)
            .WithStatorCurrentLimit(80_A)
            .WithTelemetry("ShooterMotor");

        motor_.emplace(&talon_, wpi::math::DCMotor::KrakenX60(1), &motorConfig_);

        flywheelConfig_
            .WithRollerDiameter(0.1016_m)
            .WithTelemetryName("Shooter");

        flyWheel_.emplace(&flywheelConfig_, &motor_.value());
    }

    /** Spin to a target velocity and hold indefinitely. */
    wpi::cmd::CommandPtr SpinUpCommand(wpi::units::degrees_per_second_t velocity) {
        return flyWheel_->Run(velocity);
    }

    /** Spin up and finish once within tolerance; use to gate shooting. */
    wpi::cmd::CommandPtr SpinUpAndWait(wpi::units::degrees_per_second_t velocity) {
        return flyWheel_->RunTo(velocity, kSpeedTolerance);
    }

    /** Trigger that is true whenever the wheel is near the current shoot speed. */
    wpi::cmd::Trigger ReadyToShoot() {
        return flyWheel_->IsNear(kShootSpeed, kSpeedTolerance);
    }

    wpi::units::degrees_per_second_t GetVelocity() const {
        return flyWheel_->GetVelocity();
    }

    void Periodic() override { flyWheel_->UpdateTelemetry(); }
    void SimulationPeriodic() override { flyWheel_->SimIterate(); }

private:
    ctre::phoenix6::hardware::TalonFX talon_{3, ctre::phoenix6::CANBus{}};
    yams::motorcontrollers::SmartMotorControllerConfig motorConfig_;
    std::optional<yams::motorcontrollers::remote::TalonFXWrapper> motor_;
    yams::mechanisms::config::FlyWheelConfig flywheelConfig_;
    std::optional<yams::mechanisms::velocity::FlyWheel> flyWheel_;
};
```

**Check it:** run the simulation and open **NetworkTables -> Mechanisms -> Shooter -> mechanism** in the Sim GUI. Schedule `SpinUpCommand(kShootSpeed)` from a button, and the wheel's line spins. Its velocity, in the telemetry next to it, climbs toward 4500 RPM.
{% endstep %}

{% step %}
#### Command factories and triggers

| Method                                            | Returns      | Description                                           |
| ------------------------------------------------- | ------------ | ----------------------------------------------------- |
| `Run(degrees_per_second_t velocity)`              | `CommandPtr` | Spin at velocity, hold indefinitely                   |
| `Run(meters_per_second_t surfaceSpeed)`           | `CommandPtr` | Spin at surface speed (requires `WithRollerDiameter`) |
| `RunTo(degrees_per_second_t velocity, tolerance)` | `CommandPtr` | Spin up, finish when within tolerance                 |
| `IsNear(degrees_per_second_t velocity, within)`   | `Trigger`    | True while within tolerance of velocity               |
| `Gte(degrees_per_second_t velocity)`              | `Trigger`    | True when speed >= threshold                          |
| `Lte(degrees_per_second_t velocity)`              | `Trigger`    | True when speed <= threshold                          |
| `Between(degrees_per_second_t start, end)`        | `Trigger`    | True while between the two speeds                     |
{% endstep %}

{% step %}
#### Sequencing shooter commands

Use `RunTo` to block until the wheel is at speed before feeding. The indexer is its own subsystem with its own YAMS `FlyWheel` (rollers, feeders, and indexers are flywheels too, even when run open loop).

```cpp
wpi::cmd::CommandPtr shoot = shooter_->SpinUpAndWait(ShooterSubsystem::kShootSpeed)
    .AndThen(indexer_->FeedCommand());
```

Or bind the `ReadyToShoot()` trigger to allow feeding whenever the wheel is up:

```cpp
shooter_->ReadyToShoot().WhileTrue(indexer_->FeedCommand());
```

**Check it:** in simulation, the indexer only runs once the shooter is within about 100 RPM of the shoot speed.
{% endstep %}
{% endstepper %}

## If it did not work

* **It does not build.** Check the vendordep, the `#include` for the wrapper (`YAMS.hpp` does not include it), and that the TalonFX and the config are members that outlive the wrapper.
* **The wheel never spins in simulation.** Check that `SimulationPeriodic()` calls `SimIterate()`, that `WithSubsystem(this)` is set, and that `WithMOI(...)` is on the motor config.
* **The wheel spins up very slowly or never reaches speed.** Expected with the placeholder gains: most of a flywheel's voltage comes from `kV`. Tune them as in [Tune it](https://yams.yassrobotics.com/documentation/tutorials/shooter-flywheels/tune).

[Troubleshooting](https://yams.yassrobotics.com/documentation/tutorials/troubleshooting) covers more.

***

## Notes

{% hint style="info" %}
`RunTo(velocity, tolerance)` ends once the wheel reaches the target. Use it in command sequences where the next step should not begin until the wheel is at speed.

`IsNear(velocity, tolerance)` (as a `Trigger`) stays true while the wheel remains within tolerance; use it for continuously gated logic such as a conveyor that feeds whenever the shooter is ready. (The Java equivalent is `near(...)`.)
{% endhint %}

***

## Examples

Complete C++ implementations from the `cpptest` reference project:

{% @github-files/github-code-block url="https://github.com/Yet-Another-Software-Suite/YAMS/blob/master/examples/cpptest/src/main/cpp/subsystems/ShooterSubsystem.cpp" %}

***

## Related pages

* [FlyWheel (C++)](../c++-reference/mechanisms/flywheel.md)
* [FlyWheelConfig (C++)](../c++-reference/config/flywheel-config.md)
* [SmartMotorControllerConfig (C++)](../c++-reference/motor-controllers/smart-motor-controller-config.md)
* [TalonFXWrapper (C++)](../c++-reference/motor-controllers/talonfx-wrapper.md)
* [Java equivalent: Building a Flywheel](building-a-flywheel.md)
