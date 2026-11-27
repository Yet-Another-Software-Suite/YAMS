# NovaWrapper

**Namespace:** `yams::motorcontrollers::local`\
**Extends:** `SmartMotorController`\
**Header:** `#include <yams/motorcontrollers/local/NovaWrapper.hpp>`\
**Doxygen:** [`NovaWrapper`](https://doxygen.yassrobotics.com/yams/classyams_1_1motorcontrollers_1_1local_1_1NovaWrapper.html)

`SmartMotorController` implementation for The Thrifty Bot's Thrifty Nova motor controller, using ThriftyLib 2027.

**Java equivalent:** [Java NovaWrapper](../../java-reference/motor-controllers/nova-wrapper.md)

***

## Constructor

```cpp
NovaWrapper(thrifty::Nova* nova, wpi::math::DCMotor motor, SmartMotorControllerConfig* config)
```

| Parameter | Type                          | Description                                                             |
| --------- | ----------------------------- | ----------------------------------------------------------------------- |
| `nova`    | `thrifty::Nova*`              | Nova hardware object. Must outlive this wrapper.                        |
| `motor`   | `wpi::math::DCMotor`          | DC motor model used for simulation (e.g. `wpi::math::DCMotor::NEO(1)`). |
| `config`  | `SmartMotorControllerConfig*` | Configuration to apply. Must outlive this wrapper.                      |

`thrifty::Nova` comes from `<thrifty/nova/Nova.h>` and is created with a CAN bus, a CAN ID and a motor type, for example `thrifty::Nova nova{0, 3, thrifty::Motor::NEO};`.

A vendor config passed with `WithVendorConfig` must be a `thrifty::NovaConfigBatch`. It is applied before the YAMS config, so YAMS options override it. Any other vendor config type throws `SmartMotorControllerConfigurationException`. When `WithResetPreviousConfig(true)` is set, a factory reset is sent first.

***

## Key methods

`NovaWrapper` implements the full [`SmartMotorController`](smart-motor-controller.md) interface (open loop outputs, closed loop setpoints, encoder reads and writes, live configuration setters, telemetry and live tuning). See that page for the complete method list. The notes below cover Nova-specific behavior.

### Closed loop control

Where the closed loop runs depends on the controller (ThriftyLib 2027.0.0-alpha-6):

| Controller                                                                                                                                                | Where it runs                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    |
| --------------------------------------------------------------------------------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| PID, with or without a trapezoidal position profile                                                                                                       | On the Nova: its `Pid0`/`Pid1` slots, and its `Mp0`/`Mp1` motion profile with the feedforward's kS, kV, kA and kG (cosine gravity for an arm feedforward).                                                                                                                                                                                                                                                                                                                                                                                                                                       |
| A feedforward without a motion profile, such as a swerve drive motor's or flywheel's kS and kV, or an arm's kG                                            | On the Nova. YAMS sends kS, kV, kA and kG to the slot's motion profile either way, and the Nova applies them, with the gravity type (cosine for an arm), with or without a motion profile, in plain position and velocity control too. A velocity request only carries a feedforward force from `SetVelocity(velocity, force)`. In simulation, kS (by the sign of the velocity target, or of the position error) and kG apply in position and velocity control too, and kV times the velocity target in velocity control, but not kA, since a plain velocity request has no acceleration target. |
| [LQR](../math/lqr-controller.md), exponential profile or velocity trapezoidal profile                                                                     | On the SystemCore. YAMS runs the loop in a notifier, commands the Nova with voltage, and raises an alert (see [Alerts](nova-wrapper.md#alerts)).                                                                                                                                                                                                                                                                                                                                                                                                                                                 |
| Continuous wrapping over any one rotation range of an absolute encoder closing the loop with 1:1 external encoder gearing                                 | On the Nova. The Nova wraps it itself (`AbsWrapping`), and YAMS sends each position target folded into `[0, 1)`. Positions are still reported in the configured range.                                                                                                                                                                                                                                                                                                                                                                                                                           |
| Continuous wrapping of the motor's own encoder or a quadrature encoder, such as a swerve azimuth seeded from a CANcoder                                   | On the Nova. YAMS sends each target as the equivalent nearest the mechanism (at 3.2 rotations, a target of 0.25 is sent as 3.25), so the Nova's position loop takes the short way, like a SPARK MAX with a CANcoder.                                                                                                                                                                                                                                                                                                                                                                             |
| An absolute encoder closing the loop read over a range other than `[0, 1)` without continuous wrapping, such as `WithWrapping(-0.5_tr, 0.5_tr)` on an arm | On the SystemCore, as above. The Nova's `[0, 1)` wrapping point would fall inside the range.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     |
| Continuous wrapping of an absolute encoder geared to the mechanism (external encoder gearing other than 1:1)                                              | On the SystemCore, as above. It wraps once per encoder rotation, inside the mechanism's range.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   |

The SystemCore wraps any range on any sensor. `GetClosedLoopControllerPeriod()` shows where the loop runs: 1 ms on the Nova, the closed loop control period (20 ms by default) on the SystemCore.

When the Nova runs the loop, YAMS sends it a sensor-to-mechanism ratio, so targets, gains and soft limits are in mechanism rotations. kI and kD are sent as configured (per second), the closed loop tolerance is the slot's allowable error, the mechanism limits become the Nova's soft limits, and live setters update the Nova. `ApplyConfig` throws `SmartMotorControllerConfigurationException` for:

| Option                                                                                                                                   | Why                                                          |
| ---------------------------------------------------------------------------------------------------------------------------------------- | ------------------------------------------------------------ |
| `SLOT_2` or `SLOT_3`                                                                                                                     | The Nova has two closed loop slots.                          |
| Only one mechanism limit                                                                                                                 | The Nova's soft limits need both an upper and a lower limit. |
| `WithTemperatureCutoff`, `WithClosedLoopControllerMaximumVoltage`, `WithFeedbackSynchronizationThreshold`, `WithClosedLoopControlPeriod` | Only the SystemCore's closed loop applies them.              |

`WithClosedLoopControlPeriod(...)` (default 20 ms) sets the SystemCore's loop period; it is only valid in closed loop mode.

### Encoder reads

| Method                                                                            | Nova behavior                                                                                                |
| --------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------ |
| `GetMechanismPosition()` / `GetMechanismVelocity()`                               | External encoder when `WithUseExternalFeedbackEncoder(true)` is set, otherwise the motor's internal encoder. |
| `GetRelativeMechanismPosition()` / `GetRelativeMechanismVelocity()`               | Always the motor's internal encoder, with the motor gearing applied.                                         |
| `GetExternalEncoderMechanismPosition()` / `GetExternalEncoderMechanismVelocity()` | The external encoder with the external encoder gearing applied. Empty when no external encoder is attached.  |
| `GetSupplyCurrent()`                                                              | Supply current reported by the Nova (the sim supplier's value in simulation).                                |

### Restrictions

| Option                                                         | Behavior                                                                                                                                                                                   |
| -------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| `WithOpenLoopRampRate` together with `WithClosedLoopRampRate`  | The Nova has one ramp rate. Different open and closed loop values throw `SmartMotorControllerConfigurationException`.                                                                      |
| `WithEncoderInverted(true)` / `SetEncoderInverted`             | The Nova's internal encoder cannot be inverted. `WithEncoderInverted` throws a `std::invalid_argument` when the config is applied, and `SetEncoderInverted` throws a `std::runtime_error`. |
| `WithVendorControlRequest`                                     | Not supported on the Nova. Throws `SmartMotorControllerConfigurationException`.                                                                                                            |
| `SetEncoderVelocity` (on a real robot)                         | Throws `std::runtime_error`; the Nova cannot have its encoder velocity set.                                                                                                                |
| Closed loop tolerance with an [LQR](../math/lqr-controller.md) | Throws `std::invalid_argument`.                                                                                                                                                            |

***

## External encoders

Pass one of the following to `WithExternalEncoder(...)`:

| Value                                      | Encoder                                                                                                                                                                                         |
| ------------------------------------------ | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `thrifty::Motor::FeedbackSensorType::ABS`  | An absolute encoder on the Nova's data port. Its type comes from the vendor config, such as `thrifty::NovaConfig::AbsoluteEncoderType(thrifty::Motor::AbsoluteEncoderType::REV_ENCODER)`.       |
| `thrifty::Motor::FeedbackSensorType::QUAD` | A quadrature encoder on the Nova's data port. Its type comes from the vendor config, such as `thrifty::NovaConfig::QuadratureEncoderType(thrifty::Motor::QuadratureEncoderType::Custom(2048))`. |
| `thrifty::CanEncoder*`                     | A Thrifty CAN Encoder on the same CAN bus.                                                                                                                                                      |

Any other value throws `std::invalid_argument`.

An absolute or CAN encoder reads `[0, 1)` unless a wrapping range is set with `WithWrapping(min, max)`, such as `WithWrapping(-0.5_tr, 0.5_tr)`. Any range works. The Nova closes its loop on the encoder over `[0, 1)`, or over any range with `WithContinuousWrapping` and 1:1 external encoder gearing; with any other range the closed loop runs on the SystemCore. Only `WithContinuousWrapping` turns on the Nova's `AbsWrapping`. Zero offset is only available for absolute encoders and throws a `SmartMotorControllerConfigurationException` for a quadrature encoder. A quadrature encoder counts from where it powers on, so it is set to the starting position (or zero) when the config is applied.

`WithExternalEncoderZeroOffset`, `WithExternalEncoderInverted` and `WithExternalEncoderGearing` all require `WithExternalEncoder(...)`; setting them without one throws `SmartMotorControllerConfigurationException`.

When no starting position is configured and an absolute or CAN encoder is attached, the internal encoder is seeded from it when the config is applied.

***

## Followers

`WithFollowers` accepts `thrifty::Nova*` followers only. Any other type throws `std::invalid_argument`. Use `WithLooselyCoupledFollowers` to forward setpoints to other `SmartMotorController` instances instead.

***

## Simulation

In simulation the wrapper creates a DC motor simulation from the motor model, the configured MOI and the motor gearing, and reports its current draw to [`BatterySim`](../sensor/battery-sim.md) every loop. `SimIterate()` also steps ThriftyLib's own simulation for the Nova and any attached `thrifty::CanEncoder`. ThriftyLib's simulation ignores the Nova's PID, so a `NovaSimSupplier` runs the Nova's closed loop in simulation. `SetSimSupplier` also makes a ThriftyLib `MotorSim` from the sim supplier, which drives a `thrifty::CanEncoder` (through `CanEncoderConfig::SimMotor`) and the Nova's data port encoders in simulation. Set a realistic moment of inertia with `SmartMotorControllerConfig::WithMOI(...)` for realistic current draw.

***

## Alerts

Alerts are published in the `YAMS` alert group and cleared when the wrapper is destroyed.

| Alert                                                         | When                                                                                                                                                                                                                                            |
| ------------------------------------------------------------- | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `<name> closed loop controller is running on the SystemCore.` | The loop runs on the SystemCore in closed loop mode: an [LQR](../math/lqr-controller.md), exponential profile or velocity trapezoidal profile, or a config the Nova can't run (see [Closed loop control](nova-wrapper.md#closed-loop-control)). |

***

## Example

```cpp
#include <thrifty/nova/Nova.h>
#include <wpi/commands2/SubsystemBase.hpp>
#include <yams/motorcontrollers/SmartMotorControllerConfig.hpp>
#include <yams/motorcontrollers/local/NovaWrapper.hpp>

using namespace yams::motorcontrollers;
using namespace yams::motorcontrollers::local;

class ArmSubsystem : public wpi::cmd::SubsystemBase {
 public:
  ArmSubsystem() {
    m_motorConfig.WithSubsystem(this)
        .WithFeedback(1.0, 0.0, 0.0)
        .WithStatorCurrentLimit(40.0_A)
        .WithMotorInverted(false)
        .WithVendorConfig(thrifty::NovaConfigBatch{thrifty::NovaConfig::TempThrottleEnable(true)})
        .WithClosedLoopMode()
        .WithTelemetry("ArmMotor", SmartMotorControllerConfig::TelemetryVerbosity::HIGH);

    m_motor.emplace(&m_nova, wpi::math::DCMotor::NEO(1), &m_motorConfig);
  }

 private:
  thrifty::Nova m_nova{0, 3, thrifty::Motor::NEO};
  SmartMotorControllerConfig m_motorConfig;  // must outlive the wrapper
  std::optional<NovaWrapper> m_motor;
};
```

{% hint style="info" %}
`TelemetryVerbosity::HIGH` together with `WithSubsystem(this)` automatically registers the `Live Tuning` command for the subsystem. See [SmartMotorController](smart-motor-controller.md#telemetry-and-live-tuning).
{% endhint %}

***

## See also

* [SmartMotorController](smart-motor-controller.md)
* [SmartMotorControllerConfig](smart-motor-controller-config.md)
* [SparkWrapper](spark-wrapper.md)
* [A301Wrapper](a301-wrapper.md)
* [TalonFXWrapper / TalonFXSWrapper](talonfx-wrapper.md)
* [BatterySim](../sensor/battery-sim.md)
* [Java NovaWrapper](../../java-reference/motor-controllers/nova-wrapper.md)
