# SparkWrapper

**Namespace:** `yams::motorcontrollers::local`\
**Extends:** `SmartMotorController`\
**Header:** `#include <yams/motorcontrollers/local/SparkWrapper.hpp>`\
**Doxygen:** [`SparkWrapper`](https://doxygen.yassrobotics.com/yams/classyams_1_1motorcontrollers_1_1local_1_1SparkWrapper.html)

`SmartMotorController` implementation for REV SPARK MAX and SPARK Flex motor controllers using REVLib 2027. One class covers both controllers; the constructor overload selects the hardware.

**Java equivalent:** [Java SparkWrapper](../../java-reference/motor-controllers/spark-wrapper.md)

***

## Constructors

```cpp
SparkWrapper(rev::spark::SparkMax* spark, wpi::math::DCMotor motor,
             SmartMotorControllerConfig* config)
SparkWrapper(rev::spark::SparkFlex* spark, wpi::math::DCMotor motor,
             SmartMotorControllerConfig* config)
```

| Parameter | Type                                               | Description                                                             |
| --------- | -------------------------------------------------- | ----------------------------------------------------------------------- |
| `spark`   | `rev::spark::SparkMax*` / `rev::spark::SparkFlex*` | SPARK hardware object. Must outlive this wrapper.                       |
| `motor`   | `wpi::math::DCMotor`                               | DC motor model used for simulation (e.g. `wpi::math::DCMotor::NEO(1)`). |
| `config`  | `SmartMotorControllerConfig*`                      | Configuration to apply. Must outlive this wrapper.                      |

The constructor applies the config, sets up simulation (in simulation only) and, when a telemetry verbosity is configured, publishes telemetry on the next `UpdateTelemetry()` call.

A vendor config passed with `WithVendorConfig` must be a `rev::spark::SparkMaxConfig` for a SPARK MAX or a `rev::spark::SparkFlexConfig` for a SPARK Flex. Any other type throws `SmartMotorControllerConfigurationException`.

***

## Key methods

`SparkWrapper` implements the full [`SmartMotorController`](smart-motor-controller.md) interface (open loop outputs, closed loop setpoints, encoder reads and writes, live configuration setters, telemetry and live tuning). See that page for the complete method list. The notes below cover SPARK-specific behavior.

### Units on the SPARK

The SPARK runs in the raw units of its feedback sensor (rotations and RPM of the motor's encoder, or of the external encoder when it is used for feedback). YAMS does not configure conversion factors on the SPARK. Gains, setpoints, MAXMotion constraints, tolerances and soft limits are scaled from YAMS mechanism units by the mechanism-to-feedback-sensor ratio, so you always work in mechanism units.

### Closed loop control

| Configuration                                           | Where the loop runs                                                                                                                                 |
| ------------------------------------------------------- | --------------------------------------------------------------------------------------------------------------------------------------------------- |
| PID, with or without a feedforward                      | On the SPARK.                                                                                                                                       |
| Trapezoidal profile                                     | On the SPARK, using MAXMotion position or velocity control.                                                                                         |
| Exponential profile or [LQR](../math/lqr-controller.md) | On the SystemCore. YAMS runs the loop in a notifier and sends voltages to the SPARK. An alert reports this (see [Alerts](spark-wrapper.md#alerts)). |

`WithClosedLoopControlPeriod`, `WithClosedLoopMaxVoltage` and `WithFeedbackSynchronizationThreshold` are only valid when the loop runs on the SystemCore (exponential profile or [LQR](../math/lqr-controller.md)); otherwise `ApplyConfig` throws `SmartMotorControllerConfigurationException`.

### Encoder reads

| Method                                                                            | SPARK behavior                                                                                               |
| --------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------ |
| `GetMechanismPosition()` / `GetMechanismVelocity()`                               | External encoder when `WithUseExternalFeedbackEncoder(true)` is set, otherwise the SPARK's relative encoder. |
| `GetRelativeMechanismPosition()` / `GetRelativeMechanismVelocity()`               | Always the SPARK's relative encoder, with the motor gearing applied.                                         |
| `GetExternalEncoderMechanismPosition()` / `GetExternalEncoderMechanismVelocity()` | The external encoder with the external encoder gearing applied. Empty when no external encoder is attached.  |
| `GetRotorPosition()` / `GetRotorVelocity()`                                       | Raw rotor position and velocity (before gearing).                                                            |

### Unsupported options

| Option                                             | Behavior                                                                                                                                                                                                    |
| -------------------------------------------------- | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `WithSupplyCurrentLimit` / `SetSupplyCurrentLimit` | SPARKs have no supply current limit. `ApplyConfig` throws a `SmartMotorControllerConfigurationException` for `WithSupplyCurrentLimit` (use `WithStatorCurrentLimit`); `SetSupplyCurrentLimit` does nothing. |
| `GetSupplyCurrent()`                               | Always returns `std::nullopt`. The supply current telemetry fields are not published.                                                                                                                       |
| `WithEncoderInverted(true)`                        | The SPARK's relative encoder cannot be inverted. Throws `std::invalid_argument`.                                                                                                                            |
| `WithVendorControlRequest`                         | Not supported on SPARKs. Throws `SmartMotorControllerConfigurationException`.                                                                                                                               |
| `SetEncoderVelocity` (on a real robot)             | Throws `std::runtime_error`; REV SPARKs cannot have their encoder velocity set.                                                                                                                             |

***

## External encoders

Pass a pointer to the encoder with `WithExternalEncoder(...)`:

| Encoder type                                                                      | Notes                                                                                                       |
| --------------------------------------------------------------------------------- | ----------------------------------------------------------------------------------------------------------- |
| `rev::spark::SparkAbsoluteEncoder*`                                               | Absolute encoder on the SPARK data port. Supports zero offset and a wrapping range.                         |
| `rev::spark::SparkMaxAlternateEncoder*` / `rev::spark::SparkFlexExternalEncoder*` | Quadrature encoder. Zero offset is not available and throws a `SmartMotorControllerConfigurationException`. |
| `rev::detached::DetachedEncoder*` / `rev::detached::SplineEncoder*`               | Encoder read over CAN.                                                                                      |

An absolute or detached encoder reads `[0, 1)` unless a wrapping range is set with `WithWrapping(min, max)` or `WithContinuousWrapping(min, max)`. The SPARK absolute encoder accepts any one rotation from `[-1, 0)` to `[0, 1)`, sent as REVLib's range offset, the middle of the range `(min + max) / 2`. A detached encoder accepts `[-0.5, 0.5)` (zero centered) or `[0, 1)`. Any other range throws `SmartMotorControllerConfigurationException`. Whenever wrapping is set, the SPARK's `positionWrappingInputRange` is the wrapping range; `positionWrappingEnabled` is only turned on by `WithContinuousWrapping`.

`WithExternalEncoderZeroOffset`, `WithExternalEncoderInverted` and `WithExternalEncoderGearing` all require `WithExternalEncoder(...)`; setting them without one throws `SmartMotorControllerConfigurationException`.

***

## Followers

`WithFollowers` accepts `rev::spark::SparkMax*` and `rev::spark::SparkFlex*` followers. Any other type throws `std::invalid_argument`. Use `WithLooselyCoupledFollowers` to forward setpoints to other `SmartMotorController` instances instead.

***

## Simulation

In simulation the wrapper drives a `rev::spark::SparkSim` together with a YAMS sim supplier, and reports its current draw to [`BatterySim`](../sensor/battery-sim.md) every loop. The simulated SPARK closed loop is stepped at 1 kHz within each simulation period, matching the 1 ms loop of the real controller. REVLib's MAXMotion simulation does not follow the profile, so for a position trapezoidal profile YAMS runs the profile in `SimIterate()` and holds each point with position control. Set a realistic moment of inertia with `SmartMotorControllerConfig::WithMOI(...)` for realistic current draw.

***

## Alerts

Alerts are published in the `YAMS` alert group and cleared when the wrapper is destroyed.

| Alert                                                         | When                                                                                                          |
| ------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------- |
| `<name> closed loop controller is running on the SystemCore.` | An exponential profile or [LQR](../math/lqr-controller.md) is configured, so the loop runs on the SystemCore. |

***

## Example

```cpp
#include <rev/SparkMax.h>
#include <wpi/commands2/SubsystemBase.hpp>
#include <yams/gearing/MechanismGearing.hpp>
#include <yams/mechanisms/config/ElevatorConfig.hpp>
#include <yams/mechanisms/positional/Elevator.hpp>
#include <yams/motorcontrollers/SmartMotorControllerConfig.hpp>
#include <yams/motorcontrollers/local/SparkWrapper.hpp>

using namespace yams::gearing;
using namespace yams::motorcontrollers;
using namespace yams::motorcontrollers::local;
using Cfg = SmartMotorControllerConfig;

class ElevatorSubsystem : public wpi::cmd::SubsystemBase {
 public:
  ElevatorSubsystem() {
    m_motorConfig.WithSubsystem(this)
        .WithFeedback(1.0, 0.0, 0.0)
        .WithMechanismCircumference(0.25_in, 22)
        .WithMotorGearing(MechanismGearing{GearBox::FromReductionStages({3.0, 4.0})})
        .WithZeroPower(Cfg::MotorMode::BRAKE)
        .WithStatorCurrentLimit(40.0_A)
        .WithMotorInverted(false)
        .WithClosedLoopMode()
        .WithTelemetry("ElevatorMotor", Cfg::TelemetryVerbosity::HIGH);

    m_motor.emplace(&m_spark, wpi::math::DCMotor::NEO(1), &m_motorConfig);
  }

 private:
  rev::spark::SparkMax m_spark{3, rev::spark::SparkLowLevel::MotorType::kBrushless};
  SmartMotorControllerConfig m_motorConfig;  // must outlive the wrapper
  std::optional<SparkWrapper> m_motor;
};
```

For a SPARK Flex, pass a `rev::spark::SparkFlex*` and the matching motor model:

```cpp
rev::spark::SparkFlex m_sparkFlex{4, rev::spark::SparkLowLevel::MotorType::kBrushless};

m_motor.emplace(&m_sparkFlex, wpi::math::DCMotor::NeoVortex(1), &m_motorConfig);
```

{% hint style="info" %}
`TelemetryVerbosity::HIGH` together with `WithSubsystem(this)` automatically registers the `Live Tuning` command for the subsystem. See [SmartMotorController](smart-motor-controller.md#telemetry-and-live-tuning).
{% endhint %}

***

## See also

* [SmartMotorController](smart-motor-controller.md)
* [SmartMotorControllerConfig](smart-motor-controller-config.md)
* [NovaWrapper](nova-wrapper.md)
* [A301Wrapper](a301-wrapper.md)
* [TalonFXWrapper / TalonFXSWrapper](talonfx-wrapper.md)
* [BatterySim](../sensor/battery-sim.md)
* [Java SparkWrapper](../../java-reference/motor-controllers/spark-wrapper.md)
