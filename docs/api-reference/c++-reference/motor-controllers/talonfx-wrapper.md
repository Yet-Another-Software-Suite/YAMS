# TalonFXWrapper / TalonFXSWrapper

**Namespace:** `yams::motorcontrollers::remote`\
**Extends:** `SmartMotorController`\
**Header:** `#include <yams/motorcontrollers/remote/TalonFXWrapper.hpp>`\
**Doxygen:** [`TalonFXWrapper`](https://doxygen.yassrobotics.com/yams/classyams_1_1motorcontrollers_1_1remote_1_1TalonFXWrapper.html), [`TalonFXSWrapper`](https://doxygen.yassrobotics.com/yams/classyams_1_1motorcontrollers_1_1remote_1_1TalonFXSWrapper.html)

`SmartMotorController` implementations for CTRE TalonFX and TalonFXS motor controllers using Phoenix 6. `TalonFXWrapper` targets the TalonFX (Kraken X60, Kraken X44, Falcon 500). `TalonFXSWrapper` targets the TalonFXS, which supports third-party brushed and brushless motors such as REV NEO and Minion.

**Java equivalent:** [Java TalonFXWrapper / TalonFXSWrapper](../../java-reference/motor-controllers/talonfx-wrapper.md)

***

## TalonFXWrapper

### Constructor

```cpp
TalonFXWrapper(ctre::phoenix6::hardware::TalonFX* talon,
               wpi::math::DCMotor dcMotor,
               SmartMotorControllerConfig* config)
```

| Parameter | Type                                 | Description                                                                   |
| --------- | ------------------------------------ | ----------------------------------------------------------------------------- |
| `talon`   | `ctre::phoenix6::hardware::TalonFX*` | TalonFX hardware object. Must outlive this wrapper.                           |
| `dcMotor` | `wpi::math::DCMotor`                 | DC motor model used for simulation (e.g. `wpi::math::DCMotor::KrakenX60(1)`). |
| `config`  | `SmartMotorControllerConfig*`        | Initial configuration to apply. Must outlive this wrapper.                    |

### Key methods

The `TalonFXWrapper` implements the full [`SmartMotorController`](smart-motor-controller.md) interface. The table below lists the methods most commonly used directly.

#### Open loop outputs

| Method         | Signature                                     | Description                              |
| -------------- | --------------------------------------------- | ---------------------------------------- |
| `SetDutyCycle` | `void SetDutyCycle(double dutyCycle)`         | Command a duty cycle output in \[-1, 1]. |
| `GetDutyCycle` | `double GetDutyCycle()`                       | Returns the last commanded duty cycle.   |
| `SetVoltage`   | `void SetVoltage(wpi::units::volt_t voltage)` | Command a voltage output.                |
| `GetVoltage`   | `wpi::units::volt_t GetVoltage()`             | Returns the last commanded voltage.      |

#### Closed loop setpoints

| Method        | Signature                                                    | Description                                                                                                    |
| ------------- | ------------------------------------------------------------ | -------------------------------------------------------------------------------------------------------------- |
| `SetPosition` | `void SetPosition(wpi::units::turn_t angle)`                 | Command an angular position setpoint (closed loop).                                                            |
| `SetPosition` | `void SetPosition(wpi::units::meter_t distance)`             | Command a linear position setpoint. Converts to turns using the configured mechanism circumference.            |
| `SetVelocity` | `void SetVelocity(wpi::units::turns_per_second_t velocity)`  | Command an angular velocity setpoint (closed loop).                                                            |
| `SetVelocity` | `void SetVelocity(wpi::units::meters_per_second_t velocity)` | Command a linear velocity setpoint. Converts to turns per second using the configured mechanism circumference. |

#### Encoder reads

| Method                                | Signature                                                                             | Description                                                                                          |
| ------------------------------------- | ------------------------------------------------------------------------------------- | ---------------------------------------------------------------------------------------------------- |
| `GetMechanismPosition`                | `wpi::units::turn_t GetMechanismPosition()`                                           | Mechanism-side angular position (after gearing).                                                     |
| `GetMechanismVelocity`                | `wpi::units::turns_per_second_t GetMechanismVelocity()`                               | Mechanism-side angular velocity.                                                                     |
| `GetMechanismAcceleration`            | `wpi::units::turns_per_second_squared_t GetMechanismAcceleration()`                   | Mechanism-side angular acceleration.                                                                 |
| `GetRotorPosition`                    | `wpi::units::turn_t GetRotorPosition()`                                               | Raw rotor position (before gearing).                                                                 |
| `GetRotorVelocity`                    | `wpi::units::turns_per_second_t GetRotorVelocity()`                                   | Raw rotor velocity.                                                                                  |
| `GetMeasurementPosition`              | `wpi::units::meter_t GetMeasurementPosition()`                                        | Linear measurement position.                                                                         |
| `GetMeasurementVelocity`              | `wpi::units::meters_per_second_t GetMeasurementVelocity()`                            | Linear measurement velocity.                                                                         |
| `GetMeasurementAcceleration`          | `wpi::units::meters_per_second_squared_t GetMeasurementAcceleration()`                | Linear measurement acceleration.                                                                     |
| `GetRelativeMechanismPosition`        | `wpi::units::turn_t GetRelativeMechanismPosition()`                                   | Mechanism position from the motor's own rotor sensor, never the external encoder.                    |
| `GetRelativeMechanismVelocity`        | `wpi::units::turns_per_second_t GetRelativeMechanismVelocity()`                       | Mechanism velocity from the motor's own rotor sensor.                                                |
| `GetExternalEncoderMechanismPosition` | `std::optional<wpi::units::turn_t> GetExternalEncoderMechanismPosition()`             | External encoder position in mechanism turns, empty if no external encoder is configured.            |
| `GetExternalEncoderMechanismVelocity` | `std::optional<wpi::units::turns_per_second_t> GetExternalEncoderMechanismVelocity()` | External encoder velocity in mechanism turns per second, empty if no external encoder is configured. |

{% hint style="info" %}
`GetExternalEncoderPosition()` / `GetExternalEncoderVelocity()` no longer exist. Use `GetExternalEncoderMechanismPosition()` / `GetExternalEncoderMechanismVelocity()`.
{% endhint %}

#### Motor status

| Method             | Signature                                                | Description                    |
| ------------------ | -------------------------------------------------------- | ------------------------------ |
| `GetSupplyCurrent` | `std::optional<wpi::units::ampere_t> GetSupplyCurrent()` | Supply (battery-side) current. |
| `GetStatorCurrent` | `wpi::units::ampere_t GetStatorCurrent()`                | Stator (motor-side) current.   |
| `GetTemperature`   | `wpi::units::celsius_t GetTemperature()`                 | Motor temperature.             |

#### Live configuration setters

These methods apply changes immediately without requiring a full `ApplyConfig` call.

| Method                            | Signature                                                                     | Description                                                                                                       |
| --------------------------------- | ----------------------------------------------------------------------------- | ----------------------------------------------------------------------------------------------------------------- |
| `SetZeroPower`                    | `void SetZeroPower(MotorMode mode)`                                           | Coast or Brake.                                                                                                   |
| `SetMotorInverted`                | `void SetMotorInverted(bool inverted)`                                        | Invert motor output direction.                                                                                    |
| `SetEncoderInverted`              | `void SetEncoderInverted(bool inverted)`                                      | No effect on `TalonFXWrapper`. On `TalonFXSWrapper` it sets the external feedback sensor phase.                   |
| `SetFeedback`                     | `void SetFeedback(double kP, double kI, double kD)`                           | Update PID gains in the active slot.                                                                              |
| `SetFeedforward`                  | `void SetFeedforward(double kS, double kV, double kA, double kG)`             | Update feedforward gains in the active slot.                                                                      |
| `SetStatorCurrentLimit`           | `void SetStatorCurrentLimit(wpi::units::ampere_t limit)`                      | Cap motor stator current.                                                                                         |
| `SetSupplyCurrentLimit`           | `void SetSupplyCurrentLimit(wpi::units::ampere_t limit)`                      | Cap battery supply current.                                                                                       |
| `SetMechanismLimits`              | `void SetMechanismLimits(wpi::units::turn_t lower, wpi::units::turn_t upper)` | Update angular soft limits.                                                                                       |
| `SetMechanismLimitsEnabled`       | `void SetMechanismLimitsEnabled(bool enabled)`                                | Enable or disable angular soft limits.                                                                            |
| `SetMotionProfileMaxVelocity`     | Two overloads: `turns_per_second_t` and `meters_per_second_t`                 | Cap motion profile velocity.                                                                                      |
| `SetMotionProfileMaxAcceleration` | Two overloads: `turns_per_second_squared_t` and `meters_per_second_squared_t` | Cap motion profile acceleration.                                                                                  |
| `SetClosedLoopSlot`               | `void SetClosedLoopSlot(ClosedLoopControllerSlot slot)`                       | Select active PID/FF gain slot. TalonFX supports SLOT\_0 through SLOT\_2; SLOT\_3 throws `std::invalid_argument`. |

#### CTRE-specific methods

| Method               | Signature                                                | Description                                                                                                                                                                                                                                                 |
| -------------------- | -------------------------------------------------------- | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `EnableFOC`          | `TalonFXWrapper& EnableFOC()`                            | Enable FOC for the position and velocity control requests. Ignored by unlicensed devices. Throws `SmartMotorControllerConfigurationException` if a request is a TorqueCurrentFOC request (set through `WithVendorControlRequest`), which cannot toggle FOC. |
| `DisableFOC`         | `TalonFXWrapper& DisableFOC()`                           | Disable FOC for the position and velocity control requests. Same exception as `EnableFOC`.                                                                                                                                                                  |
| `UseCANdiPWM1`       | `bool UseCANdiPWM1() const`                              | Whether CANdi PWM1 is the feedback sensor of the configuration being applied. Throws `std::invalid_argument` if it is but no CANdi is configured as the external encoder.                                                                                   |
| `UseCANdiPWM2`       | `bool UseCANdiPWM2() const`                              | Same for CANdi PWM2.                                                                                                                                                                                                                                        |
| `SetUpdateFrequency` | `void SetUpdateFrequency(wpi::units::hertz_t frequency)` | Set the update frequency of the status signals this controller reads.                                                                                                                                                                                       |
| `ForceConfigApply`   | `ctre::phoenix::StatusCode ForceConfigApply()`           | Apply the whole configuration, retrying every 10 ms up to 10 times. Returns the status of the last attempt.                                                                                                                                                 |

`TalonFXSWrapper` has the same methods (returning `TalonFXSWrapper&` where applicable).

#### Advanced accessors

```cpp
SmartMotorControllerConfig& GetConfig();

// Returns a raw pointer to the underlying ctre::phoenix6::hardware::TalonFX.
void* GetMotorController();

// Returns a raw pointer to the ctre::phoenix6::configs::TalonFXConfiguration.
void* GetMotorControllerConfig();
```

***

## TalonFXSWrapper

**Header:** `#include <yams/motorcontrollers/remote/TalonFXSWrapper.hpp>`

### Motor arrangement enum

The TalonFXS drives third-party motors through an external motor port. You must specify which motor type is connected.

```cpp
enum class MotorArrangement {
    Minion,
    NEO,
    NEO550,
    NEOVortex,
    Brushed_2Wire,
    Brushed_3Wire
};
```

A Thrifty Pulsar 775 is driven as `MotorArrangement::NEO`.

### Constructors

```cpp
TalonFXSWrapper(ctre::phoenix6::hardware::TalonFXS* talon,
                wpi::math::DCMotor dcMotor,
                SmartMotorControllerConfig* config)

TalonFXSWrapper(ctre::phoenix6::hardware::TalonFXS* talon,
                wpi::math::DCMotor dcMotor,
                TalonFXSWrapper::MotorArrangement arrangement,
                SmartMotorControllerConfig* config)
```

| Parameter     | Type                                  | Description                                                                                              |
| ------------- | ------------------------------------- | -------------------------------------------------------------------------------------------------------- |
| `talon`       | `ctre::phoenix6::hardware::TalonFXS*` | TalonFXS hardware object. Must outlive this wrapper.                                                     |
| `dcMotor`     | `wpi::math::DCMotor`                  | DC motor model used for simulation.                                                                      |
| `arrangement` | `TalonFXSWrapper::MotorArrangement`   | The motor type connected to the TalonFXS. The three-argument overload picks it from the `dcMotor` model. |
| `config`      | `SmartMotorControllerConfig*`         | Initial configuration to apply. Must outlive this wrapper.                                               |

### Key methods

`TalonFXSWrapper` exposes the same [`SmartMotorController`](smart-motor-controller.md) interface as `TalonFXWrapper`. The sections above (open loop outputs, closed loop setpoints, encoder reads, motor status and live configuration setters) apply equally.

> **Note:** On TalonFXS, `SetEncoderInverted` sets the external feedback `SensorPhase` (opposed when inverted). On TalonFX it has no effect.

#### Advanced accessors

```cpp
SmartMotorControllerConfig& GetConfig();

// Returns a raw pointer to the underlying ctre::phoenix6::hardware::TalonFXS.
void* GetMotorController();

// Returns a raw pointer to the ctre::phoenix6::configs::TalonFXSConfiguration.
void* GetMotorControllerConfig();
```

***

## Supported hardware

| Hardware             | Wrapper Class     | Recommended `wpi::math::DCMotor` Model |
| -------------------- | ----------------- | -------------------------------------- |
| TalonFX (Kraken X60) | `TalonFXWrapper`  | `wpi::math::DCMotor::KrakenX60(1)`     |
| TalonFX (Kraken X44) | `TalonFXWrapper`  | `wpi::math::DCMotor::KrakenX44(1)`     |
| TalonFX (Falcon 500) | `TalonFXWrapper`  | `wpi::math::DCMotor::Falcon500(1)`     |
| TalonFXS + NEO       | `TalonFXSWrapper` | `wpi::math::DCMotor::NEO(1)`           |
| TalonFXS + NEO 550   | `TalonFXSWrapper` | `wpi::math::DCMotor::NEO550(1)`        |
| TalonFXS + Vortex    | `TalonFXSWrapper` | `wpi::math::DCMotor::NeoVortex(1)`     |
| TalonFXS + Minion    | `TalonFXSWrapper` | `wpi::math::DCMotor::Minion(1)`        |

***

## External encoder support

### CANcoder

Pass a pointer to the `CANcoder` via `WithExternalEncoder(&cancoder)` in the `SmartMotorControllerConfig`. YAMS configures the TalonFX to fuse the CANcoder as the remote feedback source automatically. A wrapping range is optional; when set with `WithWrapping` or `WithContinuousWrapping`, its maximum, from 0 to 1 rotation, becomes the CANcoder's `AbsoluteSensorDiscontinuityPoint` (the same for a CANdi PWM input), and any other maximum throws `SmartMotorControllerConfigurationException`. Without one, the encoder keeps its own range. Only `WithContinuousWrapping` turns on `ClosedLoopGeneral.ContinuousWrap`.

```cpp
ctre::phoenix6::hardware::CANcoder cancoder{9};

yams::motorcontrollers::SmartMotorControllerConfig config{};
config.WithSubsystem(this)
      .WithExternalEncoder(&cancoder)
      .WithUseExternalFeedbackEncoder(true)
      .WithFeedback(4.0, 0.0, 0.0)
      .WithMotorGearing(yams::gearing::MechanismGearing{
          yams::gearing::GearBox::FromReductionStages({3.0, 4.0})})
      .WithClosedLoopMode();

ctre::phoenix6::hardware::TalonFX talon{5};
yams::motorcontrollers::remote::TalonFXWrapper motor{&talon, wpi::math::DCMotor::KrakenX60(1), &config};
```

### CANdi

A `CANdi` can be passed via `WithExternalEncoder(&candi)` for both `TalonFXWrapper` and `TalonFXSWrapper`. Because a CANdi has two PWM inputs (PWM1 and PWM2), YAMS cannot determine which port the encoder is wired to automatically. You must supply a vendor config that sets the correct `FeedbackSensorSource` (TalonFX) or `ExternalFeedbackSensorSource` (TalonFXS). See the examples in [SmartMotorController](smart-motor-controller.md#candi-as-external-encoder-talonfxwrapper-talonfxswrapper).

{% hint style="warning" %}
Without a vendor config that explicitly sets the sensor source, YAMS has no way to know which of the two PWM ports the CANdi encoder is connected to and the feedback loop will not work.
{% endhint %}

***

## Code examples

### TalonFX with CANcoder

```cpp
#include <yams/motorcontrollers/remote/TalonFXWrapper.hpp>
#include <yams/motorcontrollers/SmartMotorControllerConfig.hpp>
#include <yams/gearing/GearBox.hpp>
#include <yams/gearing/MechanismGearing.hpp>
#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/CANcoder.hpp>
#include <wpi/math/system/DCMotor.hpp>

using namespace yams::motorcontrollers;
using namespace yams::motorcontrollers::remote;
using namespace yams::gearing;
using Cfg = SmartMotorControllerConfig;

// Declare as subsystem members:
//   ctre::phoenix6::hardware::TalonFX    m_talon{5};
//   ctre::phoenix6::hardware::CANcoder   m_cancoder{9};
//   SmartMotorControllerConfig           m_cfg;
//   std::optional<TalonFXWrapper>        m_smc;

// m_cfg is a subsystem member so it outlives the wrapper.
m_cfg.WithSubsystem(this)
   .WithFeedback(4.0, 0.0, 0.0)
   .WithTrapezoidProfile(wpi::units::turns_per_second_t{0.5},
                         wpi::units::turns_per_second_squared_t{0.25})
   .WithMotorGearing(MechanismGearing{GearBox::FromReductionStages({3.0, 4.0})})
   .WithZeroPower(Cfg::MotorMode::BRAKE)
   .WithStatorCurrentLimit(40.0_A)
   .WithMotorInverted(false)
   .WithFeedforward(wpi::math::ArmFeedforward{0.0_V, 0.0_V,
                                        wpi::units::unit_t<wpi::math::ArmFeedforward::kv_unit>{0.0},
                                        wpi::units::unit_t<wpi::math::ArmFeedforward::ka_unit>{0.0}})
   .WithExternalEncoder(&m_cancoder)
   .WithUseExternalFeedbackEncoder(true)
   .WithClosedLoopMode()
   .WithTelemetry("ArmMotor", Cfg::TelemetryVerbosity::HIGH);

m_smc.emplace(&m_talon, wpi::math::DCMotor::KrakenX60(1), &m_cfg);
```

### TalonFXS with CANdi

```cpp
#include <yams/motorcontrollers/remote/TalonFXSWrapper.hpp>
#include <yams/motorcontrollers/SmartMotorControllerConfig.hpp>
#include <yams/gearing/GearBox.hpp>
#include <yams/gearing/MechanismGearing.hpp>
#include <ctre/phoenix6/TalonFXS.hpp>
#include <ctre/phoenix6/CANdi.hpp>
#include <ctre/phoenix6/configs/Configs.hpp>
#include <wpi/math/system/DCMotor.hpp>

using namespace yams::motorcontrollers;
using namespace yams::motorcontrollers::remote;
using namespace yams::gearing;
using Cfg = SmartMotorControllerConfig;

// Declare as subsystem members:
//   ctre::phoenix6::hardware::TalonFXS  m_talonFXS{6};
//   ctre::phoenix6::hardware::CANdi     m_candi{10};
//   SmartMotorControllerConfig          m_cfg;
//   std::optional<TalonFXSWrapper>      m_smc;

// Configure the TalonFXS to use the correct CANdi PWM port.
ctre::phoenix6::configs::TalonFXSConfiguration talonSConfig{};
talonSConfig.ExternalFeedback.ExternalFeedbackSensorSource =
    ctre::phoenix6::signals::ExternalFeedbackSensorSourceValue::SyncCANdiPWM1;
talonSConfig.ExternalFeedback.ExternalFeedbackRemoteSensorID = m_candi.GetDeviceID();

// m_cfg is a subsystem member so it outlives the wrapper.
m_cfg.WithSubsystem(this)
   .WithFeedback(4.0, 0.0, 0.0)
   .WithTrapezoidProfile(wpi::units::turns_per_second_t{0.5},
                         wpi::units::turns_per_second_squared_t{0.25})
   .WithMotorGearing(MechanismGearing{GearBox::FromReductionStages({3.0, 4.0})})
   .WithZeroPower(Cfg::MotorMode::BRAKE)
   .WithStatorCurrentLimit(40.0_A)
   .WithMotorInverted(false)
   .WithVendorConfig(talonSConfig)
   .WithExternalEncoder(&m_candi)
   .WithUseExternalFeedbackEncoder(true)
   .WithClosedLoopMode()
   .WithTelemetry("HoodMotor", Cfg::TelemetryVerbosity::HIGH);

m_smc.emplace(&m_talonFXS, wpi::math::DCMotor::NEO(1),
              TalonFXSWrapper::MotorArrangement::NEO, &m_cfg);
```

***

## Notes

**Phoenix 6 required.** YAMS only supports Phoenix 6 for TalonFX and TalonFXS. Phoenix 5 is not supported.

**Motion Magic.** When a trapezoidal or exponential profile is configured via `WithTrapezoidProfile` or `WithExponentialProfile`, YAMS uses CTRE Motion Magic control requests internally (`MotionMagicVoltage`, `MotionMagicExpoVoltage`, or the appropriate FOC variant). Phoenix 6 has no [LQR](../math/lqr-controller.md), so when an LQR is configured the closed loop runs on the SystemCore instead and an alert reports it.

**FOC support.** Both wrappers support Field-Oriented Control for Kraken X60 and other FOC-capable motors. Call `EnableFOC()` / `DisableFOC()` on the wrapper to toggle FOC for the position and velocity control requests. Unlicensed devices ignore it (CTRE requires a Phoenix Pro license for FOC).

**Configuration retries.** `TalonFXConfiguration` and `TalonFXSConfiguration` are built from `SmartMotorControllerConfig` and applied via the Phoenix 6 configurator. YAMS retries configuration on failure (every 10 ms, up to 10 times); `ForceConfigApply()` re-applies the whole configuration and returns the last status.

**Gain slots.** Both wrappers support three closed loop gain slots (`SLOT_0` through `SLOT_2`). Attempting to activate `SLOT_3` throws `std::invalid_argument`.

**Simulation.** Both wrappers use `wpi::sim::DCMotorSim` with the configured moment of inertia (`SmartMotorControllerConfig::WithMOI(...)`) and report their current draw to [`BatterySim`](../sensor/battery-sim.md). The constructor calls `SetupSimulation()` for you, and the YAMS mechanisms call `SimIterate()` from their simulation loop. Call `SimIterate()` yourself only when using the wrapper without a mechanism. The simulated TalonFX models a Kraken X44 when the `DCMotor` you pass is `wpi::math::DCMotor::KrakenX44(n)` or `wpi::math::DCMotor::KrakenX44FOC(n)`, for one to four motors, and a Kraken X60 otherwise.

**Telemetry and live tuning.** Both wrappers support every telemetry field. `TelemetryVerbosity::HIGH` together with `WithSubsystem(this)` registers the `Live Tuning` command automatically; see [SmartMotorController](smart-motor-controller.md#telemetry-and-live-tuning).

**Alerts.** Both wrappers raise alerts in the `YAMS` group: `closed loop controller is running on the SystemCore` ([LQR](../math/lqr-controller.md) configured), `starting position is not applied because an external encoder is used`, and `zero offset is not supported without an external encoder`.

**No rotor offset.** `WithExternalEncoderZeroOffset` applies to the external encoder only. The TalonFX rotor is not offset; without an external encoder a zero offset raises the alert above.

**Lifetime requirements.** The TalonFX/TalonFXS hardware object and the `SmartMotorControllerConfig` must both outlive the wrapper. The recommended pattern is to declare all three as subsystem members (or use `std::optional` for the wrapper) so that construction order and lifetimes are managed correctly by the subsystem.

***

## See also

* [SmartMotorController](smart-motor-controller.md)
* [SmartMotorControllerConfig](smart-motor-controller-config.md)
* [SparkWrapper](spark-wrapper.md)
* [NovaWrapper](nova-wrapper.md)
* [A301Wrapper](a301-wrapper.md)
* [Java TalonFXWrapper / TalonFXSWrapper](../../java-reference/motor-controllers/talonfx-wrapper.md)
