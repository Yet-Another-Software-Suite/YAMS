# SmartMotorController

**Namespace:** `yams::motorcontrollers`\
**Header:** `#include <yams/motorcontrollers/SmartMotorController.hpp>`\
**Doxygen:** [`SmartMotorController`](https://doxygen.yassrobotics.com/yams/classyams_1_1motorcontrollers_1_1SmartMotorController.html)

`SmartMotorController` is the abstract base class for all motor controller implementations. Do not instantiate this class directly. Construct concrete instances using vendor-specific subclass constructors such as [`SparkWrapper`](spark-wrapper.md), [`NovaWrapper`](nova-wrapper.md), [`A301Wrapper`](a301-wrapper.md), [`TalonFXWrapper` or `TalonFXSWrapper`](talonfx-wrapper.md).

All quantities use the WPILib 2027 C++ units library in the `wpi::units` namespace (for example `wpi::units::turn_t`), and motor models use `wpi::math::DCMotor`.

**Java equivalent:** [Java SmartMotorController](../../java-reference/motor-controllers/smart-motor-controller.md)

***

## Type aliases

```cpp
using ClosedLoopControllerSlot = SmartMotorControllerConfig::ClosedLoopControllerSlot;
using MotorMode = SmartMotorControllerConfig::MotorMode;
using ControlMode = SmartMotorControllerConfig::ControlMode;
```

***

## Pure virtual methods

Subclasses must implement all of the following methods.

### Configuration

```cpp
virtual bool ApplyConfig(const SmartMotorControllerConfig& config) = 0;
```

### Simulation

```cpp
virtual void SetupSimulation() = 0;
virtual void SimIterate() = 0;
```

The bundled wrappers call `SetupSimulation()` from their constructors (it does nothing outside simulation), and the YAMS mechanisms call `SimIterate()` from their simulation loop.

### Encoder sync

```cpp
virtual void SeedRelativeEncoder() = 0;
virtual void SynchronizeRelativeEncoder() = 0;
```

### Open loop outputs

```cpp
virtual void SetDutyCycle(double dutyCycle) = 0;
virtual double GetDutyCycle() = 0;
virtual void SetVoltage(wpi::units::volt_t voltage) = 0;
virtual wpi::units::volt_t GetVoltage() = 0;
```

### Closed loop setpoints

```cpp
virtual void SetPosition(wpi::units::turn_t angle) = 0;
virtual void SetPosition(wpi::units::meter_t distance) = 0;
virtual void SetVelocity(wpi::units::turns_per_second_t velocity) = 0;
virtual void SetVelocity(wpi::units::meters_per_second_t velocity) = 0;

// Velocity setpoint plus an extra feedforward force at the mechanism (for example from a
// path-following setpoint generator). The force is converted to a voltage with
// SmartMotorControllerConfig::ConvertToVoltage, so the mechanism circumference must be set.
virtual void SetVelocity(wpi::units::turns_per_second_t velocity,
                         wpi::units::newton_t feedforwardForce);
void SetVelocity(wpi::units::meters_per_second_t velocity, wpi::units::newton_t feedforwardForce);
```

### Encoder writes

```cpp
virtual void SetEncoderPosition(wpi::units::turn_t angle) = 0;
virtual void SetEncoderPosition(wpi::units::meter_t distance) = 0;
virtual void SetEncoderVelocity(wpi::units::turns_per_second_t velocity) = 0;
virtual void SetEncoderVelocity(wpi::units::meters_per_second_t velocity) = 0;
```

### Encoder reads

```cpp
virtual wpi::units::turn_t GetMechanismPosition() = 0;
virtual wpi::units::turns_per_second_t GetMechanismVelocity() = 0;
virtual wpi::units::turn_t GetRelativeMechanismPosition() = 0;
virtual wpi::units::turns_per_second_t GetRelativeMechanismVelocity() = 0;
virtual wpi::units::turns_per_second_squared_t GetMechanismAcceleration() = 0;
virtual wpi::units::turn_t GetRotorPosition() = 0;
virtual wpi::units::turns_per_second_t GetRotorVelocity() = 0;
virtual wpi::units::meter_t GetMeasurementPosition() = 0;
virtual wpi::units::meters_per_second_t GetMeasurementVelocity() = 0;
virtual wpi::units::meters_per_second_squared_t GetMeasurementAcceleration() = 0;
virtual std::optional<wpi::units::turn_t> GetExternalEncoderMechanismPosition() = 0;
virtual std::optional<wpi::units::turns_per_second_t> GetExternalEncoderMechanismVelocity() = 0;
```

| Methods                                                                       | Reads from                                                                                                                                                  |
| ----------------------------------------------------------------------------- | ----------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `GetMechanismPosition` / `GetMechanismVelocity`                               | The external (absolute) encoder when `WithUseExternalFeedbackEncoder(true)` is configured, otherwise the relative encoder in the motor. Gearing is applied. |
| `GetRelativeMechanismPosition` / `GetRelativeMechanismVelocity`               | Always the relative encoder in the motor, never the external encoder. Gearing is applied.                                                                   |
| `GetExternalEncoderMechanismPosition` / `GetExternalEncoderMechanismVelocity` | The external encoder with the external encoder gearing applied, whether or not it is used for feedback. Empty when no external encoder is attached.         |

{% hint style="info" %}
`GetExternalEncoderPosition()` / `GetExternalEncoderVelocity()` (which returned `degree_t` / `degrees_per_second_t`) no longer exist. Use `GetExternalEncoderMechanismPosition()` / `GetExternalEncoderMechanismVelocity()`, which return mechanism turns and turns per second.
{% endhint %}

### Motor status

```cpp
virtual std::optional<wpi::units::ampere_t> GetSupplyCurrent() = 0;
virtual wpi::units::ampere_t GetStatorCurrent() = 0;
virtual wpi::units::celsius_t GetTemperature() = 0;
virtual wpi::math::DCMotor GetDCMotor() = 0;
```

### Live configuration setters

These apply a change immediately without a full `ApplyConfig` call. They are also what live tuning calls.

| Method                            | Signature                                                                                                                                | Description                                                           |
| --------------------------------- | ---------------------------------------------------------------------------------------------------------------------------------------- | --------------------------------------------------------------------- |
| `SetZeroPower`                    | `virtual void SetZeroPower(MotorMode mode) = 0`                                                                                          | Coast or Brake                                                        |
| `SetMotorInverted`                | `virtual void SetMotorInverted(bool inverted) = 0`                                                                                       | Invert motor direction                                                |
| `SetEncoderInverted`              | `virtual void SetEncoderInverted(bool inverted) = 0`                                                                                     | Invert encoder direction                                              |
| `SetKp`                           | `virtual void SetKp(double kP) = 0`                                                                                                      | Proportional PID gain                                                 |
| `SetKi`                           | `virtual void SetKi(double kI) = 0`                                                                                                      | Integral PID gain                                                     |
| `SetKd`                           | `virtual void SetKd(double kD) = 0`                                                                                                      | Derivative PID gain                                                   |
| `SetFeedback`                     | `virtual void SetFeedback(double kP, double kI, double kD) = 0`                                                                          | Set all PID gains at once                                             |
| `SetKs`                           | `virtual void SetKs(double kS) = 0`                                                                                                      | Static feedforward gain                                               |
| `SetKv`                           | `virtual void SetKv(double kV) = 0`                                                                                                      | Velocity feedforward gain                                             |
| `SetKa`                           | `virtual void SetKa(double kA) = 0`                                                                                                      | Acceleration feedforward gain                                         |
| `SetKg`                           | `virtual void SetKg(double kG) = 0`                                                                                                      | Gravity feedforward gain                                              |
| `SetFeedforward`                  | `virtual void SetFeedforward(double kS, double kV, double kA, double kG) = 0`                                                            | Set all feedforward gains at once                                     |
| `SetStatorCurrentLimit`           | `virtual void SetStatorCurrentLimit(wpi::units::ampere_t currentLimit) = 0`                                                              | Cap motor output current                                              |
| `SetSupplyCurrentLimit`           | `virtual void SetSupplyCurrentLimit(wpi::units::ampere_t currentLimit) = 0`                                                              | Cap battery draw current                                              |
| `SetClosedLoopRampRate`           | `virtual void SetClosedLoopRampRate(wpi::units::second_t rampRate) = 0`                                                                  | Ramp time for closed loop output                                      |
| `SetOpenLoopRampRate`             | `virtual void SetOpenLoopRampRate(wpi::units::second_t rampRate) = 0`                                                                    | Ramp time for open loop output                                        |
| `SetMechanismUpperLimit`          | `virtual void SetMechanismUpperLimit(wpi::units::turn_t upperLimit) = 0`                                                                 | Update the upper angular soft limit                                   |
| `SetMechanismLowerLimit`          | `virtual void SetMechanismLowerLimit(wpi::units::turn_t lowerLimit) = 0`                                                                 | Update the lower angular soft limit                                   |
| `SetMechanismLimits`              | `virtual void SetMechanismLimits(wpi::units::turn_t lower, wpi::units::turn_t upper) = 0`                                                | Update both angular soft limits                                       |
| `SetMechanismLimitsEnabled`       | `virtual void SetMechanismLimitsEnabled(bool enabled) = 0`                                                                               | Enable or disable soft limits                                         |
| `SetMeasurementUpperLimit`        | `virtual void SetMeasurementUpperLimit(wpi::units::meter_t upperLimit) = 0`                                                              | Update the upper linear soft limit                                    |
| `SetMeasurementLowerLimit`        | `virtual void SetMeasurementLowerLimit(wpi::units::meter_t lowerLimit) = 0`                                                              | Update the lower linear soft limit                                    |
| `SetMotionProfileMaxVelocity`     | Two overloads: `wpi::units::turns_per_second_t` and `wpi::units::meters_per_second_t`                                                    | Cap motion profile velocity                                           |
| `SetMotionProfileMaxAcceleration` | Two overloads: `wpi::units::turns_per_second_squared_t` and `wpi::units::meters_per_second_squared_t`                                    | Cap motion profile acceleration                                       |
| `SetMotionProfileMaxJerk`         | `virtual void SetMotionProfileMaxJerk(wpi::units::angular_jerk::turns_per_second_cubed_t maxJerk) = 0`                                   | Cap the jerk of a velocity trapezoidal profile                        |
| `SetExponentialProfile`           | `virtual void SetExponentialProfile(std::optional<double> kV, std::optional<double> kA, std::optional<wpi::units::volt_t> maxInput) = 0` | Update the exponential profile; empty values keep the configured ones |
| `SetClosedLoopSlot`               | `virtual void SetClosedLoopSlot(ClosedLoopControllerSlot slot) = 0`                                                                      | Select active PID/FF gain slot                                        |
| `SetMechanismGearing`             | `virtual void SetMechanismGearing(const gearing::MechanismGearing& gearing)`                                                             | Update the gearing; wrappers rewrite the values that depend on it     |
| `SetMechanismCircumference`       | `virtual void SetMechanismCircumference(wpi::units::meter_t circumference)`                                                              | Update the mechanism circumference                                    |

***

## Non-virtual public methods

### Simulation

```cpp
void SetSimSupplier(std::shared_ptr<SimSupplier> supplier);
SimSupplier* GetSimSupplier() const;
void SetMechanismSimSupplier(std::shared_ptr<SimSupplier> supplier);
wpi::math::DCMotor GetMechanismDCMotor();
double GetMechanismDutyCycle(double dutyCycle);
```

`GetMechanismDCMotor()` is the `DCMotor` of every motor on the mechanism: `GetDCMotor()` and each loosely coupled follower's, with a follower's torque given at this motor's rotor. The mechanism classes simulate it, and attach their simulation with `SetMechanismSimSupplier`, which each loosely coupled follower then reads; its current is not counted again by `BatterySim`.

`GetMechanismDutyCycle(dutyCycle)` is the duty cycle a mechanism's simulation applies: each motor controller's own duty cycle, as its vendor simulation limits it, weighted by its share of the torque. The leader's `SimIterate()`, which the mechanism calls right after it steps, iterates its followers, and a follower's own `SimIterate()` does nothing while it reads a mechanism's simulation. Without a mechanism class, each follower simulates its own motor and load, and you call its `SimIterate()`.

### Closed loop controller thread

Used when the closed loop runs on the SystemCore instead of inside the motor controller (for example an [LQR](../math/lqr-controller.md), an exponential profile on a SPARK or Thrifty Nova, or any closed loop on an [A301](a301-wrapper.md)).

```cpp
void StartClosedLoopController();
void StopClosedLoopController();
bool IsClosedLoopControllerStopped() const;
void IterateClosedLoopController();
wpi::units::second_t GetClosedLoopControllerPeriod() const;
math::LQRController* GetLQRController();
```

`StartClosedLoopController()` only starts a stopped closed loop, from where the mechanism is now, and the loosely coupled followers'. One that is already running carries on, so calling it every loop is safe. The mechanism classes' position and velocity setpoints call it for you, and their voltage and duty cycle setpoints call `StopClosedLoopController()`.

While the robot is disabled, `IterateClosedLoopController()` sends no voltage. It follows the mechanism instead, resetting its PID, motion profile and LQR to where the mechanism is, so enabling starts from there rather than from where the mechanism was. Without a setpoint, the motion profile also follows the mechanism, so the next setpoint is profiled from where it is.

`IsClosedLoopControllerStopped()` is whether a closed loop run on the SystemCore was stopped, and is `false` when the closed loop runs on the motor controller. `GetLQRController()` returns the [LQR](../math/lqr-controller.md) this motor controller runs, which live tuning retunes, or `nullptr` without one.

`GetClosedLoopControllerPeriod()` is how often the closed loop runs: `WithClosedLoopControlPeriod(...)`, 20 ms by default, when it runs on the SystemCore, or 1 ms when it runs on the motor controller. It is published as `closedloop/period`.

### Telemetry and live tuning

```cpp
void SetupTelemetry(std::shared_ptr<wpi::nt::NetworkTable> dataTable,
                    std::shared_ptr<wpi::nt::NetworkTable> tuningTable);
void SetupTelemetry();   // Uses the "Mechanisms" and "Tuning" tables
void UpdateTelemetry();
bool TuningEnabled() const;
void ApplyTuningValues();
void AddCloseHook(std::function<void()> hook);
SmartMotorController& WithTelemetry(telemetry::SmartMotorControllerTelemetryConfig config);
virtual telemetry::UnsupportedTelemetryFields GetUnsupportedTelemetryFields();
```

* `UpdateTelemetry()` publishes the current readings and setpoints. It does nothing unless a telemetry verbosity is configured, and sets telemetry up on first use. It does not apply live-tuned values.
* `TuningEnabled()` reports whether tunable values are published.
* `ApplyTuningValues()` applies the values from the Tuning table to this motor controller. It throws `SmartMotorControllerConfigurationException` unless the control mode is `CLOSED_LOOP`.
* `AddCloseHook(...)` registers a callable that runs once on `Close()`.
* `GetUnsupportedTelemetryFields()` lists fields the hardware cannot report (for example SPARKs report no supply current), so they are not published.

**Live tuning.** When the config uses `TelemetryVerbosity::HIGH` and has a subsystem (`WithSubsystem(this)`), `SetupTelemetry()` registers a `Live Tuning` command for that subsystem through `SmartMotorControllerCommandRegistry` (`yams/motorcontrollers/SmartMotorControllerCommandRegistry.hpp`). There is one command per subsystem, shared by all of its motor controllers, published to the `Tuning/<SubsystemName>/Live Tuning` tunable. While it runs it calls `ApplyTuningValues()` every loop. Each subsystem instance needs a unique name, and the registration is removed when the motor controller is closed.

### Accessors

```cpp
ClosedLoopControllerSlot GetClosedLoopControllerSlot() const;
std::optional<wpi::units::turn_t> GetMechanismPositionSetpoint() const;
std::optional<wpi::units::turns_per_second_t> GetMechanismSetpointVelocity() const;
std::optional<wpi::units::meter_t> GetMeasurementPositionSetpoint() const;
std::optional<wpi::units::meters_per_second_t> GetMeasurementSetpointVelocity() const;
std::optional<wpi::units::newton_t> GetSetpointFeedforwardForce() const;
std::string GetName() const;
bool IsMotor(const wpi::math::DCMotor& a, const wpi::math::DCMotor& b) const;
void CheckConfigSafety();
void Close();   // Stops the closed-loop thread, closes telemetry, leaves BatterySim, runs close hooks
virtual SmartMotorControllerConfig& GetConfig() = 0;
virtual void* GetMotorController() = 0;
virtual void* GetMotorControllerConfig() = 0;
```

***

## Alerts

The wrappers raise WPILib alerts (`wpi::util::Alert`, group `YAMS`) for configuration problems. The alerts are cleared when the wrapper is destroyed.

| Alert                                                                | Level  | Wrappers                             |
| -------------------------------------------------------------------- | ------ | ------------------------------------ |
| `<name> closed loop controller is running on the SystemCore.`        | MEDIUM | TalonFX, TalonFXS, SPARK, Nova, A301 |
| Starting position is not applied because an external encoder is used | HIGH   | TalonFX, TalonFXS                    |
| Zero offset is not supported without an external encoder             | HIGH   | TalonFX, TalonFXS                    |

***

## SimSupplier interface

`SimSupplier` (`#include <yams/motorcontrollers/SimSupplier.hpp>`) is a pure virtual interface for physics simulation backends. Implement this interface to provide custom physics, or use one of the three built-in implementations in `yams::motorcontrollers::simulation`:

| Implementation                    | Header                                                     | Description                |
| --------------------------------- | ---------------------------------------------------------- | -------------------------- |
| `simulation::DCMotorSimSupplier`  | `yams/motorcontrollers/simulation/DCMotorSimSupplier.hpp`  | Generic DC motor physics   |
| `simulation::ArmSimSupplier`      | `yams/motorcontrollers/simulation/ArmSimSupplier.hpp`      | Single-jointed arm physics |
| `simulation::ElevatorSimSupplier` | `yams/motorcontrollers/simulation/ElevatorSimSupplier.hpp` | Linear elevator physics    |

While the robot is disabled, these three apply 0 V to the simulated motor on every step, so a mechanism falls or coasts as an unpowered one would, instead of holding its last voltage.

Each wrapper (`SparkWrapper`, `NovaWrapper`, `A301Wrapper`, `TalonFXWrapper`, `TalonFXSWrapper`) reports its simulated current draw into [`BatterySim`](../sensor/battery-sim.md) every loop, using its own instance address as a stable identity. `BatterySim` combines the draw of every registered mechanism into one shared, loaded battery voltage, which the mechanisms write with `wpi::sim::RoboRioSim::SetVInVoltage(...)`.

***

## CTRE-specific notes

### CANdi as external encoder (TalonFXWrapper / TalonFXSWrapper)

A `CANdi` can be passed via `WithExternalEncoder(&candi)` for both `TalonFXWrapper` and `TalonFXSWrapper` (external encoders are passed as pointers). Because a CANdi has two PWM inputs (PWM1 and PWM2), YAMS cannot determine which port the encoder is wired to automatically. You must pass a vendor config that sets the sensor source to one of `SyncCANdiPWM1`, `RemoteCANdiPWM1`, `SyncCANdiPWM2` or `RemoteCANdiPWM2`; otherwise offset, inversion and wrapping settings are not applied to either channel.

**TalonFXWrapper + CANdi:**

```cpp
ctre::phoenix6::configs::TalonFXConfiguration talonConfig{};
// Choose SyncCANdiPWM1 or SyncCANdiPWM2 depending on which port the encoder is wired to.
talonConfig.Feedback.FeedbackSensorSource =
    ctre::phoenix6::signals::FeedbackSensorSourceValue::SyncCANdiPWM1;
talonConfig.Feedback.FeedbackRemoteSensorID = candi.GetDeviceID();

yams::motorcontrollers::SmartMotorControllerConfig config{};
config.WithVendorConfig(talonConfig)
      .WithExternalEncoder(&candi)
      .WithUseExternalFeedbackEncoder(true)
      .WithMotorGearing(10.0);

yams::motorcontrollers::remote::TalonFXWrapper motor{&talon, wpi::math::DCMotor::KrakenX60(1),
                                                     &config};
```

**TalonFXSWrapper + CANdi:**

```cpp
ctre::phoenix6::configs::TalonFXSConfiguration talonSConfig{};
// Choose SyncCANdiPWM1 or SyncCANdiPWM2 depending on which port the encoder is wired to.
talonSConfig.ExternalFeedback.ExternalFeedbackSensorSource =
    ctre::phoenix6::signals::ExternalFeedbackSensorSourceValue::SyncCANdiPWM1;
talonSConfig.ExternalFeedback.ExternalFeedbackRemoteSensorID = candi.GetDeviceID();

yams::motorcontrollers::SmartMotorControllerConfig config{};
config.WithVendorConfig(talonSConfig)
      .WithExternalEncoder(&candi)
      .WithUseExternalFeedbackEncoder(true)
      .WithMotorGearing(10.0);

yams::motorcontrollers::remote::TalonFXSWrapper motor{
    &talonfxs, wpi::math::DCMotor::NEO(1),
    yams::motorcontrollers::remote::TalonFXSWrapper::MotorArrangement::NEO, &config};
```

{% hint style="warning" %}
Without a vendor config that explicitly sets the sensor source, YAMS has no way to know which of the two PWM ports the CANdi encoder is connected to and the feedback loop will not work.
{% endhint %}

***

## See also

* [SmartMotorControllerConfig](smart-motor-controller-config.md)
* [SmartMotorControllerTelemetryConfig](smart-motor-controller-telemetry-config.md)
* [SparkWrapper](spark-wrapper.md)
* [NovaWrapper](nova-wrapper.md)
* [A301Wrapper](a301-wrapper.md)
* [TalonFXWrapper / TalonFXSWrapper](talonfx-wrapper.md)
* [BatterySim](../sensor/battery-sim.md)
* [Java SmartMotorController](../../java-reference/motor-controllers/smart-motor-controller.md)
