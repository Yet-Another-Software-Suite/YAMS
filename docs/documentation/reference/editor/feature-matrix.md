---
icon: table-cells
---

# Motor controller feature matrix

Every `SmartMotorControllerConfig` option against every motor controller wrapper in YAMS. This is the one full list: each [SmartMotorControllerConfig](config/) page repeats the row for its own options.

## Supported motor controllers

| Wrapper           | Vendor          | Hardware                                         | Java Package                        | C++ Header                                         |
| ----------------- | --------------- | ------------------------------------------------ | ----------------------------------- | -------------------------------------------------- |
| `TalonFXWrapper`  | CTRE            | TalonFX (Kraken X60, Falcon 500)                 | `yams.core.motorcontrollers.remote` | `yams/motorcontrollers/remote/TalonFXWrapper.hpp`  |
| `TalonFXSWrapper` | CTRE            | TalonFXS (controls NEO, NEO 550, Vortex, Minion) | `yams.core.motorcontrollers.remote` | `yams/motorcontrollers/remote/TalonFXSWrapper.hpp` |
| `SparkWrapper`    | REV             | SparkMax, SparkFlex                              | `yams.core.motorcontrollers.local`  | `yams/motorcontrollers/local/SparkWrapper.hpp`     |
| `NovaWrapper`     | The Thrifty Bot | Thrifty Nova (NEO, Minion, Pulsar, brushed)      | `yams.core.motorcontrollers.local`  | `yams/motorcontrollers/local/NovaWrapper.hpp`      |
| `A301Wrapper`     | FIRST (REVLib)  | FIRST A301 smart motor                           | `yams.core.motorcontrollers.local`  | `yams/motorcontrollers/local/A301Wrapper.hpp`      |

{% hint style="info" %}
The wrappers live in `yams.core`, but you build them with the `SmartMotorControllerConfig` from your command framework's package: `yams.commands2.config.SmartMotorControllerConfig` (Commands v2) or `yams.commands3.config.SmartMotorControllerConfig` (Commands v3).
{% endhint %}

## Vendor configuration classes

Each wrapper accepts a specific vendor configuration class via `SmartMotorControllerConfig.withVendorConfig()`. This allows you to pass through vendor-specific settings that YAMS doesn't directly expose.

| Wrapper                    | Accepted Vendor Config Class                   | Package                        |
| -------------------------- | ---------------------------------------------- | ------------------------------ |
| `TalonFXWrapper`           | `TalonFXConfiguration`                         | `com.ctre.phoenix6.configs`    |
| `TalonFXSWrapper`          | `TalonFXSConfiguration`                        | `com.ctre.phoenix6.configs`    |
| `SparkWrapper` (SparkMax)  | `SparkMaxConfig`                               | `com.revrobotics.spark.config` |
| `SparkWrapper` (SparkFlex) | `SparkFlexConfig`                              | `com.revrobotics.spark.config` |
| `NovaWrapper`              | `NovaConfig.Config`, such as `NovaConfigBatch` | `com.thrifty.nova`             |
| `A301Wrapper`              | None: the A301 has no vendor config            |                                |

{% hint style="warning" %}
**SparkWrapper Note**: The vendor config class must match the actual hardware. Use `SparkMaxConfig` for `SparkMax` controllers and `SparkFlexConfig` for `SparkFlex` controllers. Using the wrong config type will throw a `SmartMotorControllerConfigurationException`.
{% endhint %}

{% hint style="warning" %}
**NovaWrapper Note**: The vendor config must be a `NovaConfig.Config`, usually a `NovaConfigBatch` (in C++ it must be a `thrifty::NovaConfigBatch`). Anything else throws a `SmartMotorControllerConfigurationException`. The vendor config is sent to the Nova before the YAMS config, which overrides it.
{% endhint %}

{% hint style="warning" %}
**A301Wrapper Note**: The A301 has no vendor config, so `withVendorConfig` throws a `SmartMotorControllerConfigurationException`.
{% endhint %}

[How do I use a vendor hardware config?](../../how-to-motor-controllers-and-sensors/how-do-i-use-a-vendor-hardware-config.md) has an example for each vendor.

{% hint style="info" %}
**Configuration Priority**: Settings in `SmartMotorControllerConfig` always take precedence over vendor config settings. The vendor config is applied first as a base, then YAMS overwrites any settings it manages directly.
{% endhint %}

## Feature support matrix

This is every option against every motor controller. What each option does, with a graphic and an example, is on the [SmartMotorControllerConfig](config/) pages.

### Legend

| Symbol | Meaning                         |
| ------ | ------------------------------- |
| ✅      | Fully Supported                 |
| ⚠️     | Partially Supported (see notes) |
| ❌      | Not Supported                   |

***

### Basic configuration

| Feature                       | TalonFX | TalonFXS | SparkMax/Flex | Nova | A301         |
| ----------------------------- | ------- | -------- | ------------- | ---- | ------------ |
| Motor Inversion               | ✅       | ✅        | ✅             | ✅    | ✅            |
| Encoder Inversion             | ❌       | ✅        | ❌             | ❌    | ❌            |
| Zero Power Mode (Brake/Coast) | ✅       | ✅        | ✅             | ✅    | ✅            |
| Reset Previous Config         | ✅       | ✅        | ✅             | ✅    | ⚠️ (ignored) |
| Vendor Config Passthrough     | ✅       | ✅        | ✅             | ✅    | ❌            |
| Vendor Control Request        | ✅       | ✅        | ❌             | ❌    | ❌            |
| Starting Position             | ✅       | ✅        | ✅             | ✅    | ✅            |

{% hint style="warning" %}
**A301 Encoder Inversion**: `A301Wrapper` throws an `IllegalArgumentException` if `withEncoderInverted` is set. Invert the motor with `withMotorInverted` instead.
{% endhint %}

### Current limits

| Feature              | TalonFX | TalonFXS | SparkMax/Flex | Nova | A301 |
| -------------------- | ------- | -------- | ------------- | ---- | ---- |
| Stator Current Limit | ✅       | ✅        | ✅             | ✅    | ❌    |
| Supply Current Limit | ✅       | ✅        | ❌             | ✅    | ❌    |

{% hint style="warning" %}
**A301 Current Limits**: the A301 has no current limits, so `A301Wrapper` throws a `SmartMotorControllerConfigurationException` if either is configured, and `setStatorCurrentLimit` and `setSupplyCurrentLimit` throw an `UnsupportedOperationException`. See [Limiting power consumption](../../how-to-motor-controllers-and-sensors/limiting-power-consumption.md).
{% endhint %}

### Voltage configuration

| Feature                 | TalonFX | TalonFXS | SparkMax/Flex                                                                                    | Nova                                                                                     | A301 |
| ----------------------- | ------- | -------- | ------------------------------------------------------------------------------------------------ | ---------------------------------------------------------------------------------------- | ---- |
| Voltage Compensation    | ❌       | ❌        | ✅                                                                                                | ✅                                                                                        | ❌    |
| Closed Loop Max Voltage | ✅       | ✅        | ⚠️ (requires an exponential profile or [LQR controller](../../understanding/lqr-controllers.md)) | ⚠️ (exponential, velocity profile or [LQR](../../understanding/lqr-controllers.md) only) | ✅    |

{% hint style="warning" %}
**Voltage Compensation**: `TalonFXWrapper` and `TalonFXSWrapper` throw an `IllegalArgumentException` if voltage compensation is configured; it's supported on `SparkWrapper` and `NovaWrapper`. `A301Wrapper` throws a `SmartMotorControllerConfigurationException`, since the A301 has no voltage compensation.
{% endhint %}

{% hint style="warning" %}
**Closed Loop Max Voltage on Sparks and Novas**: `SparkWrapper` and `NovaWrapper` only support `withClosedLoopControllerMaximumVoltage` when the closed loop controller runs on the SystemCore (an Exponential Profile or [LQR controller](../../understanding/lqr-controllers.md), or on a Nova a velocity trapezoidal profile or another loop the Nova can't run); otherwise they throw a `SmartMotorControllerConfigurationException`.
{% endhint %}

### Ramp rates

| Feature               | TalonFX | TalonFXS | SparkMax/Flex | Nova               | A301 |
| --------------------- | ------- | -------- | ------------- | ------------------ | ---- |
| Open Loop Ramp Rate   | ✅       | ✅        | ✅             | ⚠️ (one ramp rate) | ❌    |
| Closed Loop Ramp Rate | ✅       | ✅        | ✅             | ⚠️ (one ramp rate) | ❌    |

{% hint style="warning" %}
**Nova Ramp Rate**: The Thrifty Nova has one ramp rate for both open and closed loop control. `NovaWrapper` throws a `SmartMotorControllerConfigurationException` if `withOpenLoopRampRate` and `withClosedLoopRampRate` are given different values; set one of them, or both to the same value.
{% endhint %}

{% hint style="warning" %}
**A301 Ramp Rates**: the A301 has no ramp rates. `A301Wrapper` throws a `SmartMotorControllerConfigurationException` if either is configured, and the live setters throw an `UnsupportedOperationException`.
{% endhint %}

### Gearing

| Feature                          | TalonFX | TalonFXS | SparkMax/Flex | Nova | A301                            |
| -------------------------------- | ------- | -------- | ------------- | ---- | ------------------------------- |
| Mechanism Gearing                | ✅       | ✅        | ✅             | ✅    | ✅ (includes the A301's gearbox) |
| External Encoder Gearing         | ✅       | ✅        | ✅             | ✅    | ✅                               |
| Mechanism Circumference (Linear) | ✅       | ✅        | ✅             | ✅    | ✅                               |

{% hint style="info" %}
**A301 Gearing**: `withGearing` is from the A301's rotor to the mechanism, so it includes the gearbox on the A301 (135:1, 64:1, 36:1, 27:1, 13.5:1 or 3:1) and anything after it. YAMS does not detect the gearbox. The external encoder gearing is from the A301's output shaft, where its absolute encoder is, to the mechanism.
{% endhint %}

### Closed-loop control

| Feature                                                  | TalonFX                   | TalonFXS                  | SparkMax/Flex                                                                                    | Nova                                                                                     | A301                                                        |
| -------------------------------------------------------- | ------------------------- | ------------------------- | ------------------------------------------------------------------------------------------------ | ---------------------------------------------------------------------------------------- | ----------------------------------------------------------- |
| PID Controller                                           | ✅                         | ✅                         | ✅                                                                                                | ✅                                                                                        | ✅ (runs on the SystemCore)                                  |
| [LQR Controller](../../understanding/lqr-controllers.md) | ⚠️ (SystemCore)           | ⚠️ (SystemCore)           | ⚠️ (SystemCore)                                                                                  | ⚠️ (SystemCore)                                                                          | ⚠️ (SystemCore)                                             |
| Closed Loop Tolerance                                    | ❌                         | ❌                         | ⚠️ (not with [LQR](../../understanding/lqr-controllers.md))                                      | ⚠️ (not with [LQR](../../understanding/lqr-controllers.md))                              | ⚠️ (not with [LQR](../../understanding/lqr-controllers.md)) |
| Control Period Override                                  | ❌                         | ❌                         | ⚠️ (requires an exponential profile or [LQR controller](../../understanding/lqr-controllers.md)) | ⚠️ (exponential, velocity profile or [LQR](../../understanding/lqr-controllers.md) only) | ⚠️ (requires closed loop mode)                              |
| Closed Loop Slots                                        | ⚠️ (`SLOT_0` to `SLOT_2`) | ⚠️ (`SLOT_0` to `SLOT_2`) | ✅                                                                                                | ⚠️ (`SLOT_0` and `SLOT_1` when the Nova runs the loop)                                   | ✅                                                           |

{% hint style="warning" %}
**LQR Controller Execution**: LQR is **not natively supported** by any motor controller hardware. When configured, the closed loop control runs on the **SystemCore**: the wrapper starts a `Notifier` that calls `SmartMotorController.iterateClosedLoopController()` every closed loop control period (20 ms unless you set `withClosedLoopControlPeriod`), so there is nothing to call yourself. See [LQR Controllers](../../understanding/lqr-controllers.md) for details.
{% endhint %}

{% hint style="info" %}
**Nova Closed Loop Execution**: `NovaWrapper` runs a PID, and a trapezoidal position profile with its feedforward, on the Nova, in its two slots. An [LQR](../../understanding/lqr-controllers.md), an exponential profile or a velocity trapezoidal profile runs on the SystemCore, which commands the Nova with voltage, and a YAMS alert reports it. The Nova only applies its motion profile slot's kS, kV, kA and kG in a motion profile, so a feedforward without a trapezoidal profile runs on the SystemCore with its PID, and a YAMS alert says why. An absolute encoder closing the loop over a range other than \[0, 1) without continuous wrapping, and continuous wrapping of an absolute encoder geared to the mechanism, run on the SystemCore. Continuous wrapping over any range of a 1:1 absolute encoder, the motor's encoder or a quadrature encoder runs on the Nova. ThriftyLib's simulation ignores the Nova's onboard PID gains, so in simulation YAMS runs the Nova's closed loop itself, with the gains, profile and limits the Nova was sent. There, as on the Nova, the motion profile slot's feedforward only applies in a motion profile.
{% endhint %}

{% hint style="info" %}
**A301 Closed Loop Execution**: the A301 has no closed loop gains, so `A301Wrapper` always runs its closed loop controller (PID or [LQR](../../understanding/lqr-controllers.md), motion profiles and feedforwards) on the SystemCore and commands the A301 with voltage. A YAMS alert reports that the closed loop controller is running on the SystemCore.
{% endhint %}

### Motion profiles

| Feature             | TalonFX               | TalonFXS              | SparkMax/Flex   | Nova                                                 | A301            |
| ------------------- | --------------------- | --------------------- | --------------- | ---------------------------------------------------- | --------------- |
| Trapezoidal Profile | ✅ (Motion Magic)      | ✅ (Motion Magic)      | ✅ (MAXMotion)   | ✅ (on the Nova; velocity profiles on the SystemCore) | ⚠️ (SystemCore) |
| Exponential Profile | ✅ (Motion Magic Expo) | ✅ (Motion Magic Expo) | ⚠️ (SystemCore) | ⚠️ (SystemCore)                                      | ⚠️ (SystemCore) |

{% hint style="info" %}
**Profile Execution Location**:

* **TalonFX/TalonFXS**: Profiles run on the motor controller (Motion Magic)
* **SparkMax/SparkFlex**: Trapezoidal profiles run on motor controller (MAXMotion), exponential profiles run on the SystemCore
* **Thrifty Nova**: Trapezoidal position profiles run on the Nova, exponential and velocity trapezoidal profiles run on the SystemCore
* **A301**: Both profiles run on the SystemCore
{% endhint %}

### Feedforward

| Feature                | TalonFX | TalonFXS | SparkMax/Flex | Nova | A301 |
| ---------------------- | ------- | -------- | ------------- | ---- | ---- |
| SimpleMotorFeedforward | ✅       | ✅        | ✅             | ✅    | ✅    |
| ArmFeedforward         | ✅       | ✅        | ✅             | ✅    | ✅    |
| ElevatorFeedforward    | ✅       | ✅        | ✅             | ✅    | ✅    |
| Simulation Feedforward | ✅       | ✅        | ✅             | ✅    | ✅    |

### Soft limits

| Feature              | TalonFX                    | TalonFXS                   | SparkMax/Flex              | Nova                                                                           | A301                            |
| -------------------- | -------------------------- | -------------------------- | -------------------------- | ------------------------------------------------------------------------------ | ------------------------------- |
| Angle Soft Limits    | ⚠️ (closed loop mode only) | ⚠️ (closed loop mode only) | ⚠️ (closed loop mode only) | ⚠️ (closed loop mode only, both limits)                                        | ⚠️ (closed loop setpoints only) |
| Distance Soft Limits | ⚠️ (closed loop mode only) | ⚠️ (closed loop mode only) | ⚠️ (closed loop mode only) | ⚠️ (closed loop mode only, both limits)                                        | ⚠️ (closed loop setpoints only) |
| Continuous Wrapping  | ✅                          | ✅                          | ✅                          | ✅ (on the Nova; an absolute encoder geared to the mechanism on the SystemCore) | ✅                               |

{% hint style="warning" %}
**Soft limits** are only enforced in `ControlMode.CLOSED_LOOP`. They are set on the motor controller and stop any output past them, voltage and duty cycle included. A Thrifty Nova needs both an upper and a lower limit. On an A301, whose loop runs on the SystemCore, a position setpoint past a limit is moved to the limit, but a voltage or duty cycle is not stopped. Soft limits cannot be combined with continuous wrapping. See [`withSoftLimits`](config/limits-and-protection.md#withsoftlimits).
{% endhint %}

### External encoders

| Feature                      | TalonFX                                   | TalonFXS                                  | SparkMax/Flex                                                                                    | Nova                                                                                                                           | A301                               |
| ---------------------------- | ----------------------------------------- | ----------------------------------------- | ------------------------------------------------------------------------------------------------ | ------------------------------------------------------------------------------------------------------------------------------ | ---------------------------------- |
| External Encoder Support     | ✅                                         | ✅                                         | ✅                                                                                                | ✅                                                                                                                              | ⚠️ (its own absolute encoder only) |
| External Encoder Inversion   | ✅                                         | ✅                                         | ✅                                                                                                | ✅                                                                                                                              | ✅                                  |
| External Encoder Zero Offset | ✅                                         | ✅                                         | ⚠️ (absolute encoders only)                                                                      | ⚠️ (absolute and CAN encoders only)                                                                                            | ✅                                  |
| Wrapping Range               | ✅ (from \[-1, 0) to \[0, 1))              | ✅ (from \[-1, 0) to \[0, 1))              | ⚠️ (absolute encoder from \[-1, 0) to \[0, 1); `DetachedEncoder` \[-0.5, 0.5) or \[0, 1))        | ✅ (any range; for the encoder closing the loop, \[0, 1) or continuous wrapping on the Nova, any other range on the SystemCore) | ✅ (from \[-1, 0) to \[0, 1))       |
| Use External for Feedback    | ⚠️ (`false` ignores the external encoder) | ⚠️ (`false` ignores the external encoder) | ✅                                                                                                | ✅                                                                                                                              | ✅                                  |
| Feedback Synchronization     | ❌                                         | ❌                                         | ⚠️ (requires an exponential profile or [LQR controller](../../understanding/lqr-controllers.md)) | ⚠️ (exponential, velocity profile or [LQR](../../understanding/lqr-controllers.md) only)                                       | ⚠️ (requires closed loop mode)     |

**Supported External Encoder Types:**

| Wrapper       | Supported Encoders                                                                                                                                                                                                                        |
| ------------- | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| TalonFX       | `CANcoder`, `CANdi` (PWM1/PWM2)                                                                                                                                                                                                           |
| TalonFXS      | `CANcoder`, `CANdi` (PWM1/PWM2)                                                                                                                                                                                                           |
| SparkMax/Flex | `SparkAbsoluteEncoder` (an absolute encoder on the SPARK, such as a Through Bore), `SparkMaxAlternateEncoder` or `SparkFlexExternalEncoder` (a quadrature encoder on the SPARK), `DetachedEncoder` (a REV absolute encoder read over CAN) |
| Nova          | `FeedbackSensorType.ABS` (absolute encoder on the data port), `FeedbackSensorType.QUAD` (quadrature encoder on the data port), `CanEncoder` (Thrifty CAN Encoder)                                                                         |
| A301          | The `A301` itself (its own absolute encoder, on its output shaft)                                                                                                                                                                         |

{% hint style="info" %}
**Nova external encoders**: pass `FeedbackSensorType.ABS` or `FeedbackSensorType.QUAD` (from `com.thrifty.core.Motor`) to `withExternalEncoder`. The encoder's type comes from the vendor config, for example `NovaConfig.absoluteEncoderType(Motor.AbsoluteEncoderType.REV_ENCODER)` or `NovaConfig.quadratureEncoderType(Motor.QuadratureEncoderType.custom(2048))`. Zero offset and the [wrapping range](config/encoders.md#withwrapping) only apply to absolute and CAN encoders.
{% endhint %}

{% hint style="info" %}
**A301 external encoder**: pass the `A301` itself to `withExternalEncoder` to use its absolute encoder, which is on its output shaft, after the gearbox. Anything else throws an `IllegalArgumentException`. YAMS applies the zero offset, inversion and wrapping range itself; without `withWrapping` the encoder reads \[0, 1). The A301 does not report the absolute encoder's velocity, so the external encoder velocity comes from the relative encoder.
{% endhint %}

### Follower motors

| Feature                   | TalonFX | TalonFXS | SparkMax/Flex | Nova | A301 |
| ------------------------- | ------- | -------- | ------------- | ---- | ---- |
| Hardware Followers        | ✅       | ✅        | ✅             | ✅    | ❌    |
| Follower Inversion        | ✅       | ✅        | ✅             | ✅    | ❌    |
| Loosely Coupled Followers | ✅       | ✅        | ✅             | ✅    | ✅    |

{% hint style="warning" %}
**Follower Compatibility**: Followers must be the same brand as the leader motor controller. You cannot mix vendors for hardware followers. Within a vendor the models can be mixed: a TalonFX can follow a TalonFXS and a TalonFXS can follow a TalonFX, and a SPARK MAX and a SPARK Flex can follow each other. Loosely coupled followers are different: each one runs its own closed loop, so you can mix brands freely, such as a SPARK leading a TalonFX. An A301 cannot have hardware followers: `withFollowers` throws a `SmartMotorControllerConfigurationException`, so use loosely coupled followers.
{% endhint %}

### Telemetry

| Feature                 | TalonFX | TalonFXS | SparkMax/Flex | Nova | A301 |
| ----------------------- | ------- | -------- | ------------- | ---- | ---- |
| Telemetry Name          | ✅       | ✅        | ✅             | ✅    | ✅    |
| Telemetry Verbosity     | ✅       | ✅        | ✅             | ✅    | ✅    |
| Custom Telemetry Config | ✅       | ✅        | ✅             | ✅    | ✅    |

`A301Wrapper` does not report supply current, current limits or ramp rates.

### Simulation

| Feature              | TalonFX | TalonFXS | SparkMax/Flex | Nova | A301 |
| -------------------- | ------- | -------- | ------------- | ---- | ---- |
| DCMotorSim           | ✅       | ✅        | ✅             | ✅    | ✅    |
| Moment of Inertia    | ✅       | ✅        | ✅             | ✅    | ✅    |
| Custom Sim Supplier  | ✅       | ✅        | ✅             | ✅    | ✅    |
| External Encoder Sim | ✅       | ✅        | ✅             | ✅    | ✅    |

{% hint style="info" %}
**A301 Simulation**: REVLib does not simulate the A301, so `A301Wrapper` drives a WPILib `DCMotorSim` of the A301's motor, `YMotors.getA301(1)`. A mechanism's own sim, such as an Arm's, replaces it, as on any other motor controller.
{% endhint %}

### Safety features

| Feature            | TalonFX                                                                   | TalonFXS                                                                  | SparkMax/Flex                                                                                    | Nova                                                                                     | A301                           |
| ------------------ | ------------------------------------------------------------------------- | ------------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------ | ---------------------------------------------------------------------------------------- | ------------------------------ |
| Temperature Cutoff | ⚠️ (requires an [LQR controller](../../understanding/lqr-controllers.md)) | ⚠️ (requires an [LQR controller](../../understanding/lqr-controllers.md)) | ⚠️ (requires an exponential profile or [LQR controller](../../understanding/lqr-controllers.md)) | ⚠️ (exponential, velocity profile or [LQR](../../understanding/lqr-controllers.md) only) | ⚠️ (requires closed loop mode) |

{% hint style="warning" %}
**Temperature Cutoff**: the cutoff is checked by the closed loop running on the SystemCore, so it only works where the loop runs there. That is an [LQR controller](../../understanding/lqr-controllers.md) on any motor controller, an Exponential Profile on a `SparkWrapper` or `NovaWrapper`, a velocity trapezoidal profile or another loop the Nova can't run on a `NovaWrapper`, and always on an `A301Wrapper`. With PID or a profile that runs on the motor controller, it does not work.
{% endhint %}

***

## Motor type support by wrapper

### TalonFXWrapper

* Kraken X60 / Kraken X60 FOC
* Falcon 500 / Falcon 500 FOC

### TalonFXSWrapper

* NEO (via JST connector)
* NEO 550 (via JST connector)
* NEO Vortex (via JST connector)
* Minion (via JST connector, with Advanced Hall Support)

### SparkWrapper

* NEO (SparkMax with brushless mode)
* NEO 550 (SparkMax with brushless mode)
* NEO Vortex (SparkFlex)
* Any brushless motor (SparkMax/SparkFlex)

### NovaWrapper

Set with the `MotorType` given to the `Nova` constructor (`com.thrifty.core.Motor.MotorType`):

* NEO (`MotorType.NEO`)
* Minion (`MotorType.MINION`)
* Pulsar (`MotorType.PULSAR`)
* Brushed DC motors (`MotorType.BRUSHED`)

### A301Wrapper

Every A301 has the same motor, so `A301Wrapper` takes no `DCMotor`: it uses `YMotors.getA301(1)` (C++ `YMotors::A301()`).

***

## Configuration examples

These are fragments: each goes inside a subsystem, which is the `this` passed to the config.

### TalonFX with CANcoder

```java
TalonFX motor = new TalonFX(1, new CANBus(CANPort.CAN_S0));
CANcoder encoder = new CANcoder(2, new CANBus(CANPort.CAN_S0));

SmartMotorControllerConfig config = new SmartMotorControllerConfig(this)
    .withControlMode(ControlMode.CLOSED_LOOP)
    .withClosedLoopController(1, 0, 0)
    .withGearing(new MechanismGearing(GearBox.fromReductionStages(5)))
    .withExternalEncoder(encoder)
    .withExternalEncoderZeroOffset(Degrees.of(90))
    .withUseExternalFeedbackEncoder(true)
    .withZeroPower(MotorMode.BRAKE)
    .withStatorCurrentLimit(Amps.of(40))
    .withTelemetry("ArmMotor", TelemetryVerbosity.HIGH);

SmartMotorController smc = new TalonFXWrapper(motor, DCMotor.getKrakenX60(1), config);
```

### TalonFXS with NEO

```java
TalonFXS motor = new TalonFXS(1, new CANBus(CANPort.CAN_S0));

SmartMotorControllerConfig config = new SmartMotorControllerConfig(this)
    .withControlMode(ControlMode.CLOSED_LOOP)
    .withClosedLoopController(0.5, 0, 0)
    .withGearing(new MechanismGearing(GearBox.fromReductionStages(4)))
    .withFeedforward(new SimpleMotorFeedforward(0.1, 0.12, 0))
    .withZeroPower(MotorMode.COAST)
    .withStatorCurrentLimit(Amps.of(40))
    .withTelemetry("FlywheelMotor", TelemetryVerbosity.HIGH);

SmartMotorController smc = new TalonFXSWrapper(motor, DCMotor.getNEO(1), config);
```

### SparkMax with absolute encoder

```java
SparkMax spark = new SparkMax(CANPort.CAN_S0, 1, MotorType.kBrushless);

SmartMotorControllerConfig config = new SmartMotorControllerConfig(this)
    .withControlMode(ControlMode.CLOSED_LOOP)
    .withClosedLoopController(1, 0, 0)
    .withTrapezoidalProfile(DegreesPerSecond.of(180), DegreesPerSecondPerSecond.of(360))
    .withExternalEncoder(spark.getAbsoluteEncoder())
    .withExternalEncoderInverted(true)
    .withUseExternalFeedbackEncoder(true)
    .withExternalEncoderZeroOffset(Degrees.of(45))
    .withFeedforward(new ArmFeedforward(0, 0.5, 0, 0))
    .withZeroPower(MotorMode.BRAKE)
    .withTelemetry("WristMotor", TelemetryVerbosity.HIGH);

SmartMotorController smc = new SparkWrapper(spark, DCMotor.getNEO(1), config);
```

### Thrifty Nova with a CAN encoder

```java
Nova nova = new Nova(0, 3, MotorType.NEO);   // CAN bus 0, CAN ID 3
CanEncoder encoder = new CanEncoder(0, 4);   // CAN bus 0, CAN ID 4

SmartMotorControllerConfig config = new SmartMotorControllerConfig(this)
    .withControlMode(ControlMode.CLOSED_LOOP)
    .withClosedLoopController(0.2, 0, 0)
    .withGearing(new MechanismGearing(GearBox.fromReductionStages(5)))
    .withExternalEncoder(encoder)
    .withExternalEncoderZeroOffset(Degrees.of(90))
    .withUseExternalFeedbackEncoder(true)
    .withZeroPower(MotorMode.BRAKE)
    .withStatorCurrentLimit(Amps.of(40))
    .withTelemetry("ArmMotor", TelemetryVerbosity.HIGH);

SmartMotorController smc = new NovaWrapper(nova, DCMotor.getNEO(1), config);
```

### A301 arm with its absolute encoder

```java
A301 a301 = new A301(CANPorts.fromBusId(1), 20);   // CAN bus 1, CAN ID 20

SmartMotorControllerConfig config = new SmartMotorControllerConfig(this)
    .withControlMode(ControlMode.CLOSED_LOOP)
    .withClosedLoopController(30, 0, 0)
    .withTrapezoidalProfile(DegreesPerSecond.of(180), DegreesPerSecondPerSecond.of(360))
    .withFeedforward(new ArmFeedforward(0, 1.6, 10.2, 0))
    .withGearing(new MechanismGearing(GearBox.fromReductionStages(64, 3)))   // 64:1 gearbox, then a 3:1 chain
    .withExternalEncoder(a301)
    .withExternalEncoderGearing(new MechanismGearing(GearBox.fromReductionStages(3)))   // output shaft to arm
    .withUseExternalFeedbackEncoder(true)
    .withZeroPower(MotorMode.BRAKE)
    .withTelemetry("ArmMotor", TelemetryVerbosity.HIGH);

SmartMotorController smc = new A301Wrapper(a301, config);
```

The [`a301_arm` example](../../../../examples/commands2/a301_arm) (Commands v2) is a complete arm on one A301, through its 64:1 gearbox and a 3:1 chain, that runs in simulation.

***

## Known limitations

### SparkMax/SparkFlex (SparkWrapper)

* Exponential profiles run on the SystemCore, not on the motor controller
* Supply current limits are not supported
* External encoders must be REV encoders the SPARK reads itself: its absolute encoder, its alternate or external quadrature encoder, or a `DetachedEncoder`
* Soft limits are only enforced in closed loop mode

### TalonFX/TalonFXS

* Requires Phoenix 6 library
* CANcoder/CANdi must be on same CAN bus for best performance

### Thrifty Nova (NovaWrapper)

* Requires ThriftyLib 2027
* PID and trapezoidal position profiles run on the Nova, in two slots; exponential and velocity profiles, LQR and wrapping the Nova can't do run on the SystemCore
* Soft limits need both an upper and a lower limit
* One ramp rate for open and closed loop control
* The internal encoder cannot be inverted (`withEncoderInverted` throws an `IllegalArgumentException`)
* Vendor control requests (`withVendorControlRequest`) are not supported
* Hardware followers must be `Nova` controllers
* Setting the encoder velocity is only supported in simulation

### FIRST A301 (A301Wrapper)

* Requires REVLib 2027 (`com.revrobotics.spark.A301`)
* Takes no `DCMotor`: every A301 has the same motor, `YMotors.getA301(1)`
* All closed loop control, motion profiles and feedforwards run on the SystemCore
* No current limits, ramp rates, voltage compensation, vendor config or vendor control requests: configuring them throws a `SmartMotorControllerConfigurationException`
* No hardware followers; use loosely coupled followers
* The internal encoder cannot be inverted (`withEncoderInverted` throws an `IllegalArgumentException`)
* The only external encoder is the A301's own absolute encoder, on its output shaft
* Soft limits move a position setpoint to the limit, but do not stop a voltage or duty cycle
* About 28 W peak, much less than a NEO, so an arm needs a big reduction
* Setting the encoder velocity is only supported in simulation
