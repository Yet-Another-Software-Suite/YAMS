# TalonFXWrapper / TalonFXSWrapper

**Package:** `yams.core.motorcontrollers.remote`\
**Extends:** `SmartMotorController`\
**Javadoc:** [`TalonFXWrapper`](https://javadocs.yassrobotics.com/yams/yams/core/motorcontrollers/remote/TalonFXWrapper.html), [`TalonFXSWrapper`](https://javadocs.yassrobotics.com/yams/yams/core/motorcontrollers/remote/TalonFXSWrapper.html)

`SmartMotorController` implementations for CTRE TalonFX and TalonFXS controllers. `TalonFXWrapper` targets the TalonFX (Falcon 500, Kraken X60). `TalonFXSWrapper` targets the TalonFXS.

## Constructors

```java
TalonFXWrapper(TalonFX talonFX, DCMotor motor, SmartMotorControllerConfig<?> config)
TalonFXSWrapper(TalonFXS talonFXS, DCMotor motor, SmartMotorControllerConfig<?> config)
```

## Supported hardware

| Hardware                      | Class             | `DCMotor` Model                                    |
| ----------------------------- | ----------------- | -------------------------------------------------- |
| TalonFX (Kraken X60)          | `TalonFXWrapper`  | `DCMotor.getKrakenX60(1)`                          |
| TalonFX (Falcon 500)          | `TalonFXWrapper`  | `DCMotor.getFalcon500(1)`                          |
| TalonFXS (Minion)             | `TalonFXSWrapper` | `DCMotor.getMinion(1)`, driven as `Minion_JST`     |
| TalonFXS (NEO)                | `TalonFXSWrapper` | `DCMotor.getNEO(1)`, driven as `NEO_JST`           |
| TalonFXS (NEO 550)            | `TalonFXSWrapper` | `DCMotor.getNeo550(1)`, driven as `NEO550_JST`     |
| TalonFXS (NEO Vortex)         | `TalonFXSWrapper` | `DCMotor.getNeoVortex(1)`, driven as `VORTEX_JST`  |
| TalonFXS (Thrifty Pulsar 775) | `TalonFXSWrapper` | `YMotors.getThriftyPulsar(1)`, driven as `NEO_JST` |

`TalonFXSWrapper` picks the TalonFXS's motor arrangement from the `DCMotor`, and throws `IllegalArgumentException` for one it does not know. WPILib has no `DCMotor` of the Thrifty Pulsar, so YAMS has one in `yams.core.motorcontrollers.YMotors`, from Thrifty's specs. It will be removed once WPILib has one. `YMotors` also has the FIRST A301's motor (see [A301Wrapper](a301-wrapper.md)).

## Example

```java
SmartMotorControllerConfig config = new SmartMotorControllerConfig(this)
    .withMotorInverted(false)
    .withStatorCurrentLimit(Amps.of(60))
    .withSupplyCurrentLimit(Amps.of(80))
    .withClosedLoopController(0.3, 0.0, 0.0);

SmartMotorController motor = new TalonFXWrapper(
    new TalonFX(5, new CANBus(CANPort.CAN_S0)),
    DCMotor.getKrakenX60(1),
    config
);
```

## CTRE-specific notes

**Phoenix 6 required.** YAMS only supports Phoenix 6 (26.70 alpha for WPILib 2027) for TalonFX and TalonFXS. Phoenix 5 is not supported. Phoenix 6 devices take a `CANBus`, e.g. `new CANBus(CANPort.CAN_S0)` for a SystemCore CAN port.

**CANcoder support.** Pass a `CANcoder` via `withExternalEncoder(CANcoder)`. YAMS configures the TalonFX to fuse the CANcoder as the remote feedback source. A wrapping range is optional for `CANcoder` and `CANdi`; when set with `withWrapping` or `withContinuousWrapping`, its maximum, from 0 to 1 rotation, becomes the encoder's `AbsoluteSensorDiscontinuityPoint`, and any other maximum throws `SmartMotorControllerConfigurationException`. Without one, the encoder keeps its own range. Only `withContinuousWrapping` turns on `ClosedLoopGeneral.ContinuousWrap`.

**CANdi support.** A `CANdi` can be passed via `withExternalEncoder(CANdi)` for both `TalonFXWrapper` and `TalonFXSWrapper`. Because a CANdi has two PWM inputs (PWM1 and PWM2), YAMS cannot determine which one your encoder is wired to automatically. You must pass a vendor config that sets the correct `FeedbackSensorSource` (TalonFX) or `ExternalFeedbackSensorSource` (TalonFXS), otherwise the CANdi will not function as an external encoder.

**TalonFX + CANdi:**

```java
TalonFXConfiguration talonConfig = new TalonFXConfiguration();
// Choose SyncCANdiPWM1 or SyncCANdiPWM2 depending on which port the encoder is wired to.
talonConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.SyncCANdiPWM1;
talonConfig.Feedback.FeedbackRemoteSensorID = candi.getDeviceID();

SmartMotorControllerConfig config = new SmartMotorControllerConfig(this)
    .withVendorConfig(talonConfig)
    .withExternalEncoder(candi)
    .withUseExternalFeedbackEncoder(true)
    .withGearing(new MechanismGearing(GearBox.fromRatio(10.0)));

SmartMotorController motor = new TalonFXWrapper(
    new TalonFX(5, new CANBus(CANPort.CAN_S0)),
    DCMotor.getKrakenX60(1),
    config
);
```

**TalonFXS + CANdi:**

```java
TalonFXSConfiguration talonSConfig = new TalonFXSConfiguration();
// Choose SyncCANdiPWM1 or SyncCANdiPWM2 depending on which port the encoder is wired to.
talonSConfig.ExternalFeedback.ExternalFeedbackSensorSource =
    ExternalFeedbackSensorSourceValue.SyncCANdiPWM1;
talonSConfig.ExternalFeedback.ExternalFeedbackRemoteSensorID = candi.getDeviceID();

SmartMotorControllerConfig config = new SmartMotorControllerConfig(this)
    .withVendorConfig(talonSConfig)
    .withExternalEncoder(candi)
    .withUseExternalFeedbackEncoder(true)
    .withGearing(new MechanismGearing(GearBox.fromRatio(10.0)));

SmartMotorController motor = new TalonFXSWrapper(
    new TalonFXS(6, new CANBus(CANPort.CAN_S0)),
    DCMotor.getKrakenX60(1),
    config
);
```

{% hint style="warning" %}
Without a vendor config that explicitly sets the sensor source, YAMS has no way to know which of the two PWM ports the CANdi encoder is connected to and the feedback loop will not work.
{% endhint %}

**Motion Magic.** When a trapezoidal or exponential profile is configured, YAMS uses CTRE Motion Magic control requests internally (`MotionMagicVoltage`, `MotionMagicExpoVoltage`, or the appropriate current-mode variant). When an [LQR controller](../math/lqr-controller.md) is configured, the closed loop runs on the SystemCore instead and YAMS raises a `closed loop controller is running on the SystemCore` alert.

**Control requests.** TalonFX uses Phoenix 6 control request objects. YAMS selects the appropriate request type (duty cycle, voltage, position, velocity, torque-current FOC) based on the configuration. Advanced users can inject a custom control request via `withVendorControlRequest(ControlRequest)` in the config.

**Configuration.** `TalonFXConfiguration` is built from `SmartMotorControllerConfig` and applied via `TalonFXConfigurator`. YAMS retries configuration on failure.

**Starting position and zero offset.** `withStartingPosition` is not applied when an external encoder is used for feedback; YAMS raises an alert and the external encoder position is used instead. `withExternalEncoderZeroOffset` is applied to the CANcoder/CANdi, not as a rotor offset; without an external encoder YAMS raises an alert.

**Simulation.** Uses `DCMotorSim` with the configured moment of inertia. The simulated TalonFX models a Kraken X44 when the `DCMotor` you pass is `DCMotor.getKrakenX44(n)` or `DCMotor.getKrakenX44Foc(n)`, for one to four motors, and a Kraken X60 otherwise.

## See also

* [SmartMotorController](smart-motor-controller.md)
* [SmartMotorControllerConfig](smart-motor-controller-config.md)
* [SparkWrapper](spark-wrapper.md)
* [NovaWrapper](nova-wrapper.md)
* [A301Wrapper](a301-wrapper.md)
* [C++ SmartMotorController](../../c++-reference/motor-controllers/smart-motor-controller.md)
