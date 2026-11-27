# SparkWrapper

**Package:** `yams.core.motorcontrollers.local`\
**Extends:** `SmartMotorController`\
**Javadoc:** [`SparkWrapper`](https://javadocs.yassrobotics.com/yams/yams/core/motorcontrollers/local/SparkWrapper.html)

`SmartMotorController` implementation for REV SPARK MAX and SPARK FLEX controllers.

## Constructor

```java
SparkWrapper(SparkBase spark, DCMotor motor, SmartMotorControllerConfig<?> config)
```

`SparkBase` accepts `SparkMax` or `SparkFlex`. `config` is the `SmartMotorControllerConfig` from `yams.commands2.config` or `yams.commands3.config`. A vendor config passed with `withVendorConfig` must be a `SparkMaxConfig` for a `SparkMax` and a `SparkFlexConfig` for a `SparkFlex`.

## Supported hardware

| Hardware                    | `DCMotor` Model           |
| --------------------------- | ------------------------- |
| `SparkMax` with NEO         | `DCMotor.getNEO(1)`       |
| `SparkMax` with NEO 550     | `DCMotor.getNeo550(1)`    |
| `SparkFlex` with NEO Vortex | `DCMotor.getNeoVortex(1)` |

## Example

```java
SmartMotorControllerConfig config = new SmartMotorControllerConfig(this)
    .withMotorInverted(false)
    .withStatorCurrentLimit(Amps.of(40))
    .withClosedLoopController(0.2, 0.0, 0.0)
    .withWrapping(Rotations.of(-0.5), Rotations.of(0.5)); // the absolute encoder reads [-0.5, 0.5)

SmartMotorController motor = new SparkWrapper(
    new SparkMax(CANPort.CAN_S0, 3, MotorType.kBrushless),
    DCMotor.getNEO(1),
    config
);
```

## SPARK-specific notes

**Absolute encoder wrapping range.** Set where an absolute external encoder's reading wraps with `withWrapping(min, max)`, or `withContinuousWrapping(min, max)` to also have the closed loop take the shortest path (see [Wrapping](smart-motor-controller-config.md#wrapping)). Without either, the encoder reads `[0, 1)`.

* `SparkAbsoluteEncoder`: any one rotation from `[-1, 0)` to `[0, 1)`, such as `Rotations.of(-0.5)` to `Rotations.of(0.5)`. YAMS sets REVLib's range offset to the middle of the range, `(min + max) / 2`.
* `DetachedEncoder`: `[-0.5, 0.5)` (zero centered) or `[0, 1)`.

Any other range throws `SmartMotorControllerConfigurationException`. Whenever wrapping is set, the SPARK's `positionWrappingInputRange` is the wrapping range; `positionWrappingEnabled` is only turned on by `withContinuousWrapping`.

{% hint style="info" %}
Set the wrapping range when the mechanism must read across the point where `[0, 1)` wraps, such as an arm that works on both sides of zero. Without it, the encoder jumps from 1 to 0 rotations there.
{% endhint %}

**Supported external encoders.** The SPARK's absolute encoder (`SparkAbsoluteEncoder`), the SPARK MAX alternate encoder, the SPARK Flex external encoder (quadrature, counts per revolution from the vendor config), or an encoder the SPARK reads over CAN (a REVLib `DetachedEncoder`).

**Configuration persistence.** YAMS applies the SPARK configuration on construction using `SparkBase.configure()`. Safe parameters are reset first when `withResetPreviousConfig(true)` (the default). Settings are burned to flash with `PersistMode.kPersistParameters` only while the robot is disabled; configuration changes made while enabled are not persisted.

**NEO 550 safety.** `checkConfigSafety()` throws `SmartMotorControllerConfigurationException` if no stator current limit is set for a NEO 550, or if the limit exceeds 40 A.

**MAXMotion.** When a trapezoidal profile is configured and the onboard SPARK closed loop controller is used, YAMS translates the profile to MAXMotion position and velocity control. When the closed loop runs on the SystemCore instead (for example with an exponential profile or an [LQR controller](../math/lqr-controller.md)), YAMS raises a `closed loop controller is running on the SystemCore` alert.

**Simulation.** Uses `DCMotorSim` with the configured moment of inertia. The simulated SPARK closes its loop every 1 ms (1 kHz), like the real SPARK, stepping through each `withSimulationPeriod` period. Because REVLib's MAXMotion simulation does not follow the profile, YAMS runs the trapezoidal profile in `simIterate()` and has the simulated SPARK hold each point with position control. The `SimSupplier` can be replaced at runtime via `setSimSupplier`.

## See also

* [SmartMotorController](smart-motor-controller.md)
* [SmartMotorControllerConfig](smart-motor-controller-config.md)
* [NovaWrapper](nova-wrapper.md)
* [A301Wrapper](a301-wrapper.md)
* [TalonFXWrapper](talonfx-wrapper.md)
