# SmartMotorControllerConfig

**Package:** `yams.commands2.config` (Commands v2) or `yams.commands3.config` (Commands v3)\
**Extends:** `yams.core.motorcontrollers.SmartMotorControllerConfig<T>`\
**Javadoc:** [`SmartMotorControllerConfig`](https://javadocs.yassrobotics.com/yams/yams/commands2/config/SmartMotorControllerConfig.html) (Commands v2), [`SmartMotorControllerConfig`](https://javadocs.yassrobotics.com/yams/yams/commands3/config/SmartMotorControllerConfig.html) (Commands v3)

Configuration class for [SmartMotorController](smart-motor-controller.md). All builder methods return the `SmartMotorControllerConfig` instance for chaining. Pass the completed config to a `SmartMotorController` wrapper constructor.

Robot code imports the config from the command framework package it uses. Both classes extend the abstract `yams.core.motorcontrollers.SmartMotorControllerConfig<T>`, which holds every builder method on this page. They only differ in what the config is bound to: a `Subsystem` in Commands v2, a `Mechanism` in Commands v3. The binding is used to register the `Live Tuning` command.

{% tabs %}
{% tab title="Commands v2" %}
```java
import yams.commands2.config.SmartMotorControllerConfig;
```
{% endtab %}

{% tab title="Commands v3" %}
```java
import yams.commands3.config.SmartMotorControllerConfig;
```
{% endtab %}
{% endtabs %}

## Enums

### ControlMode

`yams.core.motorcontrollers.enums.ControlMode`

| Value         | Description                                                                     |
| ------------- | ------------------------------------------------------------------------------- |
| `OPEN_LOOP`   | Motor is driven directly by duty cycle or voltage. The onboard PID is not used. |
| `CLOSED_LOOP` | Motor uses the configured PID controller (onboard or YAMS software). Default.   |

### MotorMode

`yams.core.motorcontrollers.enums.MotorMode`

| Value   | Description                                       |
| ------- | ------------------------------------------------- |
| `BRAKE` | Motor actively resists motion when not commanded. |
| `COAST` | Motor freewheels when not commanded.              |

### TelemetryVerbosity

`yams.core.telemetry.enums.TelemetryVerbosity`

| Value  | Description                                              |
| ------ | -------------------------------------------------------- |
| `LOW`  | Position and velocity only.                              |
| `MID`  | Adds current and voltage.                                |
| `HIGH` | Full state including temperature and feedforward values. |

## Constructors

{% tabs %}
{% tab title="Commands v2" %}
```java
SmartMotorControllerConfig()
SmartMotorControllerConfig(Subsystem subsystem)
```

The no-argument constructor is valid; call `withSubsystem(Subsystem)` before passing the config to a `SmartMotorController`.
{% endtab %}

{% tab title="Commands v3" %}
```java
SmartMotorControllerConfig()
SmartMotorControllerConfig(Mechanism mechanism)
```

`Mechanism` is `org.wpilib.command3.Mechanism`. The no-argument constructor is valid; call `withMechanism(Mechanism)` before passing the config to a `SmartMotorController`.
{% endtab %}
{% endtabs %}

## Builder methods: Motor settings

| Method                                          | Parameters      | Description                                                                                                  |
| ----------------------------------------------- | --------------- | ------------------------------------------------------------------------------------------------------------ |
| `withMotorInverted(boolean inverted)`           | `inverted`      | Reverses motor output direction.                                                                             |
| `withEncoderInverted(boolean inverted)`         | `inverted`      | Reverses the reported encoder direction.                                                                     |
| `withExternalEncoderInverted(boolean inverted)` | `inverted`      | Reverses the external encoder reading direction.                                                             |
| `withZeroPower(MotorMode mode)`                 | `mode`          | Sets coast or brake mode.                                                                                    |
| `withStatorCurrentLimit(Current limit)`         | `limit`         | Maximum motor output current.                                                                                |
| `withOutputCurrentLimit(Current stallCurrent)`  | `stallCurrent`  | Alias for `withStatorCurrentLimit`. Use when thinking in terms of output current rather than stator current. |
| `withSupplyCurrentLimit(Current limit)`         | `limit`         | Maximum current drawn from the battery.                                                                      |
| `withInputCurrentLimit(Current supplyCurrent)`  | `supplyCurrent` | Alias for `withSupplyCurrentLimit`. Use when thinking in terms of input current rather than supply current.  |
| `withVoltageCompensation(Voltage voltage)`      | `voltage`       | Enables voltage compensation; normalizes output to this battery voltage.                                     |
| `withTemperatureCutoff(Temperature temp)`       | `temp`          | Disables motor output when temperature exceeds this value.                                                   |
| `withOpenLoopRampRate(Time rate)`               | `rate`          | Time to ramp from 0 to full output in open loop mode.                                                        |
| `withClosedLoopRampRate(Time rate)`             | `rate`          | Time to ramp from 0 to full output in closed loop mode.                                                      |
| `withControlMode(ControlMode mode)`             | `mode`          | Selects `OPEN_LOOP` or `CLOSED_LOOP`.                                                                        |
| `withResetPreviousConfig(boolean reset)`        | `reset`         | When `true` (default), clears any previously burned configuration before applying this one.                  |

## Builder methods: Gearing & measurement

| Method                                               | Parameters            | Description                                                                                                                                                                                                                                                                                                                                                                         |
| ---------------------------------------------------- | --------------------- | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `withGearing(MechanismGearing gearing)`              | `gearing`             | Sets the motor-to-mechanism gear ratio. Affects all position and velocity scaling.                                                                                                                                                                                                                                                                                                  |
| `withGearing(double reductionRatio)`                 | `ratio`               | Shorthand for a single-stage reduction. A ratio of `3.0` means 3 motor rotations per mechanism rotation.                                                                                                                                                                                                                                                                            |
| `withMechanismCircumference(Distance circumference)` | `circumference`       | Converts rotational position/velocity to linear distance (e.g., for an elevator or drive wheel).                                                                                                                                                                                                                                                                                    |
| `withWheelRadius(Distance radius)`                   | `radius`              | Sets circumference from wheel radius: `2π × radius`.                                                                                                                                                                                                                                                                                                                                |
| `withWheelDiameter(Distance diameter)`               | `diameter`            | Sets circumference from wheel diameter: `π × diameter`.                                                                                                                                                                                                                                                                                                                             |
| `withDrumRadius(Distance radius)`                    | `radius`              | Alias for `withWheelRadius`. Intended for elevator drums.                                                                                                                                                                                                                                                                                                                           |
| `withDrumRadius(Distance chainPitch, int teeth)`     | `chainPitch`, `teeth` | Computes radius from chain pitch and sprocket tooth count.                                                                                                                                                                                                                                                                                                                          |
| `withCascadingElevatorStages(int stages)`            | `stages`              | Divides the configured gearing by `stages` to account for a cascading elevator.                                                                                                                                                                                                                                                                                                     |
| `withSoftLimits(Angle lower, Angle upper)`           | `lower`, `upper`      | Sets both mechanism soft limits. Lower must be less than upper.                                                                                                                                                                                                                                                                                                                     |
| `withSoftLimits(Distance low, Distance high)`        | `low`, `high`         | Sets soft limits in distance units. Requires `withMechanismCircumference`.                                                                                                                                                                                                                                                                                                          |
| `withExternalEncoderZeroOffset(Angle offset)`        | `offset`              | Applies a zero offset to the external encoder reading. Negative values are automatically wrapped. Read it back with `getExternalEncoderZeroOffset()` (formerly `getZeroOffset()`).                                                                                                                                                                                                  |
| `withExternalEncoderZeroOffset(Distance distance)`   | `distance`            | Zero offset in distance units. Requires `withMechanismCircumference`.                                                                                                                                                                                                                                                                                                               |
| `withWrapping(Angle min, Angle max)`                 | `min`, `max`          | Sets the wrapping range: one rotation an absolute external encoder reads, such as `Rotations.of(-0.5)` to `Rotations.of(0.5)` or `Rotations.of(0)` to `Rotations.of(1)`. The closed loop does not wrap, so it never takes the mechanism the other way round. Can be combined with soft limits. See [Wrapping](smart-motor-controller-config.md#wrapping).                           |
| `withContinuousWrapping(Angle min, Angle max)`       | `min`, `max`          | Sets the same wrapping range as `withWrapping`, and makes the closed loop take the shortest path to a setpoint, which can be the other way round. For a swerve azimuth or turret. Incompatible with soft limits (in either order) and distance-based mechanisms. Requires `withClosedLoopController` to be called first. See [Wrapping](smart-motor-controller-config.md#wrapping). |
| `withClosedLoopTolerance(Angle tolerance)`           | `tolerance`           | Sets the position tolerance for the PID controller.                                                                                                                                                                                                                                                                                                                                 |
| `withClosedLoopTolerance(Distance tolerance)`        | `tolerance`           | Sets position tolerance in distance units. Requires `withLinearClosedLoopController(true)`.                                                                                                                                                                                                                                                                                         |
| `withFeedbackSynchronizationThreshold(Angle angle)`  | `angle`               | Relative encoder is re-seeded from absolute encoder when drift exceeds this angle. Incompatible with distance-based mechanisms.                                                                                                                                                                                                                                                     |

## Builder methods: Closed loop control

| Method                                                                                            | Parameters    | Description                                                                                                                                        |
| ------------------------------------------------------------------------------------------------- | ------------- | -------------------------------------------------------------------------------------------------------------------------------------------------- |
| `withClosedLoopController(double kP, double kI, double kD)`                                       | N/A           | Sets position PID gains for SLOT\_0. Units: rotations in, volts out (meters in if linear).                                                         |
| `withClosedLoopController(double kP, double kI, double kD, ClosedLoopControllerSlot slot)`        | N/A           | Same as above for a specific slot.                                                                                                                 |
| `withClosedLoopController(PIDController controller)`                                              | `controller`  | Attaches an existing `PIDController` to SLOT\_0.                                                                                                   |
| `withClosedLoopController(PIDController controller, ClosedLoopControllerSlot slot)`               | N/A           | Same, for a specific slot.                                                                                                                         |
| `withClosedLoopController(LQRController controller)`                                              | `controller`  | Replaces the PID with an [LQR controller](../math/lqr-controller.md). Clears all PID slots.                                                        |
| `withClosedLoopControlPeriod(Time time)`                                                          | `time`        | Period of the software closed loop thread. Default: 20 ms.                                                                                         |
| `withClosedLoopControlPeriod(Frequency freq)`                                                     | `freq`        | Sets the period from a frequency.                                                                                                                  |
| `withClosedLoopControllerMaximumVoltage(Voltage volts)`                                           | `volts`       | Clamps the closed loop voltage output to ±`volts`.                                                                                                 |
| `withLinearClosedLoopController(boolean linear)`                                                  | `linear`      | When `true`, the controller operates in meters instead of rotations.                                                                               |
| `withFeedforward(SimpleMotorFeedforward ff)`                                                      | `ff`          | Attaches a `SimpleMotorFeedforward` (kS, kV, kA) to SLOT\_0.                                                                                       |
| `withFeedforward(SimpleMotorFeedforward ff, ClosedLoopControllerSlot slot)`                       | N/A           | Same, for a specific slot.                                                                                                                         |
| `withFeedforward(ArmFeedforward ff)`                                                              | `ff`          | Attaches an `ArmFeedforward` (kS, kG, kV, kA) to SLOT\_0. Feedforward is gravity-compensated as a function of angle.                               |
| `withFeedforward(ArmFeedforward ff, ClosedLoopControllerSlot slot)`                               | N/A           | Same, for a specific slot.                                                                                                                         |
| `withFeedforward(ElevatorFeedforward ff)`                                                         | `ff`          | Attaches an `ElevatorFeedforward` (kS, kG, kV, kA) to SLOT\_0. Enables linear closed loop mode.                                                    |
| `withFeedforward(ElevatorFeedforward ff, ClosedLoopControllerSlot slot)`                          | N/A           | Same, for a specific slot.                                                                                                                         |
| `withExponentialProfile(ExponentialProfile.Constraints constraints)`                              | `constraints` | Enables an exponential motion profile.                                                                                                             |
| `withExponentialProfile(Voltage maxVolts, DCMotor motor, MomentOfInertia moi)`                    | N/A           | Derives exponential profile constraints from motor and MOI (single-jointed arm).                                                                   |
| `withExponentialProfile(Voltage maxVolts, DCMotor motor, Mass mass, Distance drumRadius)`         | N/A           | Derives exponential profile constraints for an elevator. Enables linear mode.                                                                      |
| `withExponentialProfile(Voltage maxVolts, AngularVelocity maxVel, AngularAcceleration maxAccel)`  | N/A           | Derives exponential profile constraints from velocity and acceleration bounds.                                                                     |
| `withTrapezoidalProfile(AngularVelocity maxVel, AngularAcceleration maxAccel)`                    | N/A           | Enables a trapezoidal motion profile for angular position control.                                                                                 |
| `withTrapezoidalProfile(LinearVelocity maxVel, LinearAcceleration maxAccel)`                      | N/A           | Trapezoidal profile for linear position control. Enables linear mode.                                                                              |
| `withTrapezoidalProfile(AngularAcceleration maxAccel, Velocity<AngularAccelerationUnit> maxJerk)` | N/A           | Trapezoidal profile for angular velocity control.                                                                                                  |
| `withTrapezoidalProfile(LinearAcceleration maxAccel, Velocity<LinearAccelerationUnit> maxJerk)`   | N/A           | Trapezoidal profile for linear velocity control.                                                                                                   |
| `withVelocityTrapezoidalProfile(boolean enabled)`                                                 | `enabled`     | Marks the trapezoidal profile as a velocity profile explicitly.                                                                                    |
| `withTrapezoidalProfileConstraints(TrapezoidProfile.Constraints constraints)`                     | `constraints` | Sets trapezoidal constraints directly, in rotations or meters per second (per second squared for a velocity profile).                              |
| `withProfile(TrapezoidProfile.Constraints profile)`                                               | `profile`     | Sets a trapezoidal profile from raw constraints (rotations/s and rotations/s², or meters for a linear controller). Clears any exponential profile. |
| `withProfile(ExponentialProfile.Constraints profile)`                                             | `profile`     | Sets an exponential profile from raw constraints. Clears any trapezoidal profile.                                                                  |

{% hint style="info" %}
`withClosedLoopController` must be called before `withContinuousWrapping` or `withClosedLoopTolerance`. The PID controller must exist to configure those options. `withWrapping` does not need one.
{% endhint %}

## Builder methods: Simulation overrides

These methods set values used only when running in simulation. If no sim-specific value is set, the real-hardware value is used in simulation as well.

| Method                                                                                        | Parameters   | Description                                                                                                     |
| --------------------------------------------------------------------------------------------- | ------------ | --------------------------------------------------------------------------------------------------------------- |
| `withSimClosedLoopController(double kP, double kI, double kD)`                                | N/A          | PID gains used only in simulation.                                                                              |
| `withSimClosedLoopController(double kP, double kI, double kD, ClosedLoopControllerSlot slot)` | N/A          | Same, for a specific slot.                                                                                      |
| `withSimClosedLoopController(PIDController controller)`                                       | `controller` | `PIDController` used only in simulation.                                                                        |
| `withSimClosedLoopController(PIDController controller, ClosedLoopControllerSlot slot)`        | N/A          | Same, for a specific slot.                                                                                      |
| `withSimClosedLoopController(LQRController controller)`                                       | `controller` | [LQR controller](../math/lqr-controller.md) used only in simulation.                                            |
| `withSimFeedforward(ArmFeedforward ff)`                                                       | `ff`         | Arm feedforward used only in simulation.                                                                        |
| `withSimFeedforward(ElevatorFeedforward ff)`                                                  | `ff`         | Elevator feedforward used only in simulation.                                                                   |
| `withSimFeedforward(SimpleMotorFeedforward ff)`                                               | `ff`         | Simple feedforward used only in simulation.                                                                     |
| `withSimFeedforward(<feedforward> ff, ClosedLoopControllerSlot slot)`                         | N/A          | Each `withSimFeedforward` overload also takes a slot.                                                           |
| `withSimulationPeriod(Time time)`                                                             | `time`       | Period at which the simulated physics is stepped, independent of `withClosedLoopControlPeriod`. Default: 20 ms. |
| `withSimStartingPosition(Angle angle)`                                                        | `angle`      | Seeds the simulated encoder to this angle at startup (simulation only).                                         |
| `withSimStartingPosition(Distance distance)`                                                  | `distance`   | Seeds from a linear distance (simulation only).                                                                 |

## Builder methods: External encoder

| Method                                                 | Parameters       | Description                                                                                                                                                                                                                                                                                                                                                                                                                                     |
| ------------------------------------------------------ | ---------------- | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `withExternalEncoder(Object encoder)`                  | `encoder`        | Attaches an external encoder. SPARK MAX/Flex: the SPARK's absolute encoder, the SPARK MAX alternate encoder, the SPARK Flex external encoder, or a REVLib `DetachedEncoder`. TalonFX/TalonFXS: a `CANcoder` or `CANdi`. Thrifty Nova: `FeedbackSensorType.ABS`, `FeedbackSensorType.QUAD` or a `CanEncoder` (see [NovaWrapper](nova-wrapper.md)). FIRST A301: the `A301` itself, for its absolute encoder (see [A301Wrapper](a301-wrapper.md)). |
| `withExternalEncoderGearing(MechanismGearing gearing)` | `gearing`        | Gear ratio between the external encoder and the mechanism.                                                                                                                                                                                                                                                                                                                                                                                      |
| `withExternalEncoderGearing(double reductionRatio)`    | `reductionRatio` | Shorthand for a single reduction, e.g. `3` for 3:1 or `0.5` for 1:2.                                                                                                                                                                                                                                                                                                                                                                            |
| `withUseExternalFeedbackEncoder(boolean use)`          | `use`            | When `true` (default), the external encoder is used as the PID feedback source.                                                                                                                                                                                                                                                                                                                                                                 |

## Wrapping

`withWrapping(min, max)` and `withContinuousWrapping(min, max)` both set the range an absolute external encoder reads, one rotation from `min` up to `max`. Only `withContinuousWrapping` makes the closed loop wrap (PID continuous input, Talon `ClosedLoopGeneral.ContinuousWrap`, SPARK position wrapping, Nova `absWrapping`). Use `withWrapping` on an arm whose encoder must read across zero, and `withContinuousWrapping` on a mechanism that can turn all the way round.

```java
// Arm: the encoder reads [-0.5, 0.5) rotations, soft limits still apply
.withWrapping(Rotations.of(-0.5), Rotations.of(0.5))
.withSoftLimits(Degrees.of(-30), Degrees.of(100))

// Turret or swerve azimuth: the closed loop takes the shortest path
.withClosedLoopController(4, 0, 0)
.withContinuousWrapping(Rotations.of(-0.5), Rotations.of(0.5))
```

Each vendor limits the range an absolute encoder can read; any other range throws `SmartMotorControllerConfigurationException` when the config is applied:

| Encoder                            | Accepted wrapping range                                                                                                                                                                     | Without wrapping         |
| ---------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ------------------------ |
| `CANcoder`, `CANdi` PWM            | A maximum from 0 to 1 rotation, sent as the encoder's `AbsoluteSensorDiscontinuityPoint`.                                                                                                   | The encoder's own range. |
| `SparkAbsoluteEncoder`             | One rotation from \[-1, 0) to \[0, 1). REVLib's range offset is the middle of the range, `(min + max) / 2`.                                                                                 | \[0, 1)                  |
| REVLib `DetachedEncoder`           | \[-0.5, 0.5) (zero centered) or \[0, 1).                                                                                                                                                    | \[0, 1)                  |
| Thrifty Nova `ABS` or `CanEncoder` | Any range. For the absolute encoder closing the loop, \[0, 1), or any range with continuous wrapping and 1:1 external encoder gearing, runs on the Nova; any other range on the SystemCore. | \[0, 1)                  |
| FIRST A301 absolute encoder        | Any range, applied by the wrapper.                                                                                                                                                          | \[0, 1)                  |

Getters: `getWrappingMax()` and `getWrappingMin()` (formerly `getContinuousWrapping()` and `getContinuousWrappingMin()`) return the range, or empty without wrapping. `isWrappingContinuous()` says whether the closed loop wraps, and throws `SmartMotorControllerConfigurationException` if continuous wrapping and soft limits are both set. `getContinuousWrappingSetpoint(setpoint, current)` returns the setpoint unchanged unless the wrapping is continuous.

{% hint style="info" %}
`withExternalEncoderDiscontinuityPoint` is removed. Replace `.withExternalEncoderDiscontinuityPoint(Rotations.of(0.5))` with `.withWrapping(Rotations.of(-0.5), Rotations.of(0.5))`, and `Rotations.of(1)` with `.withWrapping(Rotations.of(0), Rotations.of(1))`.
{% endhint %}

## Builder methods: Simulation model

| Method                                            | Parameters       | Description                                                                         |
| ------------------------------------------------- | ---------------- | ----------------------------------------------------------------------------------- |
| `withMomentOfInertia(Distance length, Mass mass)` | `length`, `mass` | Estimates MOI for simulation using `SingleJointedArmSim.estimateMOI(length, mass)`. |
| `withMomentOfInertia(MomentOfInertia moi)`        | `moi`            | Sets MOI directly.                                                                  |
| `withStartingPosition(Angle angle)`               | `angle`          | Seeds the encoder to this angle at startup (real hardware and simulation).          |
| `withStartingPosition(Distance distance)`         | `distance`       | Seeds from a linear distance.                                                       |

## Builder methods: Subsystem/Mechanism & telemetry

| Method                                                                            | Parameters             | Description                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         |
| --------------------------------------------------------------------------------- | ---------------------- | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `withSubsystem(Subsystem subsystem)`                                              | `subsystem`            | Commands v2 only. Associates this motor with a WPILib `Subsystem`. Must be called exactly once if not provided in the constructor.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  |
| `withMechanism(Mechanism mechanism)`                                              | `mechanism`            | Commands v3 only. Associates this motor with a Commands v3 `Mechanism`. Must be called exactly once if not provided in the constructor.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             |
| `withTelemetry(String name, TelemetryVerbosity verbosity)`                        | N/A                    | Enables NetworkTables telemetry. Publishes under `Mechanisms/<name>`.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                               |
| `withTelemetry(TelemetryVerbosity verbosity)`                                     | `verbosity`            | Enables telemetry with the default name `"motor"`.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  |
| `withTelemetry(String name, SmartMotorControllerTelemetryConfig telemetryConfig)` | N/A                    | Enables telemetry with a custom field selection. Sets the verbosity to `HIGH`.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      |
| `setupLiveTuning()`                                                               | N/A                    | Registers the `Live Tuning` command for the motor controller this config is attached to, on the config's `Subsystem` (v2) or `Mechanism` (v3). Called automatically at `TelemetryVerbosity.HIGH`; call it yourself to opt in at other verbosities. Returns `void`.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  |
| `withFollowers(Pair<Object, Boolean>... followers)`                               | `followers`            | Adds native follower motors (varargs). Each `Pair` contains the vendor motor controller object and an inversion boolean relative to the leader. Followers must be the same vendor as the leader. An A301 is followed by `A301Wrapper`s instead (see [A301Wrapper](a301-wrapper.md#several-a301s-on-one-mechanism)).                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 |
| `withLooselyCoupledFollowers(SmartMotorController... followers)`                  | `followers`            | Adds `SmartMotorController` followers on separate shafts, each with its own config and closed loop. They are sent the leader's position and velocity setpoints and live tuning changes, but not its configuration, other duty cycles or voltages, or `setEncoderPosition`. Stopping the leader's closed loop, as a mechanism does before driving it at a duty cycle or voltage, stops theirs and sets them to a duty cycle of 0, and starting it starts theirs. Call `updateTelemetry()` on each follower yourself. In a mechanism class, such as an `Arm`, the followers are simulated with the leader: from its `getMechanismDCMotor()`, with each motor controller's own output, and the mechanism iterates them. Without one, call `simIterate()` on each follower too. See [Simulation](smart-motor-controller.md#simulation). |
| `withVendorConfig(Object vendorConfig)`                                           | `vendorConfig`         | Provides a vendor-specific base configuration. YAMS options applied via this class always take precedence and overwrite it.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         |
| `withVendorControlRequest(Object vendorControlRequest)`                           | `vendorControlRequest` | Provides a vendor-specific control request object that overrides YAMS default control requests for velocity and position.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           |

## Feedforward conversion

| Method                                         | Returns   | Description                                                                                                                                                                                                                                                                                                                                                                                       |
| ---------------------------------------------- | --------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `convertToVoltage(DCMotor motor, Force force)` | `Voltage` | Converts a feedforward `Force` applied at the mechanism into the motor voltage that produces it (the resistive drop of the matching current), using the configured `withGearing`/`withMechanismCircumference`. It does not include the back-EMF of the commanded speed, because it is added on top of a velocity feedforward that already covers that. Used for voltage-mode closed loop control. |
| `convertToCurrent(DCMotor motor, Force force)` | `Current` | Same conversion, but returns the equivalent motor current instead of voltage. Used for torque-current-mode closed loop control (e.g. TorqueCurrentFOC).                                                                                                                                                                                                                                           |

These back `SmartMotorController.setVelocity(velocity, Force feedforwardForce)` (see [SmartMotorController](smart-motor-controller.md#control)). A `SwerveModule`'s drive motor uses them to turn a PathPlanner setpoint generator's per-wheel `Force` into the right feedforward for whichever control mode that motor is using.

{% hint style="warning" %}
Both methods throw `SmartMotorControllerConfigurationException` if no mechanism circumference was configured with `withMechanismCircumference(...)` (or `withWheelRadius(...)`/`withWheelDiameter(...)`), including for a `Force` of exactly zero.
{% endhint %}

## See also

* [SmartMotorController](smart-motor-controller.md)
* [SparkWrapper](spark-wrapper.md)
* [NovaWrapper](nova-wrapper.md)
* [A301Wrapper](a301-wrapper.md)
* [TalonFXWrapper](talonfx-wrapper.md)
