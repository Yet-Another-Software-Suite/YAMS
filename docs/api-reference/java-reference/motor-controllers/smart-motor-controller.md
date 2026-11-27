# SmartMotorController

**Package:** `yams.core.motorcontrollers`\
**Javadoc:** [`SmartMotorController`](https://javadocs.yassrobotics.com/yams/yams/core/motorcontrollers/SmartMotorController.html)

Abstract base class for all motor controller wrappers. Do not instantiate directly; construct a concrete subclass (`SparkWrapper`, `NovaWrapper`, `A301Wrapper`, `TalonFXWrapper`, or `TalonFXSWrapper`) and assign it to a `SmartMotorController` variable. Configure all implementations via [SmartMotorControllerConfig](smart-motor-controller-config.md).

`SmartMotorController` is the same class for Commands v2 and Commands v3 robot code. Only the config you pass to the wrapper differs: `yams.commands2.config.SmartMotorControllerConfig` (bound to a `Subsystem`) or `yams.commands3.config.SmartMotorControllerConfig` (bound to a `Mechanism`).

## Construction

Instantiate the vendor-specific subclass directly:

```java
import yams.commands2.config.SmartMotorControllerConfig; // or yams.commands3.config
import yams.core.motorcontrollers.SmartMotorController;
import yams.core.motorcontrollers.local.SparkWrapper;
import yams.core.motorcontrollers.remote.TalonFXWrapper;

SmartMotorControllerConfig config = new SmartMotorControllerConfig(this)
    .withMotorInverted(false)
    .withGearing(new MechanismGearing(GearBox.fromTeeth(14, 72)))
    .withClosedLoopController(0.5, 0.0, 0.0)
    .withFeedforward(new ArmFeedforward(0.1, 0.5, 0.01))
    .withSoftLimits(Degrees.of(-90), Degrees.of(90));

// REV SPARK MAX
SmartMotorController motor = new SparkWrapper(
    new SparkMax(CANPort.CAN_S0, 1, MotorType.kBrushless),
    DCMotor.getNEO(1),
    config
);

// CTRE TalonFX
SmartMotorController motor = new TalonFXWrapper(
    new TalonFX(1, new CANBus(CANPort.CAN_S0)),
    DCMotor.getKrakenX60(1),
    config
);
```

## Utility

| Method                          | Returns   | Description                                                                                                                                                                                              |
| ------------------------------- | --------- | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `isMotor(DCMotor a, DCMotor b)` | `boolean` | Compares two `DCMotor` instances by all physical parameters. Useful for runtime motor type checks.                                                                                                       |
| `close()`                       | `void`    | Stops the closed loop thread, removes this controller from `BatterySim`, closes its telemetry and alerts, and runs any registered close hooks (such as the live tuning registration). Use in unit tests. |

## Control

| Method                                                          | Returns   | Description                                                                                                                                                                                                                                                                  |
| --------------------------------------------------------------- | --------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `setDutyCycle(double dutyCycle)`                                | `void`    | Sets output duty cycle in `[-1, 1]`.                                                                                                                                                                                                                                         |
| `getDutyCycle()`                                                | `double`  | Returns current duty cycle output.                                                                                                                                                                                                                                           |
| `setVoltage(Voltage voltage)`                                   | `void`    | Sets output voltage directly. Useful for SysId.                                                                                                                                                                                                                              |
| `getVoltage()`                                                  | `Voltage` | Returns current voltage output.                                                                                                                                                                                                                                              |
| `setPosition(Angle angle)`                                      | `void`    | Commands the mechanism to the given angular position using configured PID and feedforward.                                                                                                                                                                                   |
| `setPosition(Distance distance)`                                | `void`    | Commands the mechanism to the given linear position. Requires `withMechanismCircumference` to be set.                                                                                                                                                                        |
| `setVelocity(AngularVelocity velocity)`                         | `void`    | Commands the mechanism to the given angular velocity.                                                                                                                                                                                                                        |
| `setVelocity(LinearVelocity velocity)`                          | `void`    | Commands the mechanism to the given linear velocity.                                                                                                                                                                                                                         |
| `setVelocity(AngularVelocity velocity, Force feedforwardForce)` | `void`    | Commands the mechanism to the given angular velocity, with an additional feedforward `Force` applied on top (e.g. from a PathPlanner setpoint generator). Requires `withGearing`/`withMechanismCircumference` to be set. Pass `null` to fall back to plain velocity control. |
| `setVelocity(LinearVelocity velocity, Force feedforwardForce)`  | `void`    | Linear-velocity counterpart of the above.                                                                                                                                                                                                                                    |

{% hint style="info" %}
`Force` here means the drive-wheel feedforward force from a setpoint generator such as PathPlanner's `DriveFeedforwards.linearForces()`, not a torque or a raw voltage. `SmartMotorControllerConfig.convertToVoltage(...)`/`convertToCurrent(...)` convert it into the correct motor-level feedforward using the configured gearing and mechanism circumference (wheel radius); see [SmartMotorControllerConfig](smart-motor-controller-config.md#feedforward-conversion). A `null` force is treated identically to calling the no-force overload: no feedforward is applied, and `getSetpointFeedforwardForce()` reads back empty.
{% endhint %}

## Measurement

<table><thead><tr><th>Method</th><th>Returns</th><th width="250">Description</th></tr></thead><tbody><tr><td><code>getMechanismPosition()</code></td><td><code>Angle</code></td><td>Mechanism-space angle after gearing is applied. Uses the external encoder when <code>withUseExternalFeedbackEncoder(true)</code> is set, otherwise the relative encoder.</td></tr><tr><td><code>getMechanismVelocity()</code></td><td><code>AngularVelocity</code></td><td>Mechanism-space angular velocity after gearing.</td></tr><tr><td><code>getRelativeMechanismPosition()</code></td><td><code>Angle</code></td><td>Mechanism angle from the motor's relative encoder, scaled by the configured gearing. Never uses the external encoder.</td></tr><tr><td><code>getRelativeMechanismVelocity()</code></td><td><code>AngularVelocity</code></td><td>Mechanism angular velocity from the motor's relative encoder. Never uses the external encoder.</td></tr><tr><td><code>getMechanismAcceleration()</code></td><td><code>AngularAcceleration</code></td><td>Mechanism-space angular acceleration.</td></tr><tr><td><code>getMeasurementPosition()</code></td><td><code>Distance</code></td><td>Linear position. Requires <code>withMechanismCircumference</code>.</td></tr><tr><td><code>getMeasurementVelocity()</code></td><td><code>LinearVelocity</code></td><td>Linear velocity. Requires <code>withMechanismCircumference</code>.</td></tr><tr><td><code>getMeasurementAcceleration()</code></td><td><code>LinearAcceleration</code></td><td>Linear acceleration. Requires <code>withMechanismCircumference</code>.</td></tr><tr><td><code>getRotorPosition()</code></td><td><code>Angle</code></td><td>Raw rotor-encoder position scaled to mechanism rotations.</td></tr><tr><td><code>getRotorVelocity()</code></td><td><code>AngularVelocity</code></td><td>Raw rotor-encoder angular velocity.</td></tr><tr><td><code>getExternalEncoderMechanismPosition()</code></td><td><code>Optional&#x3C;Angle></code></td><td>Mechanism angle from the external encoder, scaled by the external encoder gearing. Available whether or not the external encoder is used for feedback; empty if none is attached.</td></tr><tr><td><code>getExternalEncoderMechanismVelocity()</code></td><td><code>Optional&#x3C;AngularVelocity></code></td><td>Mechanism angular velocity from the external encoder; empty if none is attached.</td></tr><tr><td><code>getMechanismPositionSetpoint()</code></td><td><code>Optional&#x3C;Angle></code></td><td>Current position setpoint, if set.</td></tr><tr><td><code>getMechanismSetpointVelocity()</code></td><td><code>Optional&#x3C;AngularVelocity></code></td><td>Current velocity setpoint, if set.</td></tr><tr><td><code>getSetpointFeedforwardForce()</code></td><td><code>Optional&#x3C;Force></code></td><td>Feedforward force last supplied to <code>setVelocity(velocity, Force)</code>. Empty if the no-force overload (or a <code>null</code> force) was used most recently.</td></tr></tbody></table>

{% hint style="info" %}
`getExternalEncoderPosition()` and `getExternalEncoderVelocity()` were renamed to `getExternalEncoderMechanismPosition()` and `getExternalEncoderMechanismVelocity()`. Use `getRelativeMechanismPosition()` / `getRelativeMechanismVelocity()` when you need the motor encoder reading even while the external encoder is the feedback source.
{% endhint %}

## Electrical

| Method               | Returns             | Description                                               |
| -------------------- | ------------------- | --------------------------------------------------------- |
| `getStatorCurrent()` | `Current`           | Motor output current (stator).                            |
| `getSupplyCurrent()` | `Optional<Current>` | Battery-side current draw. Not available on all hardware. |
| `getTemperature()`   | `Temperature`       | Motor controller temperature.                             |

## Encoder

| Method                                         | Returns | Description                                                                                            |
| ---------------------------------------------- | ------- | ------------------------------------------------------------------------------------------------------ |
| `setEncoderPosition(Angle angle)`              | `void`  | Seeds the relative encoder to the given mechanism angle.                                               |
| `setEncoderPosition(Distance distance)`        | `void`  | Seeds the relative encoder from a linear position.                                                     |
| `setEncoderVelocity(AngularVelocity velocity)` | `void`  | Override the reported angular velocity (simulation use).                                               |
| `setEncoderVelocity(LinearVelocity velocity)`  | `void`  | Override the reported linear velocity (simulation use).                                                |
| `seedRelativeEncoder()`                        | `void`  | Seeds the relative encoder from the absolute/external encoder.                                         |
| `synchronizeRelativeEncoder()`                 | `void`  | Checks whether the relative encoder has drifted beyond the configured threshold and re-seeds it if so. |

## Closed loop controller

| Method                                                                                                         | Returns                    | Description                                                                                                                                                                                                                                                                               |
| -------------------------------------------------------------------------------------------------------------- | -------------------------- | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `startClosedLoopController()`                                                                                  | `void`                     | Starts the background closed loop control thread, and the loosely coupled followers'. Only a stopped one is started, from where the mechanism is now; a running one carries on, so calling it every loop is safe. The mechanism classes' position and velocity setpoints call it for you. |
| `isClosedLoopControllerStopped()`                                                                              | `boolean`                  | Whether a closed loop run on the SystemCore, such as an LQR or an exponential profile, was stopped, as a duty cycle or voltage command does. `false` when the closed loop runs on the motor controller.                                                                                   |
| `stopClosedLoopController()`                                                                                   | `void`                     | Stops the closed loop control thread.                                                                                                                                                                                                                                                     |
| `getClosedLoopControllerPeriod()`                                                                              | `Time`                     | How often the closed loop runs: `withClosedLoopControlPeriod(...)`, 20 ms by default, when it runs on the SystemCore, or 1 ms when it runs on the motor controller. Published as `closedloop/period`.                                                                                     |
| `iterateClosedLoopController()`                                                                                | `void`                     | Executes one closed loop iteration. Called automatically by the background thread. While the robot is disabled it sends no voltage, and resets its PID, motion profile and LQR to where the mechanism is, so enabling starts from there.                                                  |
| `setClosedLoopSlot(ClosedLoopControllerSlot slot)`                                                             | `void`                     | Selects the active PID slot.                                                                                                                                                                                                                                                              |
| `setKp(double kP)` / `setKi(double kI)` / `setKd(double kD)`                                                   | `void`                     | Sets one PID gain at runtime.                                                                                                                                                                                                                                                             |
| `setFeedback(double kP, double kI, double kD)`                                                                 | `void`                     | Sets all PID gains at runtime.                                                                                                                                                                                                                                                            |
| `setKs(double)` / `setKv(double)` / `setKa(double)` / `setKg(double)`                                          | `void`                     | Sets one feedforward gain at runtime.                                                                                                                                                                                                                                                     |
| `setFeedforward(double kS, double kV, double kA, double kG)`                                                   | `void`                     | Sets all feedforward gains at runtime.                                                                                                                                                                                                                                                    |
| `setMotionProfileMaxVelocity(AngularVelocity)` / `setMotionProfileMaxVelocity(LinearVelocity)`                 | `void`                     | Updates the motion profile cruise velocity at runtime.                                                                                                                                                                                                                                    |
| `setMotionProfileMaxAcceleration(AngularAcceleration)` / `setMotionProfileMaxAcceleration(LinearAcceleration)` | `void`                     | Updates the motion profile acceleration at runtime.                                                                                                                                                                                                                                       |
| `setMotionProfileMaxJerk(Velocity<AngularAccelerationUnit> maxJerk)`                                           | `void`                     | Updates the motion profile jerk at runtime.                                                                                                                                                                                                                                               |
| `setExponentialProfile(OptionalDouble kV, OptionalDouble kA, Optional<Voltage> maxInput)`                      | `void`                     | Updates the exponential profile constraints at runtime.                                                                                                                                                                                                                                   |
| `getClosedLoopControllerSlot()`                                                                                | `ClosedLoopControllerSlot` | Returns the active PID slot.                                                                                                                                                                                                                                                              |

### ClosedLoopControllerSlot

| Value    | Description  |
| -------- | ------------ |
| `SLOT_0` | Default slot |
| `SLOT_1` | Second slot  |
| `SLOT_2` | Third slot   |
| `SLOT_3` | Fourth slot  |

## Configuration

| Method                                              | Returns                      | Description                                                                                                                                                 |
| --------------------------------------------------- | ---------------------------- | ----------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `applyConfig(SmartMotorControllerConfig config)`    | `boolean`                    | Applies a new configuration to the controller. Returns `true` on success.                                                                                   |
| `getConfig()`                                       | `SmartMotorControllerConfig` | Returns the active configuration.                                                                                                                           |
| `checkConfigSafety()`                               | `void`                       | Validates safety constraints. Throws `SmartMotorControllerConfigurationException` for unsafe configurations (e.g., NEO 550 without a stator current limit). |
| `setZeroPower(MotorMode mode)`                      | `void`                       | Sets coast or brake mode at runtime.                                                                                                                        |
| `setMotorInverted(boolean inverted)`                | `void`                       | Sets motor output inversion at runtime.                                                                                                                     |
| `setEncoderInverted(boolean inverted)`              | `void`                       | Sets encoder phase inversion at runtime.                                                                                                                    |
| `setStatorCurrentLimit(Current limit)`              | `void`                       | Sets stator current limit at runtime.                                                                                                                       |
| `setSupplyCurrentLimit(Current limit)`              | `void`                       | Sets supply current limit at runtime.                                                                                                                       |
| `setOpenLoopRampRate(Time rampRate)`                | `void`                       | Sets the open loop ramp rate at runtime.                                                                                                                    |
| `setClosedLoopRampRate(Time rampRate)`              | `void`                       | Sets the closed loop ramp rate at runtime.                                                                                                                  |
| `setMeasurementUpperLimit(Distance upper)`          | `void`                       | Sets the upper soft limit in distance units at runtime.                                                                                                     |
| `setMeasurementLowerLimit(Distance lower)`          | `void`                       | Sets the lower soft limit in distance units at runtime.                                                                                                     |
| `setMechanismUpperLimit(Angle upper)`               | `void`                       | Sets upper soft limit at runtime.                                                                                                                           |
| `setMechanismLowerLimit(Angle lower)`               | `void`                       | Sets lower soft limit at runtime.                                                                                                                           |
| `setMechanismLimits(Angle lower, Angle upper)`      | `void`                       | Sets both soft limits at runtime.                                                                                                                           |
| `setMechanismLimitsEnabled(boolean enabled)`        | `void`                       | Enables or disables soft limit enforcement.                                                                                                                 |
| `setMechanismGearing(MechanismGearing gearing)`     | `void`                       | Updates the gear ratio at runtime.                                                                                                                          |
| `setMechanismCircumference(Distance circumference)` | `void`                       | Updates the mechanism circumference at runtime.                                                                                                             |

## Telemetry

| Method                                                        | Returns                                                                             | Description                                                                                                                                                  |
| ------------------------------------------------------------- | ----------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| `setupTelemetry()`                                            | `void`                                                                              | Registers this motor under the default `Mechanisms` and `Tuning` NetworkTables. At `TelemetryVerbosity.HIGH` it also calls the config's `setupLiveTuning()`. |
| `setupTelemetry(NetworkTable telemetry, NetworkTable tuning)` | `void`                                                                              | Registers under the provided NetworkTables.                                                                                                                  |
| `updateTelemetry()`                                           | `void`                                                                              | Publishes current state to NetworkTables. Call from `periodic()`.                                                                                            |
| `getName()`                                                   | `String`                                                                            | Returns the telemetry name, or `"SmartMotorController"` if none is set.                                                                                      |
| `tuningEnabled()`                                             | `boolean`                                                                           | Whether live tuning values are pulled from NetworkTables.                                                                                                    |
| `applyTuningValues()`                                         | `void`                                                                              | Applies the values from the `Tuning` table to this controller. Run every loop by the `Live Tuning` command.                                                  |
| `getUnsupportedTelemetryFields()`                             | `Pair<Optional<List<BooleanTelemetryField>>, Optional<List<DoubleTelemetryField>>>` | Telemetry fields this implementation cannot report.                                                                                                          |

### Live tuning

At `TelemetryVerbosity.HIGH`, `setupTelemetry()` registers a `Live Tuning` command for the motor through the config's `setupLiveTuning()`: a Commands v2 `Command` on the config's `Subsystem` (`yams.commands2.config.SmartMotorControllerConfig`), or a Commands v3 `Command` on the config's `Mechanism` (`yams.commands3.config.SmartMotorControllerConfig`). Start it from the dashboard and the values edited under `Tuning/<name>` are applied every loop. At other verbosities, call `config.setupLiveTuning()` yourself after the motor controller is created.

## Simulation

| Method                                          | Returns                 | Description                                                                                                                                                                                                                                                      |
| ----------------------------------------------- | ----------------------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `setupSimulation()`                             | `void`                  | Initialises the simulation model. Called once on construction.                                                                                                                                                                                                   |
| `simIterate()`                                  | `void`                  | Advances the simulation by one loop iteration. Call from `simulationPeriodic()`.                                                                                                                                                                                 |
| `getSimSupplier()`                              | `Optional<SimSupplier>` | Returns the attached simulation supplier.                                                                                                                                                                                                                        |
| `setSimSupplier(SimSupplier supplier)`          | `void`                  | Attaches or replaces the simulation supplier.                                                                                                                                                                                                                    |
| `setMechanismSimSupplier(SimSupplier supplier)` | `void`                  | Attaches the simulation of the mechanism this controller drives, made with `getMechanismDCMotor()`. Each loosely coupled follower reads it, and its current is not counted again by `BatterySim`. The mechanism classes call it.                                 |
| `getMechanismDutyCycle(double dutyCycle)`       | `double`                | The duty cycle the motors of this controller's simulation apply. In a mechanism's simulation, each motor controller's own duty cycle weighted by its share of `getMechanismDCMotor()`'s torque; otherwise the given duty cycle. The simulation suppliers use it. |

In a mechanism class, such as an `Arm`, the leader's loosely coupled followers are simulated with it:

* The mechanism's simulation is made from the leader's `getMechanismDCMotor()`, which has every motor.
* It applies each motor controller's own output, as its vendor simulation limits it, weighted by its share of the torque: `getMechanismDutyCycle(double)`.
* The leader's `simIterate()`, which the mechanism calls right after it steps, iterates the followers, and each follower's encoder reads its simulation. A follower's own `simIterate()` does nothing while it reads a mechanism's simulation, so it is not iterated twice in a loop.

Without a mechanism class, each follower simulates its own motor and load, and you call its `simIterate()`.

`simIterate()` also reports this controller's simulated current draw into [`BatterySim`](../sensor/battery-sim.md) under a UUID unique to this instance (`m_batterySimUUID`), which combines the draw of every simulated `SmartMotorController` on the robot into one shared, loaded battery voltage.

While the robot is disabled, the `ArmSimSupplier`, `DCMotorSimSupplier` and `ElevatorSimSupplier` apply 0 V to the simulated motor on every step, so a mechanism falls or coasts as an unpowered one would, instead of holding its last voltage.

## Introspection

| Method                       | Returns   | Description                                                                                                                                                                                         |
| ---------------------------- | --------- | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `getMotorController()`       | `Object`  | Returns the underlying vendor motor controller object.                                                                                                                                              |
| `getMotorControllerConfig()` | `Object`  | Returns the vendor-specific configuration object generated by YAMS.                                                                                                                                 |
| `getDCMotor()`               | `DCMotor` | Returns the `DCMotor` model.                                                                                                                                                                        |
| `getMechanismDCMotor()`      | `DCMotor` | Returns the `DCMotor` of every motor on the mechanism: `getDCMotor()` and each loosely coupled follower's, with a follower's torque given at this motor's rotor. The mechanism classes simulate it. |

## Examples

{% @github-files/github-code-block url="https://github.com/Yet-Another-Software-Suite/YAMS/blob/master/examples/commands2/advantage_kit/java/first/robot/subsystems/ArmSubsystem.java#L114-L147" %}

## See also

* [SmartMotorControllerConfig](smart-motor-controller-config.md)
* [SparkWrapper](spark-wrapper.md)
* [NovaWrapper](nova-wrapper.md)
* [A301Wrapper](a301-wrapper.md)
* [TalonFXWrapper](talonfx-wrapper.md)
* [BatterySim](../sensor/battery-sim.md)
* [C++ SmartMotorController](../../c++-reference/motor-controllers/smart-motor-controller.md)
