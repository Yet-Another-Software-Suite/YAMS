# FlyWheel

Namespace: `yams::mechanisms::velocity`\
Header: `<yams/mechanisms/velocity/FlyWheel.hpp>`\
Inherits: `SmartVelocityMechanism`\
Doxygen: [`FlyWheel`](https://doxygen.yassrobotics.com/yams/classyams_1_1mechanisms_1_1velocity_1_1FlyWheel.html)

**Java equivalent:** [Java FlyWheel](../../java-reference/mechanisms/flywheel.md)

Controls a flywheel or other continuously spinning mechanism. Setpoint is angular velocity or wheel surface speed. Position tracking is not used.

{% hint style="info" %}
For intakes, rollers, feeders and indexers use a `FlyWheel` too, even when you only drive it open loop with `Set(...)` or `SetVoltage(...)`.
{% endhint %}

***

## Constructor

```cpp
FlyWheel(config::FlyWheelConfig* config, motorcontrollers::SmartMotorController* smc)
```

Both parameters are non-owning pointers. The `FlyWheelConfig` and `SmartMotorController` must outlive the `FlyWheel` instance.

***

## State queries

| Method        | Signature                                              | Description              |
| ------------- | ------------------------------------------------------ | ------------------------ |
| `GetVelocity` | `wpi::units::degrees_per_second_t GetVelocity() const` | Current angular velocity |

***

## Command factories

| Method                     | Signature                                                                                                                                            | Description                                                            |
| -------------------------- | ---------------------------------------------------------------------------------------------------------------------------------------------------- | ---------------------------------------------------------------------- |
| `Run` (angular)            | `wpi::cmd::CommandPtr Run(wpi::units::degrees_per_second_t velocity)`                                                                                | Runs indefinitely at `velocity`                                        |
| `Run` (angular supplier)   | `wpi::cmd::CommandPtr Run(std::function<wpi::units::degrees_per_second_t()> velocity)`                                                               | Supplier variant, re-evaluated each iteration                          |
| `Run` (linear)             | `wpi::cmd::CommandPtr Run(wpi::units::meters_per_second_t surfaceSpeed)`                                                                             | Controls by wheel surface speed                                        |
| `Run` (linear supplier)    | `wpi::cmd::CommandPtr Run(std::function<wpi::units::meters_per_second_t()> surfaceSpeed)`                                                            | Supplier variant                                                       |
| `RunTo` (angular)          | `wpi::cmd::CommandPtr RunTo(wpi::units::degrees_per_second_t velocity, wpi::units::degrees_per_second_t tolerance = 5.0_deg_per_s)`                  | Runs until within `tolerance` of `velocity`. Default tolerance: 5°/s.  |
| `RunTo` (angular supplier) | `wpi::cmd::CommandPtr RunTo(std::function<wpi::units::degrees_per_second_t()> velocity, wpi::units::degrees_per_second_t tolerance = 5.0_deg_per_s)` | Supplier variant                                                       |
| `RunTo` (linear)           | `wpi::cmd::CommandPtr RunTo(wpi::units::meters_per_second_t velocity, wpi::units::meters_per_second_t tolerance)`                                    | Surface speed variant; `tolerance` has no default and must be supplied |
| `RunTo` (linear supplier)  | `wpi::cmd::CommandPtr RunTo(std::function<wpi::units::meters_per_second_t()> velocity, wpi::units::meters_per_second_t tolerance)`                   | Surface speed supplier variant; `tolerance` must be supplied           |
| `Set` (duty cycle)         | `wpi::cmd::CommandPtr Set(double dutycycle)`                                                                                                         | Open loop duty cycle (inherited from `SmartMechanism`)                 |
| `Set` (supplier)           | `wpi::cmd::CommandPtr Set(std::function<double()> dutycycle)`                                                                                        | Open loop duty cycle with supplier                                     |
| `SetVoltage`               | `wpi::cmd::CommandPtr SetVoltage(wpi::units::volt_t volts)`                                                                                          | Open loop voltage                                                      |
| `SetVoltage` (supplier)    | `wpi::cmd::CommandPtr SetVoltage(std::function<wpi::units::volt_t()> volts)`                                                                         | Open loop voltage with supplier                                        |

{% hint style="info" %}
The C++ `FlyWheel` accepts both angular (`degrees_per_second_t`) and linear surface speed (`meters_per_second_t`) overloads. The linear overload requires `FlyWheelConfig::WithRollerDiameter()` to compute the radius for conversion.
{% endhint %}

***

## Triggers

| Method    | Signature                                                                                                                            | Description                                                      |
| --------- | ------------------------------------------------------------------------------------------------------------------------------------ | ---------------------------------------------------------------- |
| `IsNear`  | `wpi::cmd::Trigger IsNear(wpi::units::degrees_per_second_t velocity, wpi::units::degrees_per_second_t within = 5.0_deg_per_s) const` | True when current speed is within `within` of `velocity`         |
| `Gte`     | `wpi::cmd::Trigger Gte(wpi::units::degrees_per_second_t velocity)`                                                                   | True when speed ≥ threshold                                      |
| `Lte`     | `wpi::cmd::Trigger Lte(wpi::units::degrees_per_second_t velocity)`                                                                   | True when speed ≤ threshold                                      |
| `Between` | `wpi::cmd::Trigger Between(wpi::units::degrees_per_second_t start, wpi::units::degrees_per_second_t end)`                            | True when speed is in `[start, end]`                             |
| `Max`     | `wpi::cmd::Trigger Max() override`                                                                                                   | Always `false`: a flywheel has no velocity hard limit by default |
| `Min`     | `wpi::cmd::Trigger Min() override`                                                                                                   | Always `false`: a flywheel has no velocity hard limit by default |

{% hint style="info" %}
C++ keeps the `IsNear(...)` name for this trigger factory. The Java API renamed its trigger factory to `near(...)` in 2027; the C++ name did not change.
{% endhint %}

***

## Other public methods

| Method                 | Signature                                                     | Description                                                                                                                                                                           |
| ---------------------- | ------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `SetVelocity`          | `void SetVelocity(wpi::units::degrees_per_second_t velocity)` | Direct setpoint without command wrapper                                                                                                                                               |
| `SetSurfaceSpeed`      | `void SetSurfaceSpeed(wpi::units::meters_per_second_t speed)` | Surface speed setpoint; requires `WithRollerDiameter()` in config                                                                                                                     |
| `GetPose3d`            | `wpi::math::Pose3d GetPose3d() const`                         | The flywheel's axle, turned by the wheel's position, relative to the robot, from its [`MechanismPositionConfig`](../config/mechanism-position-config.md)                              |
| `GetConfig`            | `const config::FlyWheelConfig& GetConfig() const`             | Returns the configuration this flywheel was constructed with                                                                                                                          |
| `GetMotor`             | `motorcontrollers::SmartMotorController* GetMotor()`          | Returns the underlying motor controller pointer                                                                                                                                       |
| `SimIterate`           | `void SimIterate() override`                                  | Advances flywheel physics simulation one loop iteration                                                                                                                               |
| `UpdateTelemetry`      | `void UpdateTelemetry() override`                             | Publishes current state to NetworkTables                                                                                                                                              |
| `VisualizationUpdate`  | `void VisualizationUpdate() override`                         | Syncs Mechanism2d with the current state, and publishes the Mechanism2d window again (protected `SmartMechanism::PublishMechanismWindow()`) so it moves in the Sim GUI and dashboards |
| `GetMechanismLigament` | `wpi::MechanismLigament2d* GetMechanismLigament()`            | The Mechanism2d visualization ligament                                                                                                                                                |
| `GetMechanismRoot`     | `wpi::MechanismRoot2d* GetMechanismRoot()`                    | The Mechanism2d visualization root                                                                                                                                                    |
| `GetName`              | `std::string GetName() const override`                        | Returns the mechanism name                                                                                                                                                            |

***

## Inherited from SmartMechanism

Every mechanism also exposes the helpers declared in `yams::mechanisms::SmartMechanism` (`<yams/mechanisms/SmartMechanism.hpp>`):

| Method                           | Signature                                                                       | Description                                                        |
| -------------------------------- | ------------------------------------------------------------------------------- | ------------------------------------------------------------------ |
| `SetMechanismPositionSetpoint`   | `void SetMechanismPositionSetpoint(wpi::units::turn_t angle)`                   | Starts closed loop control and sets a mechanism angle setpoint     |
| `SetMeasurementPositionSetpoint` | `void SetMeasurementPositionSetpoint(wpi::units::meter_t distance)`             | Starts closed loop control and sets a linear position setpoint     |
| `SetMechanismVelocitySetpoint`   | `void SetMechanismVelocitySetpoint(wpi::units::turns_per_second_t velocity)`    | Starts closed loop control and sets a mechanism velocity setpoint  |
| `SetMeasurementVelocitySetpoint` | `void SetMeasurementVelocitySetpoint(wpi::units::meters_per_second_t velocity)` | Starts closed loop control and sets a linear velocity setpoint     |
| `SetVoltageSetpoint`             | `void SetVoltageSetpoint(wpi::units::volt_t voltage)`                           | Stops the closed loop controller and applies a voltage immediately |
| `SetDutyCycleSetpoint`           | `void SetDutyCycleSetpoint(double dutycycle)`                                   | Stops the closed loop controller and applies a duty cycle          |
| `GetMotorController`             | `motorcontrollers::SmartMotorController* GetMotorController()`                  | Returns the underlying motor controller                            |
| `GetMechanismSetpoint`           | `std::optional<wpi::units::turn_t> GetMechanismSetpoint()`                      | Current mechanism setpoint, if one has been set                    |
| `GetMechanismWindow`             | `wpi::Mechanism2d* GetMechanismWindow()`                                        | The Mechanism2d window, or `nullptr` if not initialized            |

***

## Example

```cpp
config::FlyWheelConfig flywheelConfig;
flywheelConfig.WithRollerDiameter(0.1_m)        // required for surface speed overloads
              .WithTelemetryName("Shooter");

// 2027: MOI lives on the SmartMotorControllerConfig.
motorcontrollers::SmartMotorControllerConfig motorConfig;
motorConfig.WithSubsystem(this)
           .WithMotorInverted(false)
           .WithMotorGearing(gearing::MechanismGearing{1.0})
           .WithFeedback(0.0, 0.0002, 0.0)
           .WithMOI(0.05_m, 0.5_kg)
           .WithClosedLoopMode();

// motor is a SmartMotorController* (for example a SparkWrapper) built from motorConfig
velocity::FlyWheel flywheel{&flywheelConfig, motor};

// Run at a fixed angular velocity:
wpi::cmd::CommandPtr spinUp = flywheel.RunTo(3600_deg_per_s, 50_deg_per_s);

// Or by surface speed (requires WithRollerDiameter in config):
wpi::cmd::CommandPtr spinUpLinear = flywheel.RunTo(15_mps, 0.3_mps);
```

{% @github-files/github-code-block url="https://github.com/Yet-Another-Software-Suite/YAMS/blob/master/examples/cpptest/src/main/cpp/subsystems/ShooterSubsystem.cpp#L45-L51" %}

***

## See also

* [Java FlyWheel](../../java-reference/mechanisms/flywheel.md)
* [C++ FlyWheelConfig](../config/flywheel-config.md)
