# Elevator

Namespace: `yams::mechanisms::positional`\
Header: `<yams/mechanisms/positional/Elevator.hpp>`\
Inherits: `SmartPositionalMechanism`\
Doxygen: [`Elevator`](https://doxygen.yassrobotics.com/yams/classyams_1_1mechanisms_1_1positional_1_1Elevator.html)

**Java equivalent:** [Java Elevator](../../java-reference/mechanisms/elevator.md)

Controls a linear elevator mechanism. Position is measured in meters (carriage height).

***

## Constructor

```cpp
Elevator(config::ElevatorConfig* config, motorcontrollers::SmartMotorController* smc)
```

Both parameters are non-owning pointers. The `ElevatorConfig` and `SmartMotorController` must outlive the `Elevator` instance.

***

## State queries

| Method                   | Signature                                                     | Description                                                    |
| ------------------------ | ------------------------------------------------------------- | -------------------------------------------------------------- |
| `GetHeight`              | `wpi::units::meter_t GetHeight() const`                       | Current carriage height                                        |
| `GetMeasurementSetpoint` | `std::optional<wpi::units::meter_t> GetMeasurementSetpoint()` | The closed loop position setpoint as a height, if there is one |

***

## Command factories

| Method                  | Signature                                                                                                         | Description                                                 |
| ----------------------- | ----------------------------------------------------------------------------------------------------------------- | ----------------------------------------------------------- |
| `Run` (target)          | `wpi::cmd::CommandPtr Run(wpi::units::meter_t height)`                                                            | Drives to `height`, runs indefinitely                       |
| `Run` (supplier)        | `wpi::cmd::CommandPtr Run(std::function<wpi::units::meter_t()> height)`                                           | Supplier target, re-evaluated each iteration                |
| `RunTo` (target)        | `wpi::cmd::CommandPtr RunTo(wpi::units::meter_t height, wpi::units::meter_t tolerance = 0.01_m)`                  | Drives to `height`, ends within `tolerance`. Default: 1 cm. |
| `RunTo` (supplier)      | `wpi::cmd::CommandPtr RunTo(std::function<wpi::units::meter_t()> height, wpi::units::meter_t tolerance = 0.01_m)` | `RunTo` with supplier                                       |
| `Set` (duty cycle)      | `wpi::cmd::CommandPtr Set(double dutycycle)`                                                                      | Open loop duty cycle                                        |
| `Set` (supplier)        | `wpi::cmd::CommandPtr Set(std::function<double()> dutycycle)`                                                     | Open loop duty cycle with supplier                          |
| `SetVoltage`            | `wpi::cmd::CommandPtr SetVoltage(wpi::units::volt_t volts)`                                                       | Open loop voltage output                                    |
| `SetVoltage` (supplier) | `wpi::cmd::CommandPtr SetVoltage(std::function<wpi::units::volt_t()> volts)`                                      | Open loop voltage with supplier                             |

***

## Triggers

| Method    | Signature                                                                                   | Description                                             |
| --------- | ------------------------------------------------------------------------------------------- | ------------------------------------------------------- |
| `IsNear`  | `wpi::cmd::Trigger IsNear(wpi::units::meter_t height, wpi::units::meter_t within = 0.01_m)` | True when current height is within `within` of `height` |
| `Gte`     | `wpi::cmd::Trigger Gte(wpi::units::meter_t height)`                                         | True when height ≥ threshold                            |
| `Lte`     | `wpi::cmd::Trigger Lte(wpi::units::meter_t height)`                                         | True when height ≤ threshold                            |
| `Between` | `wpi::cmd::Trigger Between(wpi::units::meter_t start, wpi::units::meter_t end)`             | True when height is in `[start, end]`                   |
| `Max`     | `wpi::cmd::Trigger Max() override`                                                          | True when elevator is at its upper hard limit           |
| `Min`     | `wpi::cmd::Trigger Min() override`                                                          | True when elevator is at its lower hard limit           |

{% hint style="info" %}
C++ keeps the `IsNear(...)` name for this trigger factory. The Java API renamed its trigger factory to `near(...)` in 2027; the C++ name did not change.
{% endhint %}

***

## Other public methods

| Method                 | Signature                                                                      | Description                                                                                                                                                                                                                                                                                                                               |
| ---------------------- | ------------------------------------------------------------------------------ | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `SetHeight`            | `void SetHeight(wpi::units::meter_t height)`                                   | Directly commands the setpoint without a command wrapper                                                                                                                                                                                                                                                                                  |
| `GetPose3d`            | `wpi::math::Pose3d GetPose3d() const`                                          | The carriage, moved by its height along the elevator's angle, relative to the robot, from its [`MechanismPositionConfig`](../config/mechanism-position-config.md)                                                                                                                                                                         |
| `GetCascadePose3d`     | `std::vector<wpi::math::Pose3d> GetCascadePose3d() const`                      | Each moving stage's pose for a cascade elevator, whose stages all move at once, each by its share of the carriage's height. From the stage nearest the fixed stage to the carriage, which is `GetPose3d()`. Needs 2 or more stages from `ElevatorConfig::WithStages(...)`, or throws `ElevatorConfigurationException`                     |
| `GetContinuousPose3d`  | `std::vector<wpi::math::Pose3d> GetContinuousPose3d(bool carriageFirst) const` | Each moving stage's pose for a continuous elevator, whose stages move one at a time. `true`: the carriage moves the whole of its travel first, then the stage below it, and so on. `false`: the stage nearest the fixed stage moves first, and the carriage last. In the same order and with the same requirement as `GetCascadePose3d()` |
| `GetConfig`            | `const config::ElevatorConfig& GetConfig() const`                              | Returns the configuration this elevator was constructed with                                                                                                                                                                                                                                                                              |
| `GetMotor`             | `motorcontrollers::SmartMotorController* GetMotor()`                           | Returns the underlying motor controller pointer                                                                                                                                                                                                                                                                                           |
| `GetMechanismLigament` | `wpi::MechanismLigament2d* GetMechanismLigament()`                             | The Mechanism2d visualization ligament                                                                                                                                                                                                                                                                                                    |
| `GetMechanismRoot`     | `wpi::MechanismRoot2d* GetMechanismRoot()`                                     | The Mechanism2d visualization root                                                                                                                                                                                                                                                                                                        |
| `SimIterate`           | `void SimIterate() override`                                                   | Advances elevator physics simulation one loop iteration                                                                                                                                                                                                                                                                                   |
| `UpdateTelemetry`      | `void UpdateTelemetry() override`                                              | Publishes current state to NetworkTables                                                                                                                                                                                                                                                                                                  |
| `VisualizationUpdate`  | `void VisualizationUpdate() override`                                          | Syncs Mechanism2d with current encoder reading, and publishes the Mechanism2d window again (protected `SmartMechanism::PublishMechanismWindow()`) so it moves in the Sim GUI and dashboards                                                                                                                                               |
| `GetName`              | `std::string GetName() const override`                                         | Returns the mechanism name                                                                                                                                                                                                                                                                                                                |

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
config::ElevatorConfig elevatorConfig;
elevatorConfig.WithMinimumHeight(0.0_m)
              .WithMaximumHeight(1.5_m)
              .WithCarriageMass(4.5_kg)
              .WithTelemetryName("Elevator");

// 2027: the starting position lives on the SmartMotorControllerConfig.
motorcontrollers::SmartMotorControllerConfig motorConfig;
motorConfig.WithSubsystem(this)
           .WithMotorInverted(false)
           .WithMechanismCircumference(0.0254_m, 22)  // chain pitch, sprocket teeth
           .WithMotorGearing(gearing::MechanismGearing{9.0})
           .WithFeedback(1.2, 0.0, 0.0)
           .WithStartingPosition(0.0_m)
           .WithClosedLoopMode();

// motor is a SmartMotorController* (for example a TalonFXWrapper) built from motorConfig
positional::Elevator elevator{&elevatorConfig, motor};

wpi::cmd::CommandPtr goToTop = elevator.RunTo(1.4_m);
```

{% hint style="warning" %}
In simulation the `Elevator` constructor throws `exceptions::ElevatorConfigurationException` if the carriage mass, minimum height, maximum height, starting position (`SmartMotorControllerConfig::WithStartingPosition(wpi::units::meter_t)`) or mechanism circumference (`SmartMotorControllerConfig::WithMechanismCircumference`) is missing.
{% endhint %}

{% @github-files/github-code-block url="https://github.com/Yet-Another-Software-Suite/YAMS/blob/master/examples/cpptest/src/main/cpp/subsystems/ElevatorSubsystem.cpp" %}

***

## See also

* [Java Elevator](../../java-reference/mechanisms/elevator.md)
* [C++ ElevatorConfig](../config/elevator-config.md)
