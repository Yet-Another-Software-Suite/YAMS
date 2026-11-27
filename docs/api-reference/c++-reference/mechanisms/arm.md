# Arm

Namespace: `yams::mechanisms::positional`\
Header: `<yams/mechanisms/positional/Arm.hpp>`\
Inherits: `SmartPositionalMechanism`\
Doxygen: [`Arm`](https://doxygen.yassrobotics.com/yams/classyams_1_1mechanisms_1_1positional_1_1Arm.html)

**Java equivalent:** [Java Arm](../../java-reference/mechanisms/arm.md)

Controls a single-jointed arm rotating in the XZ plane. For rotation in the XY plane (turret, shooter hood), use [Pivot](pivot.md).

***

## Constructor

```cpp
Arm(config::ArmConfig* config, motorcontrollers::SmartMotorController* smc)
```

Both parameters are non-owning pointers. The `ArmConfig` and `SmartMotorController` must outlive the `Arm` instance.

***

## State queries

| Method     | Signature                               | Description                              |
| ---------- | --------------------------------------- | ---------------------------------------- |
| `GetAngle` | `wpi::units::degree_t GetAngle() const` | Current mechanism angle from the encoder |

***

## Command factories

| Method                  | Signature                                                                                                           | Description                                                             |
| ----------------------- | ------------------------------------------------------------------------------------------------------------------- | ----------------------------------------------------------------------- |
| `Run` (target)          | `wpi::cmd::CommandPtr Run(wpi::units::degree_t angle)`                                                              | Drives to `angle`, runs indefinitely                                    |
| `Run` (supplier)        | `wpi::cmd::CommandPtr Run(std::function<wpi::units::degree_t()> angle)`                                             | Drives to a supplier-provided angle, re-evaluated each iteration        |
| `RunTo` (target)        | `wpi::cmd::CommandPtr RunTo(wpi::units::degree_t angle, wpi::units::degree_t tolerance = 1.0_deg)`                  | Drives to `angle`, ends when within `tolerance`. Default tolerance: 1°. |
| `RunTo` (supplier)      | `wpi::cmd::CommandPtr RunTo(std::function<wpi::units::degree_t()> angle, wpi::units::degree_t tolerance = 1.0_deg)` | `RunTo` with a supplier target                                          |
| `Set` (duty cycle)      | `wpi::cmd::CommandPtr Set(double dutycycle)`                                                                        | Open loop duty cycle                                                    |
| `Set` (supplier)        | `wpi::cmd::CommandPtr Set(std::function<double()> dutycycle)`                                                       | Open loop duty cycle with supplier                                      |
| `SetVoltage`            | `wpi::cmd::CommandPtr SetVoltage(wpi::units::volt_t volts)`                                                         | Open loop voltage                                                       |
| `SetVoltage` (supplier) | `wpi::cmd::CommandPtr SetVoltage(std::function<wpi::units::volt_t()> volts)`                                        | Open loop voltage with supplier                                         |

***

## Triggers

| Method    | Signature                                                                                     | Description                                           |
| --------- | --------------------------------------------------------------------------------------------- | ----------------------------------------------------- |
| `IsNear`  | `wpi::cmd::Trigger IsNear(wpi::units::degree_t angle, wpi::units::degree_t within = 1.0_deg)` | True when current angle is within `within` of `angle` |
| `Gte`     | `wpi::cmd::Trigger Gte(wpi::units::degree_t angle)`                                           | True when current angle ≥ `angle`                     |
| `Lte`     | `wpi::cmd::Trigger Lte(wpi::units::degree_t angle)`                                           | True when current angle ≤ `angle`                     |
| `Between` | `wpi::cmd::Trigger Between(wpi::units::degree_t start, wpi::units::degree_t end)`             | True when angle is in `[start, end]`                  |
| `Max`     | `wpi::cmd::Trigger Max() override`                                                            | True when arm is at its upper hard limit              |
| `Min`     | `wpi::cmd::Trigger Min() override`                                                            | True when arm is at its lower hard limit              |

{% hint style="info" %}
C++ keeps the `IsNear(...)` name for this trigger factory. The Java API renamed its trigger factory to `near(...)` in 2027; the C++ name did not change.
{% endhint %}

***

## Other public methods

| Method                 | Signature                                            | Description                                                                                                                                                                                 |
| ---------------------- | ---------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `SetAngle`             | `void SetAngle(wpi::units::degree_t angle)`          | Directly commands the setpoint without a command wrapper                                                                                                                                    |
| `GetPose3d`            | `wpi::math::Pose3d GetPose3d() const`                | The arm's axis, turned to its angle, relative to the robot, from its [`MechanismPositionConfig`](../config/mechanism-position-config.md)                                                    |
| `GetConfig`            | `const config::ArmConfig& GetConfig() const`         | Returns the configuration this arm was constructed with                                                                                                                                     |
| `GetMotor`             | `motorcontrollers::SmartMotorController* GetMotor()` | Returns the underlying motor controller pointer                                                                                                                                             |
| `GetMechanismLigament` | `wpi::MechanismLigament2d* GetMechanismLigament()`   | The Mechanism2d visualization ligament                                                                                                                                                      |
| `GetMechanismRoot`     | `wpi::MechanismRoot2d* GetMechanismRoot()`           | The Mechanism2d visualization root                                                                                                                                                          |
| `SimIterate`           | `void SimIterate() override`                         | Advances arm physics simulation one loop iteration                                                                                                                                          |
| `UpdateTelemetry`      | `void UpdateTelemetry() override`                    | Publishes current state to NetworkTables                                                                                                                                                    |
| `VisualizationUpdate`  | `void VisualizationUpdate() override`                | Syncs Mechanism2d with current encoder reading, and publishes the Mechanism2d window again (protected `SmartMechanism::PublishMechanismWindow()`) so it moves in the Sim GUI and dashboards |
| `GetName`              | `std::string GetName() const override`               | Returns the mechanism name                                                                                                                                                                  |

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
config::ArmConfig armConfig;
armConfig.WithMinAngle(-90_deg)
         .WithMaxAngle(90_deg)
         .WithArmLength(0.6_m)
         .WithTelemetryName("Arm");

// 2027: MOI and starting position live on the SmartMotorControllerConfig.
motorcontrollers::SmartMotorControllerConfig motorConfig;
motorConfig.WithSubsystem(this)
           .WithMotorInverted(false)
           .WithMotorGearing(gearing::MechanismGearing{
               gearing::GearBox::FromReductionStages({3.0, 4.0})})
           .WithFeedback(0.5, 0.0, 0.0)
           .WithStartingPosition(0_deg)
           .WithMOI(0.6_m, 2.0_kg)
           .WithClosedLoopMode();

// motor is a SmartMotorController* (for example a TalonFXWrapper) built from motorConfig
positional::Arm arm{&armConfig, motor};

// In the subsystem command factory:
wpi::cmd::CommandPtr setAngleCmd = arm.RunTo(45_deg);
```

{% hint style="warning" %}
In simulation the `Arm` constructor throws `exceptions::ArmConfigurationException` if the arm length, minimum angle, maximum angle, starting position (`SmartMotorControllerConfig::WithStartingPosition`, or an external encoder zero offset) or MOI (`SmartMotorControllerConfig::WithMOI`) is missing.
{% endhint %}

{% @github-files/github-code-block url="https://github.com/Yet-Another-Software-Suite/YAMS/blob/master/examples/cpptest/src/main/cpp/subsystems/ArmSubsystem.cpp#L64-L78" %}

***

## See also

* [Java Arm](../../java-reference/mechanisms/arm.md)
* [C++ ArmConfig](../config/arm-config.md)
* [Pivot](pivot.md): rotation in the XY plane
