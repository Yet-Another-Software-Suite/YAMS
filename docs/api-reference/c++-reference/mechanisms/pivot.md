# Pivot

Namespace: `yams::mechanisms::positional`\
Header: `<yams/mechanisms/positional/Pivot.hpp>`\
Inherits: `SmartPositionalMechanism`\
Doxygen: [`Pivot`](https://doxygen.yassrobotics.com/yams/classyams_1_1mechanisms_1_1positional_1_1Pivot.html)

**Java equivalent:** [Java Pivot](../../java-reference/mechanisms/pivot.md)

Controls rotation in the XY plane (turret, shooter hood). For rotation in the XZ plane, use [Arm](arm.md).

***

## Constructor

```cpp
Pivot(config::PivotConfig* config, motorcontrollers::SmartMotorController* smc)
```

Both parameters are non-owning pointers. The `PivotConfig` and `SmartMotorController` must outlive the `Pivot` instance.

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
| `Run` (supplier)        | `wpi::cmd::CommandPtr Run(std::function<wpi::units::degree_t()> angle)`                                             | Supplier target, re-evaluated each iteration                            |
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
| `Max`     | `wpi::cmd::Trigger Max() override`                                                            | True when pivot is at its upper hard limit            |
| `Min`     | `wpi::cmd::Trigger Min() override`                                                            | True when pivot is at its lower hard limit            |

{% hint style="info" %}
C++ keeps the `IsNear(...)` name for this trigger factory. The Java API renamed its trigger factory to `near(...)` in 2027; the C++ name did not change.
{% endhint %}

***

## Other public methods

| Method                 | Signature                                            | Description                                                                                                                                                                                 |
| ---------------------- | ---------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `SetAngle`             | `void SetAngle(wpi::units::degree_t angle)`          | Directly commands the setpoint without a command wrapper                                                                                                                                    |
| `GetPose3d`            | `wpi::math::Pose3d GetPose3d() const`                | The pivot's axis, turned to its angle, relative to the robot, from its [`MechanismPositionConfig`](../config/mechanism-position-config.md)                                                  |
| `GetConfig`            | `const config::PivotConfig& GetConfig() const`       | Returns the configuration this pivot was constructed with                                                                                                                                   |
| `GetMotor`             | `motorcontrollers::SmartMotorController* GetMotor()` | Returns the underlying motor controller pointer                                                                                                                                             |
| `GetMechanismLigament` | `wpi::MechanismLigament2d* GetMechanismLigament()`   | The Mechanism2d visualization ligament                                                                                                                                                      |
| `GetMechanismRoot`     | `wpi::MechanismRoot2d* GetMechanismRoot()`           | The Mechanism2d visualization root                                                                                                                                                          |
| `SimIterate`           | `void SimIterate() override`                         | Advances pivot physics simulation one loop iteration                                                                                                                                        |
| `UpdateTelemetry`      | `void UpdateTelemetry() override`                    | Publishes current state to NetworkTables                                                                                                                                                    |
| `VisualizationUpdate`  | `void VisualizationUpdate() override`                | Syncs Mechanism2d with current encoder reading, and publishes the Mechanism2d window again (protected `SmartMechanism::PublishMechanismWindow()`) so it moves in the Sim GUI and dashboards |
| `GetName`              | `std::string GetName() const override`               | Returns the mechanism name                                                                                                                                                                  |

***

## Difference from Arm

`Pivot` and `Arm` have identical APIs. The distinction is the plane of rotation and the underlying physics model used for simulation and feedforward:

* `Arm`: rotates in the XZ plane, models gravity acting against the arm length
* `Pivot`: rotates in the XY plane, no gravity component along the rotation axis

Use `PivotConfig` with `Pivot` and `ArmConfig` with `Arm`.

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
config::PivotConfig pivotConfig;
pivotConfig.WithMinAngle(-180_deg)
           .WithMaxAngle(180_deg)
           .WithTelemetryName("Pivot");

// 2027: MOI and starting position live on the SmartMotorControllerConfig.
// motorConfig.WithStartingPosition(0_deg).WithMOI(0.3_m, 3.0_kg);
// motor is a SmartMotorController* (for example a TalonFXWrapper) built from motorConfig
positional::Pivot pivot{&pivotConfig, motor};

wpi::cmd::CommandPtr aimCmd = pivot.RunTo(30_deg);
```

{% hint style="warning" %}
In simulation the `Pivot` constructor throws `exceptions::PivotConfigurationException` if the minimum angle, maximum angle, starting position (`SmartMotorControllerConfig::WithStartingPosition`) or MOI (`SmartMotorControllerConfig::WithMOI`) is missing.
{% endhint %}

{% @github-files/github-code-block url="https://github.com/Yet-Another-Software-Suite/YAMS/blob/master/examples/cpptest/src/main/cpp/subsystems/TurretSubsystem.cpp" %}

***

## See also

* [Java Pivot](../../java-reference/mechanisms/pivot.md)
* [C++ PivotConfig](../config/pivot-config.md)
* [Arm](arm.md): rotation in the XZ plane
