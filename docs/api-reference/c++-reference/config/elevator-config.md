# ElevatorConfig

**Namespace:** `yams::mechanisms::config`\
**Header:** `<yams/mechanisms/config/ElevatorConfig.hpp>`\
**Java equivalent:** [Java ElevatorConfig](../../java-reference/config/elevator-config.md)\
**Doxygen:** [`ElevatorConfig`](https://doxygen.yassrobotics.com/yams/classyams_1_1mechanisms_1_1config_1_1ElevatorConfig.html)

`ElevatorConfig` holds physical and simulation parameters for an `Elevator` mechanism.

***

## Constructor

```cpp
ElevatorConfig() = default;
```

***

## Builder methods

All methods return `ElevatorConfig&` for chaining.

| Method                                                                                             | Description                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       |
| -------------------------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `WithTelemetryName(const std::string& name)`                                                       | Name used for SmartDashboard and log entries                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      |
| `WithMinimumHeight(wpi::units::meter_t height)`                                                    | **Required for simulation**, unless the stages are set with `WithStages(...)`. Minimum allowable carriage height                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  |
| `WithMaximumHeight(wpi::units::meter_t height)`                                                    | **Required for simulation**, unless the stages are set with `WithStages(...)`. Maximum allowable carriage height                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  |
| `WithCarriageMass(wpi::units::kilogram_t mass)`                                                    | **Required for simulation.** Mass of the carriage, used in simulation                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             |
| `WithIsHorizontal(bool horizontal)`                                                                | When `true`, disables gravity simulation. Useful for horizontal linear sliders                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    |
| `WithStages(Heights... stageHeights)`, `WithStages(std::vector<wpi::units::meter_t> stageHeights)` | The elevator's moving stages, such as `WithStages(0.6_m, 0.6_m, 0.5_m)`: one height for each stage that moves, how far it travels relative to the stage below it, from the stage nearest the fixed stage to the carriage. For `Elevator::GetCascadePose3d()` and `GetContinuousPose3d(bool)`. 1 stage is valid. The heights must add up to the maximum height, or the `Elevator` throws `ElevatorConfigurationException`; without a minimum and maximum height, the height is from 0 to the stage heights' total. Throws `std::invalid_argument` for no heights, or a height that is not positive |
| `WithSimColor(const wpi::util::Color8Bit& color)`                                                  | Color of the simulated elevator (default: orange)                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 |
| `WithAngle(wpi::units::degree_t angle)`                                                            | Angle of the elevator in its movement plane, for Mechanism2d and `GetPose3d()` (default: `90_deg`, vertical)                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      |
| `WithMechanismPositionConfig(const MechanismPositionConfig& config)`                               | Where the mechanism is on the robot, for `GetPose3d()`, and the plane it moves in. See [MechanismPositionConfig](mechanism-position-config.md)                                                                                                                                                                                                                                                                                                                                                                                                                                                    |

> **Note:** Drum or sprocket size is configured on `SmartMotorControllerConfig` via `.WithMechanismCircumference(circumference)` or `.WithMechanismCircumference(gearPitch, teeth)`, and the starting height via `.WithStartingPosition(wpi::units::meter_t)`. In simulation the `Elevator` constructor throws an `ElevatorConfigurationException` if the carriage mass, minimum height, maximum height, starting position or mechanism circumference is missing. Cascading elevator stages are configured via `SmartMotorControllerConfig::WithCascadingElevatorStages(int stages)`.

***

## Getters

```cpp
std::string GetTelemetryName() const;
std::optional<wpi::units::meter_t> GetMinHeight() const;  // or 0 when the stages are set without one
std::optional<wpi::units::meter_t> GetMaxHeight() const;  // or the stage heights' total without one
std::optional<int> GetStages() const;
const std::vector<wpi::units::meter_t>& GetStageHeights() const;
std::optional<wpi::units::meter_t> GetStageHeightTotal() const;
std::optional<wpi::units::kilogram_t> GetCarriageMass() const;
bool IsHorizontal() const;
wpi::util::Color8Bit GetSimColor() const;
wpi::units::degree_t GetAngle() const;
const MechanismPositionConfig& GetMechanismPositionConfig() const;
```

Getters for optional fields return `std::nullopt` if the corresponding setter was not called.

***

## Exceptions

| Exception                                    | Thrown When                                                                                                                      |
| -------------------------------------------- | -------------------------------------------------------------------------------------------------------------------------------- |
| `exceptions::ElevatorConfigurationException` | Required fields (on `ElevatorConfig` or the motor controller config) are missing when an `Elevator` is constructed in simulation |

***

## Example

```cpp
config::ElevatorConfig elevConfig;
elevConfig.WithMinimumHeight(0.0_m)
          .WithMaximumHeight(1.5_m)
          .WithCarriageMass(4.5_kg)
          .WithTelemetryName("Elevator");
```
