# PivotConfig

**Namespace:** `yams::mechanisms::config`\
**Header:** `<yams/mechanisms/config/PivotConfig.hpp>`\
**Java equivalent:** [Java PivotConfig](../../java-reference/config/pivot-config.md)\
**Doxygen:** [`PivotConfig`](https://doxygen.yassrobotics.com/yams/classyams_1_1mechanisms_1_1config_1_1PivotConfig.html)

`PivotConfig` holds physical and simulation parameters for a `Pivot` mechanism. A pivot is a single-jointed rotational mechanism without a load-bearing length (use `ArmConfig` if arm length affects simulation accuracy).

***

## Constructor

```cpp
PivotConfig() = default;
```

***

## Builder methods

All methods return `PivotConfig&` for chaining.

> **Required for simulation:** `WithMinAngle()` and `WithMaxAngle()` must be set, or the `Pivot` constructor throws a `PivotConfigurationException` in simulation. A pivot does not use arm length in its physics model. The `SmartMotorControllerConfig` must also set a starting position (`WithStartingPosition`) and an MOI (`WithMOI`); in 2027 both live on `SmartMotorControllerConfig`.

| Method                                                               | Required                    | Description                                                                                                                                                   |
| -------------------------------------------------------------------- | --------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `WithTelemetryName(const std::string& name)`                         | Optional                    | Name used for SmartDashboard and log entries                                                                                                                  |
| `WithMinAngle(wpi::units::degree_t angle)`                           | **Required for simulation** | Minimum allowable mechanism angle; also applied as the motor controller lower limit                                                                           |
| `WithMaxAngle(wpi::units::degree_t angle)`                           | **Required for simulation** | Maximum allowable mechanism angle; also applied as the motor controller upper limit                                                                           |
| `WithSimColor(const wpi::util::Color8Bit& color)`                    | Optional                    | Color of the simulated pivot ligament. Defaults to green if not set.                                                                                          |
| `WithMechanismPositionConfig(const MechanismPositionConfig& config)` | Optional                    | Where the mechanism is on the robot, for `GetPose3d()`, and the plane it moves in, XY unless set. See [MechanismPositionConfig](mechanism-position-config.md) |

***

## Getters

```cpp
std::string GetTelemetryName() const;
std::optional<wpi::units::degree_t> GetMinAngle() const;
std::optional<wpi::units::degree_t> GetMaxAngle() const;
wpi::util::Color8Bit GetSimColor() const;
const MechanismPositionConfig& GetMechanismPositionConfig() const;
```

Getters for optional fields return `std::nullopt` if the corresponding setter was not called. `GetSimColor()` always returns a value (defaults to green).

***

## Exceptions

| Exception                                 | Thrown When                                                                                                               |
| ----------------------------------------- | ------------------------------------------------------------------------------------------------------------------------- |
| `exceptions::PivotConfigurationException` | Required fields (on `PivotConfig` or the motor controller config) are missing when a `Pivot` is constructed in simulation |

***

## Example

```cpp
config::PivotConfig pivotConfig;
pivotConfig.WithMinAngle(0_deg)
           .WithMaxAngle(180_deg)
           .WithTelemetryName("Wrist");
```
