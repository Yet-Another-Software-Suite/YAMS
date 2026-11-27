# FlyWheelConfig

**Namespace:** `yams::mechanisms::config`\
**Header:** `<yams/mechanisms/config/FlyWheelConfig.hpp>`\
**Java equivalent:** [Java FlyWheelConfig](../../java-reference/config/flywheel-config.md)\
**Doxygen:** [`FlyWheelConfig`](https://doxygen.yassrobotics.com/yams/classyams_1_1mechanisms_1_1config_1_1FlyWheelConfig.html)

`FlyWheelConfig` holds physical and simulation parameters for a `FlyWheel` mechanism.

{% hint style="info" %}
**Simulation Note:** Moment of inertia is not configured on `FlyWheelConfig`. Set it via `SmartMotorControllerConfig::WithMOI()`.
{% endhint %}

***

## Constructor

```cpp
FlyWheelConfig() = default;
```

***

## Builder methods

All methods return `FlyWheelConfig&` for chaining.

| Method                                                               | Description                                                                                                                                    |
| -------------------------------------------------------------------- | ---------------------------------------------------------------------------------------------------------------------------------------------- |
| `WithTelemetryName(const std::string& name)`                         | Name used for SmartDashboard and log entries                                                                                                   |
| `WithRollerDiameter(wpi::units::meter_t diameter)`                   | Roller/flywheel outer diameter, used for surface speed calculations                                                                            |
| `WithSimColor(const wpi::util::Color8Bit& color)`                    | Color of the simulated flywheel (default: orange)                                                                                              |
| `WithMechanismPositionConfig(const MechanismPositionConfig& config)` | Where the mechanism is on the robot, for `GetPose3d()`, and the plane it moves in. See [MechanismPositionConfig](mechanism-position-config.md) |

***

## Getters

```cpp
std::string GetTelemetryName() const;
std::optional<wpi::units::meter_t> GetRollerDiameter() const;
wpi::util::Color8Bit GetSimColor() const;
const MechanismPositionConfig& GetMechanismPositionConfig() const;
```

Getters for optional fields return `std::nullopt` if the corresponding setter was not called.

***

## Example

```cpp
config::FlyWheelConfig flywheelConfig;
flywheelConfig.WithRollerDiameter(0.1016_m)
              .WithTelemetryName("Shooter");
```
