# ArmConfig

**Namespace:** `yams::mechanisms::config`\
**Header:** `<yams/mechanisms/config/ArmConfig.hpp>`\
**Java equivalent:** [Java ArmConfig](../../java-reference/config/arm-config.md)\
**Doxygen:** [`ArmConfig`](https://doxygen.yassrobotics.com/yams/classyams_1_1mechanisms_1_1config_1_1ArmConfig.html)

`ArmConfig` holds physical and simulation parameters for an `Arm` mechanism.

***

## Constructor

```cpp
ArmConfig() = default;
```

***

## Builder methods

All methods return `ArmConfig&` for chaining.

> **Required for simulation:** `WithArmLength()`, `WithMinAngle()` and `WithMaxAngle()` must be set, or the `Arm` constructor throws an `ArmConfigurationException` in simulation. The `SmartMotorControllerConfig` must also set a starting position (`WithStartingPosition`, or an external encoder zero offset) and an MOI (`WithMOI`). In 2027 MOI and starting position live on `SmartMotorControllerConfig`, not `ArmConfig`.

| Method                                                               | Required                    | Description                                                                                                                                    |
| -------------------------------------------------------------------- | --------------------------- | ---------------------------------------------------------------------------------------------------------------------------------------------- |
| `WithArmLength(wpi::units::meter_t length)`                          | **Required for simulation** | Arm length from pivot to end effector. Without this, the physics simulation cannot model the arm.                                              |
| `WithMinAngle(wpi::units::degree_t angle)`                           | **Required for simulation** | Minimum allowable mechanism angle; also applied as the motor controller lower limit                                                            |
| `WithMaxAngle(wpi::units::degree_t angle)`                           | **Required for simulation** | Maximum allowable mechanism angle; also applied as the motor controller upper limit                                                            |
| `WithTelemetryName(const std::string& name)`                         | Optional                    | Name used for SmartDashboard and log entries                                                                                                   |
| `WithSimColor(const wpi::util::Color8Bit& color)`                    | Optional                    | Color of the simulated arm ligament                                                                                                            |
| `WithMechanismPositionConfig(const MechanismPositionConfig& config)` | Optional                    | Where the mechanism is on the robot, for `GetPose3d()`, and the plane it moves in. See [MechanismPositionConfig](mechanism-position-config.md) |

***

## Getters

```cpp
std::string GetTelemetryName() const;
std::optional<wpi::units::degree_t> GetMinAngle() const;
std::optional<wpi::units::degree_t> GetMaxAngle() const;
std::optional<wpi::units::meter_t> GetArmLength() const;
wpi::util::Color8Bit GetSimColor() const;
const MechanismPositionConfig& GetMechanismPositionConfig() const;
```

Getters for optional fields return `std::nullopt` if the corresponding setter was not called.

***

## Exceptions

| Exception                               | Thrown When                                                                                                            |
| --------------------------------------- | ---------------------------------------------------------------------------------------------------------------------- |
| `exceptions::ArmConfigurationException` | Required fields (on `ArmConfig` or the motor controller config) are missing when an `Arm` is constructed in simulation |

***

## Example

```cpp
config::ArmConfig armConfig;
armConfig.WithMinAngle(-90_deg)
         .WithMaxAngle(90_deg)
         .WithArmLength(0.6_m)
         .WithTelemetryName("Arm");
```
