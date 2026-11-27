# Exceptions

**Namespace:** `yams::exceptions`\
**Header:** `#include <yams/exceptions.hpp>`\
**Java equivalent:** [Java Exceptions](../java-reference/exceptions.md)\
**Doxygen:** [`yams::exceptions`](https://doxygen.yassrobotics.com/yams/namespaceyams_1_1exceptions.html)

All YAMS C++ exceptions extend `yams::exceptions::YamsException`, which extends `std::runtime_error`.

***

## Base exception

```cpp
class YamsException : public std::runtime_error {
public:
    explicit YamsException(const std::string& message);
};
```

***

## Exception classes

| Class                                        | Thrown When                                                  |
| -------------------------------------------- | ------------------------------------------------------------ |
| `ArmConfigurationException`                  | Required `ArmConfig` fields are missing or invalid           |
| `ElevatorConfigurationException`             | Required `ElevatorConfig` fields are missing or invalid      |
| `PivotConfigurationException`                | Required `PivotConfig` fields are missing or invalid         |
| `SwerveDriveConfigurationException`          | Required swerve drive config fields are missing or invalid   |
| `SmartMotorControllerConfigurationException` | `SmartMotorControllerConfig` is invalid                      |
| `NoStagesGivenException`                     | `GearBox` or `Sprocket` constructed with an empty stage list |
| `InvalidStageGivenException`                 | A stage string is not in `"IN:OUT"` format                   |

***

## Constructor patterns

### Configuration exceptions

Most configuration exceptions follow this three-argument pattern:

```cpp
SomeConfigurationException(const std::string& issue,
                           const std::string& result,
                           const std::string& fix)
```

| Parameter | Description                                         |
| --------- | --------------------------------------------------- |
| `issue`   | Description of the invalid or missing configuration |
| `result`  | The runtime consequence of the misconfiguration     |
| `fix`     | Suggested corrective action                         |

### SwerveDriveConfigurationException

```cpp
explicit SwerveDriveConfigurationException(const std::string& message)
```

Takes a single message string rather than the three-argument pattern.

### Gearing exceptions

```cpp
NoStagesGivenException()
explicit InvalidStageGivenException(const std::string& stage)
```

***

## Example

```cpp
try {
    // motor is a SmartMotorController (e.g. a SparkWrapper or TalonFXWrapper)
    yams::mechanisms::positional::Arm arm{&armConfig, &motor};
} catch (const yams::exceptions::ArmConfigurationException& e) {
    fmt::print("Arm config error: {}\n", e.what());
} catch (const yams::exceptions::YamsException& e) {
    fmt::print("YAMS error: {}\n", e.what());
}
```

{% hint style="info" %}
All YAMS exceptions are unchecked (derived from `std::runtime_error`). They are thrown eagerly at mechanism construction time when required config fields are absent, not deferred to the first control call.
{% endhint %}
