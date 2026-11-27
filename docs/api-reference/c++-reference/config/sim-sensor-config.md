# SimSensorConfig

**Namespace:** `yams::mechanisms::config`\
**Header:** `yams/mechanisms/config/SimSensorConfig.hpp`\
**Doxygen:** [`SimSensorConfig`](https://doxygen.yassrobotics.com/yams/classyams_1_1mechanisms_1_1config_1_1SimSensorConfig.html)

Builder for a simulated sensor that bridges real hardware I/O and the YAMS simulation framework. This is the C++ equivalent of the Java [`SensorConfig`](../../java-reference/config/sensor-config.md).

Register typed fields backed by live hardware suppliers, then inject override values during simulation by match-time window or an arbitrary trigger callable. The finished config resolves to a `Sensor` instance via `GetSensor()`, which performs real/simulated value arbitration at runtime.

## Constructor

```cpp
explicit SimSensorConfig(std::string name)
```

| Parameter | Description                                  |
| --------- | -------------------------------------------- |
| `name`    | Display name shown in the simulation window. |

## Builder methods: Field registration

Register each hardware value you want to expose. Four overloads accept `double`, `int`, `bool`, and `int64_t` suppliers.

```cpp
SimSensorConfig& WithField(const std::string& name, std::function<double()> supplier, double defaultVal)
SimSensorConfig& WithField(const std::string& name, std::function<int()> supplier, int defaultVal)
SimSensorConfig& WithField(const std::string& name, std::function<bool()> supplier, bool defaultVal)
SimSensorConfig& WithField(const std::string& name, std::function<int64_t()> supplier, int64_t defaultVal)
```

| Parameter    | Description                                                               |
| ------------ | ------------------------------------------------------------------------- |
| `name`       | Field identifier; must match the name used in `WithSimulatedValue` calls. |
| `supplier`   | Live hardware value source, called every loop iteration on a real robot.  |
| `defaultVal` | Value used before any simulation override fires.                          |

## Builder methods: Match-time simulated value injection

Override a field's value during a match-time window. Overloads exist for `double`, `int`, `int64_t`, and `bool`.

```cpp
SimSensorConfig& WithSimulatedValue(const std::string& fieldName,
                                    wpi::units::second_t start, wpi::units::second_t end,
                                    double value)
// ... int, int64_t, bool overloads follow the same signature
```

| Parameter   | Description                                             |
| ----------- | ------------------------------------------------------- |
| `fieldName` | Name of a previously registered field.                  |
| `start`     | Match time at which the simulated value becomes active. |
| `end`       | Match time at which the simulated value deactivates.    |
| `value`     | Value to inject while active.                           |

## Builder methods: Trigger-based simulated value injection

```cpp
SimSensorConfig& WithSimulatedValue(const std::string& fieldName,
                                    std::function<bool()> trigger,
                                    double value)
// ... int, int64_t, bool overloads follow the same signature
```

| Parameter | Description                                                                  |
| --------- | ---------------------------------------------------------------------------- |
| `trigger` | Evaluated every loop; the simulated value is active while it returns `true`. |

## Accessors

| Method        | Returns                   | Description                                              |
| ------------- | ------------------------- | -------------------------------------------------------- |
| `GetSensor()` | `Sensor&`                 | Lazily constructs and returns the `Sensor` instance.     |
| `GetName()`   | `const std::string&`      | The display name passed to the constructor.              |
| `GetFields()` | `std::vector<SensorData>` | The registered field list (used internally by `Sensor`). |

## Example

```cpp
#include <yams/mechanisms/config/SimSensorConfig.hpp>
#include <wpi/hardware/discrete/DigitalInput.hpp>

// Hardware limit switch on DIO 0
wpi::DigitalInput limitSwitch{0};

yams::mechanisms::config::SimSensorConfig forwardLimit{"ForwardLimitSwitch"};
forwardLimit
    .WithField("triggered", [&]() { return limitSwitch.Get(); }, false)
    // Simulate the switch being pressed from 10 s to 12 s into the match
    .WithSimulatedValue("triggered", 10_s, 12_s, true);

// Custom encoder exposing position and velocity
yams::mechanisms::config::SimSensorConfig encoderConfig{"ArmEncoder"};
encoderConfig
    .WithField("positionRotations", [&]() { return myEncoder.GetPosition(); }, 0.0)
    .WithField("velocityRPM",       [&]() { return myEncoder.GetVelocity(); }, 0.0)
    // Override position to 2.5 rotations whenever a test condition is active
    .WithSimulatedValue("positionRotations",
                        [&]() { return wpi::RobotBase::IsSimulation(); },
                        2.5);

// Get the ready-to-use Sensor reference (lazy construction on first call)
auto& sensor = encoderConfig.GetSensor();
```

{% hint style="info" %}
Multiple `WithSimulatedValue` calls on the same field are all registered. The first active trigger wins; subsequent triggers are evaluated in registration order.
{% endhint %}

{% hint style="warning" %}
`GetSensor()` is lazy: the `Sensor` instance is not constructed until the first call. Call it once after configuration is complete and store the returned reference.
{% endhint %}

## Related pages

* [SensorConfig (C++)](sensorconfig.md): why there is no separate C++ `SensorConfig`
* [SensorConfig (Java)](../../java-reference/config/sensor-config.md)
* [SmartMotorControllerConfig (C++)](../motor-controllers/smart-motor-controller-config.md)
