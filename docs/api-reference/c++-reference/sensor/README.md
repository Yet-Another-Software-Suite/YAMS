# Sensor and simulation

**Namespace:** `yams::motorcontrollers::simulation`\
**Header:** `#include <yams/motorcontrollers/simulation/Sensor.hpp>`\
**Doxygen:** [`Sensor`](https://doxygen.yassrobotics.com/yams/classyams_1_1motorcontrollers_1_1simulation_1_1Sensor.html)

A `Sensor` models a named hardware sensor whose fields are readable from robot code and, in simulation, visible in the WPILib **Glass** GUI. Every field delegates to a real hardware supplier on a physical robot. In simulation, Glass can override any field's value, and trigger-based overrides can inject specific readings at scripted moments during automated testing.

Use [SimSensorConfig](../config/sim-sensor-config.md) to build a `Sensor` declaratively. Direct construction is available when you need finer control over the underlying `SensorData` objects.

## Value resolution priority

Each field's value is resolved in this order every loop iteration:

1. **Real robot:** always returns the live hardware supplier value immediately.
2. **Active trigger:** if any registered `AddSimTrigger` condition returns `true`, the associated override value is written to Glass and returned.
3. **Glass value:** if no trigger fired, returns whatever Glass has set for the field.
4. **Supplier fallback:** if no Glass value is available, returns the hardware supplier value.

## Constructors

```cpp
Sensor(std::string sensorName, std::vector<SensorData> fields)
explicit Sensor(const yams::mechanisms::config::SimSensorConfig& cfg)
```

The `SimSensorConfig` overload is the preferred way to create sensors; see [SimSensorConfig](../config/sim-sensor-config.md).

| Parameter    | Description                                                             |
| ------------ | ----------------------------------------------------------------------- |
| `sensorName` | Display name shown in the Glass simulation window under `Sensor[name]`. |
| `fields`     | Fields to register. Each must have a unique name within this sensor.    |
| `cfg`        | A completed `SimSensorConfig` builder.                                  |

## Field access methods

| Method                                  | Returns                                                    | Description                                                                             |
| --------------------------------------- | ---------------------------------------------------------- | --------------------------------------------------------------------------------------- |
| `GetField(const std::string& name)`     | `SensorData&` (or `const SensorData&` on a const `Sensor`) | Returns the named field. Throws `std::out_of_range` if not found.                       |
| `GetAsDouble(const std::string& name)`  | `double`                                                   | Typed accessor. Throws a `std::logic_error` if not a double field.                      |
| `GetAsInt(const std::string& name)`     | `int`                                                      | Typed accessor. Throws a `std::logic_error` if not an int field.                        |
| `GetAsBoolean(const std::string& name)` | `bool`                                                     | Typed accessor. Throws a `std::logic_error` if not a bool field.                        |
| `GetAsLong(const std::string& name)`    | `int64_t`                                                  | Typed accessor. Throws a `std::logic_error` if not a long field.                        |
| `GetDevice()`                           | `HAL_SimDeviceHandle`                                      | The underlying WPILib SimDevice handle. Invalid (`HAL_kInvalidHandle`) on a real robot. |

## Simulation methods

| Method                                                                                    | Description                                                                                                                                                                                    |
| ----------------------------------------------------------------------------------------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `AddSimTrigger(const std::string& field, HAL_Value value, std::function<bool()> trigger)` | Registers a conditional override: whenever `trigger` returns `true`, the named field is set to `value` and Glass is updated. Use `SensorData::Convert()` to wrap a primitive in a `HAL_Value`. |

## SensorData

`SensorData` (`yams::motorcontrollers::simulation::SensorData`) represents a single typed field inside a `Sensor`. You interact with it directly when constructing sensors without `SimSensorConfig`, or when calling `AddSimTrigger` on an individual field.

### Constructors

```cpp
SensorData(std::string name, std::function<double()> supplier, double defaultVal)
SensorData(std::string name, std::function<int()> supplier, int defaultVal)
SensorData(std::string name, std::function<bool()> supplier, bool defaultVal)
SensorData(std::string name, std::function<int64_t()> supplier, int64_t defaultVal)
SensorData(std::string name, std::function<HAL_Value()> supplier, HAL_Value defaultValue,
           HALValueType type)  // raw HAL form
```

### Typed getters

| Method           | Returns        | Notes                                            |
| ---------------- | -------------- | ------------------------------------------------ |
| `GetAsDouble()`  | `double`       | Throws a `std::logic_error` on type mismatch.    |
| `GetAsInt()`     | `int`          |                                                  |
| `GetAsBoolean()` | `bool`         |                                                  |
| `GetAsLong()`    | `int64_t`      |                                                  |
| `GetValue()`     | `HAL_Value`    | Raw value applying full priority logic.          |
| `GetDefault()`   | `HAL_Value`    | Default value from construction.                 |
| `GetName()`      | `std::string`  | Field name.                                      |
| `GetType()`      | `HALValueType` | `kBoolean`, `kDouble`, `kEnum`, `kInt`, `kLong`. |

### Setters (simulation only)

| Method               | Description                            |
| -------------------- | -------------------------------------- |
| `Set(double val)`    | Directly writes to the Glass SimValue. |
| `Set(int val)`       |                                        |
| `Set(bool val)`      |                                        |
| `Set(int64_t val)`   |                                        |
| `Set(HAL_Value val)` | Raw HAL write.                         |

### Trigger injection

```cpp
void AddSimTrigger(HAL_Value value, std::function<bool()> trigger)
```

Registers a conditional override directly on this field. Triggers are evaluated in registration order; the first active one wins.

### Static converters

| Method                                                   | Description                                         |
| -------------------------------------------------------- | --------------------------------------------------- |
| `SensorData::Convert(double value)`                      | Wraps a primitive in a `HAL_Value`.                 |
| `SensorData::Convert(int value)`                         |                                                     |
| `SensorData::Convert(bool value)`                        |                                                     |
| `SensorData::Convert(int64_t value)`                     |                                                     |
| `SensorData::Convert(std::function<double()> supplier)`  | Wraps a supplier in a `std::function<HAL_Value()>`. |
| `SensorData::Convert(std::function<int()> supplier)`     |                                                     |
| `SensorData::Convert(std::function<bool()> supplier)`    |                                                     |
| `SensorData::Convert(std::function<int64_t()> supplier)` |                                                     |

## Example

```cpp
#include <yams/mechanisms/config/SimSensorConfig.hpp>
#include <yams/motorcontrollers/simulation/Sensor.hpp>
#include <yams/motorcontrollers/simulation/SensorData.hpp>
#include <wpi/hardware/discrete/DigitalInput.hpp>
#include <wpi/driverstation/RobotState.hpp>

using namespace yams::mechanisms::config;
using namespace yams::motorcontrollers::simulation;

wpi::DigitalInput beamBreak{3};

// --- Preferred: build via SimSensorConfig ---
SimSensorConfig cfg{"Intake"};
cfg.WithField("BeamBreak", [&beamBreak] { return !beamBreak.Get(); }, false)
   .WithSimulatedValue("BeamBreak", 2_s, 4_s, true);   // tripped at t=2s..4s in sim

Sensor& sensor = cfg.GetSensor();

// Read the value (real supplier on robot, Glass/trigger in sim)
bool isTriggered = sensor.GetAsBoolean("BeamBreak");

// --- Direct trigger injection after construction ---
sensor.AddSimTrigger(
    "BeamBreak",
    SensorData::Convert(true),
    [] { return wpi::RobotState::IsAutonomous(); }
);
```

{% hint style="info" %}
`Sensor[name]` appears as a device group in the Glass simulation window. Each registered field shows as a widget that can be read or manually overridden.
{% endhint %}

{% hint style="warning" %}
The typed `GetAs*` accessors throw a `std::logic_error` if the field was registered with a different type. Register a `bool` field with a `std::function<bool()>` supplier and read it with `GetAsBoolean`.
{% endhint %}

## Related pages

* [SimSensorConfig (C++)](../config/sim-sensor-config.md)
* [Sensor (Java)](../../java-reference/sensor/)
* [Tutorial: Using YAMS Sensors](../../guides/using-yams-sensors.md)
