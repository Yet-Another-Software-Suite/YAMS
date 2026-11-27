# Sensor and simulation

**Package:** `yams.core.motorcontrollers.simulation`\
**Javadoc:** [`Sensor`](https://javadocs.yassrobotics.com/yams/yams/core/motorcontrollers/simulation/Sensor.html)

A `Sensor` models a named hardware sensor whose fields are readable from robot code and, in simulation, visible in the WPILib **Glass** GUI. Every field delegates to a real hardware supplier on a physical robot. In simulation, Glass can override any field's value, and trigger-based overrides can inject specific readings at scripted moments during automated testing.

Use [SensorConfig](../config/sensor-config.md) to build a `Sensor` declaratively. Direct construction is available when you need finer control over the underlying `SensorData` objects.

## Value resolution priority

Each field's value is resolved in this order every loop iteration:

1. **Real robot**: always returns the live hardware supplier value immediately.
2. **Active trigger**: if any registered `addSimTrigger` condition returns `true`, the associated override value is written to Glass and returned.
3. **Glass value**: if no trigger fired, returns whatever Glass has set for the field (including the configured default).
4. **Supplier fallback**: if no Glass value is available, returns the hardware supplier value.

## Constructors

```java
Sensor(String sensorName, List<SensorData> sensorFields)
Sensor(SensorConfig cfg)
```

Both constructors throw `IllegalStateException` if two fields share the same name.

The `SensorConfig` overload is the preferred way to create sensors; see [SensorConfig](../config/sensor-config.md).

| Parameter      | Description                                                                |
| -------------- | -------------------------------------------------------------------------- |
| `sensorName`   | Display name shown in the Glass simulation window under `Sensor[name]`.    |
| `sensorFields` | Fields to register. Each field must have a unique name within this sensor. |
| `cfg`          | A completed `SensorConfig` builder.                                        |

## Field access methods

| Method                      | Returns               | Description                                                                              |
| --------------------------- | --------------------- | ---------------------------------------------------------------------------------------- |
| `getField(String name)`     | `SensorData`          | Returns the named field. Throws `IllegalArgumentException` if the name does not exist.   |
| `getAsDouble(String name)`  | `double`              | Typed convenience accessor. Throws `IllegalStateException` if the field is not a double. |
| `getAsInt(String name)`     | `int`                 | Typed convenience accessor. Throws an `IllegalStateException` if not an int field.       |
| `getAsBoolean(String name)` | `boolean`             | Typed convenience accessor. Throws an `IllegalStateException` if not a boolean field.    |
| `getAsLong(String name)`    | `long`                | Typed convenience accessor. Throws an `IllegalStateException` if not a long field.       |
| `getDevice()`               | `Optional<SimDevice>` | The underlying WPILib `SimDevice`. Empty on a real robot.                                |

## Simulation methods

| Method                                                                 | Description                                                                                                                                                                                       |
| ---------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `addSimTrigger(String field, HALValue value, BooleanSupplier trigger)` | Registers a conditional override: whenever `trigger` returns `true`, the named field is set to `value` and Glass is updated. Use `SensorData.convert()` to produce a `HALValue` from a primitive. |

## SensorData

`SensorData` (`yams.core.motorcontrollers.simulation.SensorData`) represents a single typed field inside a `Sensor`. You interact with `SensorData` directly when constructing sensors without `SensorConfig`, or when calling `addSimTrigger` on an individual field.

### Constructors

```java
SensorData(String name, DoubleSupplier supplier, double defaultVal)
SensorData(String name, IntSupplier supplier, int defaultVal)
SensorData(String name, BooleanSupplier supplier, boolean defaultVal)
SensorData(String name, LongSupplier supplier, long defaultVal)
SensorData(String name, Supplier<HALValue> supplier, HALValue defaultValue, HALValueType type)
```

### Typed getters

| Method           | Returns        | Notes                                                                       |
| ---------------- | -------------- | --------------------------------------------------------------------------- |
| `getAsDouble()`  | `double`       | Throws `IllegalStateException` if type mismatch.                            |
| `getAsInt()`     | `int`          |                                                                             |
| `getAsBoolean()` | `boolean`      |                                                                             |
| `getAsLong()`    | `long`         |                                                                             |
| `getValue()`     | `HALValue`     | Raw value applying full priority logic.                                     |
| `getDefault()`   | `HALValue`     | The default value set at construction.                                      |
| `getName()`      | `String`       | Field name.                                                                 |
| `getType()`      | `HALValueType` | `SensorData.HALValueType`: `kBoolean`, `kDouble`, `kEnum`, `kInt`, `kLong`. |

### Setters (simulation only)

| Method              | Description                            |
| ------------------- | -------------------------------------- |
| `set(double val)`   | Directly writes to the Glass SimValue. |
| `set(int val)`      |                                        |
| `set(boolean val)`  |                                        |
| `set(long val)`     |                                        |
| `set(HALValue val)` | Raw HAL write.                         |

### Trigger injection

```java
void addSimTrigger(HALValue value, BooleanSupplier trigger)
```

Registers a conditional override on this field directly. Triggers are evaluated in registration order; the first active one wins.

### Static converters

| Method                                         | Description                                 |
| ---------------------------------------------- | ------------------------------------------- |
| `SensorData.convert(double value)`             | Wraps a primitive in a `HALValue`.          |
| `SensorData.convert(int value)`                |                                             |
| `SensorData.convert(boolean value)`            |                                             |
| `SensorData.convert(long value)`               |                                             |
| `SensorData.convert(DoubleSupplier supplier)`  | Wraps a supplier in a `Supplier<HALValue>`. |
| `SensorData.convert(IntSupplier supplier)`     |                                             |
| `SensorData.convert(BooleanSupplier supplier)` |                                             |
| `SensorData.convert(LongSupplier supplier)`    |                                             |

## Example

```java
import yams.core.mechanisms.config.SensorConfig;
import yams.core.motorcontrollers.simulation.Sensor;
import yams.core.motorcontrollers.simulation.SensorData;
import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Seconds;
import org.wpilib.hardware.discrete.DigitalInput;

// --- Preferred: build via SensorConfig ---
DigitalInput beamBreak = new DigitalInput(3);

SensorConfig cfg = new SensorConfig("Intake")
    .withField("BeamBreak", beamBreak::get, false)
    .withSimulatedValue("BeamBreak", Seconds.of(2), Seconds.of(4), true);

Sensor sensor = cfg.getSensor();

// Read the value (real supplier on robot, Glass/trigger in sim)
boolean isTriggered = sensor.getAsBoolean("BeamBreak");

// --- Direct trigger injection after construction ---
sensor.addSimTrigger(
    "BeamBreak",
    SensorData.convert(true),
    () -> arm.isNear(Degrees.of(40), Degrees.of(2))
);
```

{% hint style="info" %}
`Sensor[name]` appears as a device group in the Glass simulation window. Each registered field shows as a widget that can be read or manually overridden.
{% endhint %}

{% hint style="warning" %}
The typed `getAs*` methods throw `IllegalStateException` if the field was registered with a different type. Register a boolean field with a `BooleanSupplier`, and read it with `getAsBoolean`.
{% endhint %}

## Related pages

* [SensorConfig](../config/sensor-config.md)
* [Sensor (C++)](../../c++-reference/sensor/)
* [Tutorial: Using YAMS Sensors](../../guides/using-yams-sensors.md)
