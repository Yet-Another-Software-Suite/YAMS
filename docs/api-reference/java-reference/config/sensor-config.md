# SensorConfig

**Package:** `yams.core.mechanisms.config`\
**Javadoc:** [`SensorConfig`](https://javadocs.yassrobotics.com/yams/yams/core/mechanisms/config/SensorConfig.html)

Builder for a simulated sensor that bridges real hardware I/O and the YAMS simulation framework. Attach typed fields backed by live hardware suppliers, then inject override values during simulation by match-time window or an arbitrary trigger.

The finished config is resolved to a `Sensor` instance via `getSensor()`, which performs real/simulated value arbitration at runtime: the hardware value is used normally, and the injected value takes over whenever its trigger is active.

## Constructor

```java
SensorConfig(String name)
```

| Parameter | Description                                  |
| --------- | -------------------------------------------- |
| `name`    | Display name shown in the simulation window. |

## Builder methods: Field registration

Register each hardware value you want to expose. Four overloads accept `DoubleSupplier`, `IntSupplier`, `BooleanSupplier`, and `LongSupplier`.

```java
SensorConfig withField(String name, DoubleSupplier supplier, double defaultVal)
SensorConfig withField(String name, IntSupplier supplier, int defaultVal)
SensorConfig withField(String name, BooleanSupplier supplier, boolean defaultVal)
SensorConfig withField(String name, LongSupplier supplier, long defaultVal)
```

| Parameter    | Description                                                                |
| ------------ | -------------------------------------------------------------------------- |
| `name`       | Field identifier (must match the name used in `withSimulatedValue` calls). |
| `supplier`   | Live hardware value source (called every loop iteration on a real robot).  |
| `defaultVal` | Value used before any simulation override fires.                           |

## Builder methods: Simulated value injection

Override a field's value during a match-time window or when a `BooleanSupplier` trigger returns `true`. Overloads exist for `double`, `int`, `long`, and `boolean` values.

### By match-time window

```java
SensorConfig withSimulatedValue(String fieldName, Time start, Time end, double value)
SensorConfig withSimulatedValue(String fieldName, Time start, Time end, int value)
SensorConfig withSimulatedValue(String fieldName, Time start, Time end, long value)
SensorConfig withSimulatedValue(String fieldName, Time start, Time end, boolean value)
```

| Parameter   | Description                                                                      |
| ----------- | -------------------------------------------------------------------------------- |
| `fieldName` | Name of a previously registered field.                                           |
| `start`     | Match time (`Timer.getMatchTime()`) at which the simulated value becomes active. |
| `end`       | Match time at which the simulated value deactivates.                             |
| `value`     | Value to inject while active.                                                    |

### By arbitrary trigger

```java
SensorConfig withSimulatedValue(String fieldName, BooleanSupplier trigger, double value)
SensorConfig withSimulatedValue(String fieldName, BooleanSupplier trigger, int value)
SensorConfig withSimulatedValue(String fieldName, BooleanSupplier trigger, long value)
SensorConfig withSimulatedValue(String fieldName, BooleanSupplier trigger, boolean value)
```

| Parameter | Description                                                                  |
| --------- | ---------------------------------------------------------------------------- |
| `trigger` | Evaluated every loop; the simulated value is active while it returns `true`. |

## Accessors

| Method        | Returns            | Description                                                                          |
| ------------- | ------------------ | ------------------------------------------------------------------------------------ |
| `getSensor()` | `Sensor`           | Lazily constructs and returns the `Sensor` instance. Call once and cache the result. |
| `getName()`   | `String`           | The display name passed to the constructor.                                          |
| `getFields()` | `List<SensorData>` | The registered field list (used internally by `Sensor`).                             |

## Example

```java
import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Seconds;
import org.wpilib.hardware.discrete.DigitalInput;
import yams.core.mechanisms.config.SensorConfig;
import yams.core.motorcontrollers.simulation.Sensor;

// Hardware limit switch on DIO 0
DigitalInput limitSwitch = new DigitalInput(0);

SensorConfig forwardLimit = new SensorConfig("ForwardLimitSwitch")
    .withField("triggered", limitSwitch::get, false)
    // Simulate the switch being pressed from 10 s to 12 s into the match
    .withSimulatedValue("triggered", Seconds.of(10), Seconds.of(12), true);

// Beam break that reads "blocked" in simulation whenever the arm is near 40 degrees
DigitalInput beamBreak = new DigitalInput(1);
Sensor coralSensor = new SensorConfig("CoralDetectorBeamBreak")
    .withField("Beam", beamBreak::get, false)
    .withSimulatedValue("Beam", () -> arm.isNear(Degrees.of(40), Degrees.of(2)), true)
    .getSensor();
```

{% hint style="info" %}
Multiple `withSimulatedValue` calls on the same field are all registered. The first trigger that becomes active wins; subsequent triggers are evaluated in registration order.
{% endhint %}

{% hint style="warning" %}
`getSensor()` is lazy: the `Sensor` instance is not built until first called. Call it once after configuration is complete and store the result.
{% endhint %}

## Related pages

* [SensorConfig (C++)](../../c++-reference/config/sensorconfig.md)
* [SimSensorConfig (C++)](../../c++-reference/config/sim-sensor-config.md)
