package yams.motorcontrollers.simulation;

import edu.wpi.first.hal.HALValue;
import edu.wpi.first.hal.SimDevice;
import edu.wpi.first.hal.SimDevice.Direction;
import edu.wpi.first.hal.SimValue;
import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj.RobotBase;
import java.util.List;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;
import java.util.function.LongSupplier;
import java.util.function.Supplier;

/**
 * Sensor data class to encapsulate sensor data.
 */
public class SensorData
{

  /**
   * Sensor name.
   */
  private final String                                          m_name;
  /**
   * Sensor value supplier.
   */
  private final Supplier<HALValue>                              m_supplier;
  /**
   * {@link HALValueType} Type of data.
   */
  private final HALValueType                                    m_type;
  /**
   * {@link HALValue} default value.
   */
  private final HALValue                                        m_defaultValue;
  /**
   * Values, based off triggers.
   */
  private       Optional<List<Pair<BooleanSupplier, HALValue>>> m_triggerValues = Optional.empty();
  /**
   * Sim value from Glass.
   */
  private       Optional<SimValue>                              m_glassValue    = Optional.empty();
  /**
   * Previous sensor value when override takes place.
   */
  private       Optional<HALValue>                              m_prev          = Optional.empty();

  /**
   * Sensor data constructor.
   *
   * @param name         Name of sensor.
   * @param supplier     {@link Supplier<HALValue>} of sensor. Use {@link #convert} to convert primitive suppliers.
   * @param defaultValue Default value of sensor.
   * @param type         {@link HALValueType} of sensor.
   */
  public SensorData(String name, Supplier<HALValue> supplier, HALValue defaultValue, HALValueType type)
  {
    m_supplier = supplier;
    m_name = name;
    m_defaultValue = defaultValue;
    m_type = type;
  }

  /**
   * Sensor data constructor.
   *
   * @param name       Name of sensor.
   * @param supplier   {@link DoubleSupplier} supplier
   * @param defaultVal Double default value.
   */
  public SensorData(String name, DoubleSupplier supplier, double defaultVal)
  {
    this(name, convert(supplier), convert(defaultVal), HALValueType.kDouble);
  }

  /**
   * Sensor data constructor.
   *
   * @param name       Name of sensor.
   * @param supplier   {@link IntSupplier}
   * @param defaultVal Int default value.
   */
  public SensorData(String name, IntSupplier supplier, int defaultVal)
  {
    this(name, convert(supplier), convert(defaultVal), HALValueType.kInt);
  }
  /**
   * Sensor data constructor.
   *
   * @param name       Name of sensor.
   * @param supplier   {@link BooleanSupplier}
   * @param defaultVal Boolean default value.
   */
  public SensorData(String name, BooleanSupplier supplier, boolean defaultVal)
  {
    this(name, convert(supplier), convert(defaultVal), HALValueType.kBoolean);
  }
  /**
   * Sensor data constructor.
   *
   * @param name       Name of sensor.
   * @param supplier   {@link LongSupplier}
   * @param defaultVal Long default value.
   */
  public SensorData(String name, LongSupplier supplier, long defaultVal)
  {
    this(name, convert(supplier), convert(defaultVal), HALValueType.kLong);
  }

  /**
   * Convert a {@link DoubleSupplier} into a {@link Supplier<HALValue>}.
   *
   * @param supplier Double supplier.
   * @return HALValue supplier.
   */
  public static Supplier<HALValue> convert(DoubleSupplier supplier)
  {
    return () -> convert(supplier.getAsDouble());
  }

  /**
   * Convert an {@link IntSupplier} into a {@link Supplier<HALValue>}.
   *
   * @param supplier Int supplier.
   * @return HALValue supplier.
   */
  public static Supplier<HALValue> convert(IntSupplier supplier)
  {
    return () -> convert(supplier.getAsInt());
  }

  /**
   * Convert a {@link BooleanSupplier} into a {@link Supplier<HALValue>}.
   *
   * @param supplier Boolean supplier.
   * @return HALValue supplier.
   */
  public static Supplier<HALValue> convert(BooleanSupplier supplier)
  {
    return () -> convert(supplier.getAsBoolean());
  }

  /**
   * Convert a {@link LongSupplier} into a {@link Supplier<HALValue>}.
   *
   * @param supplier Long supplier.
   * @return {@link Supplier<HALValue>}.
   */
  public static Supplier<HALValue> convert(LongSupplier supplier)
  {
    return () -> convert(supplier.getAsLong());
  }

  /**
   * Convert a double into a {@link HALValue}.
   *
   * @param value Double value.
   * @return {@link HALValue}.
   */
  public static HALValue convert(double value)
  {
    return HALValue.makeDouble(value);
  }

  /**
   * Convert an int into a {@link HALValue}.
   *
   * @param value Int value.
   * @return {@link HALValue}.
   */
  public static HALValue convert(int value)
  {
    return HALValue.makeInt(value);
  }

  /**
   * Convert a boolean into a {@link HALValue}.
   *
   * @param value Boolean value.
   * @return {@link HALValue}.
   */
  public static HALValue convert(boolean value)
  {
    return HALValue.makeBoolean(value);
  }

  /**
   * Convert a long into a {@link HALValue}.
   *
   * @param value Long value.
   * @return {@link HALValue}.
   */
  public static HALValue convert(long value)
  {
    return HALValue.makeLong(value);
  }

  /**
   * Get the sensor value as a double.
   *
   * @return Sensor value.
   */
  public double getAsDouble()
  {
    if (m_type != HALValueType.kDouble)
    {
      throw new IllegalStateException(m_name + " HALValue is not a double!");
    }
    return getValue().getDouble();
  }

  /**
   * Get the sensor value as an int.
   *
   * @return Sensor value.
   */
  public int getAsInt()
  {
    if (m_type != HALValueType.kInt)
    {
      throw new IllegalStateException(m_name + " HALValue is not an int!");
    }
    return (int) getValue().getDouble();
  }

  /**
   * Get the sensor value as a long.
   *
   * @return Sensor value.
   */
  public long getAsLong()
  {
    if (m_type != HALValueType.kLong)
    {
      throw new IllegalStateException(m_name + " HALValue is not a long!");
    }
    return getValue().getLong();
  }

  /**
   * Get the sensor value as a boolean.
   *
   * @return Sensor value.
   */
  public boolean getAsBoolean()
  {
    if (m_type != HALValueType.kBoolean)
    {
      throw new IllegalStateException(m_name + " HALValue is not a boolean!");
    }
    return getValue().getBoolean();
  }

  /**
   * Set the sensor value.
   *
   * @param val Value to set.
   */
  public void set(HALValue val)
  {
    if (m_prev.isEmpty() && m_glassValue.isPresent())
    {
      m_prev = Optional.of(m_glassValue.get().getValue());
    }
    m_glassValue.ifPresent(simValue -> simValue.setValue(val));
  }

  /**
   * Get the sensor value, real sensor value if the robot is real.
   *
   * @return Sensor value.
   */
  public HALValue getValue()
  {
    // If the robot is real return the real value ASAP.
    if (RobotBase.isReal())
    {
      return m_supplier.get();
    }

    // Override sensor values with trigger values during a simulated match
    if (m_triggerValues.isPresent())
    {
      for (var entry : m_triggerValues.get())
      {
        if (entry.getFirst().getAsBoolean())
        {
          var value = entry.getSecond();
          set(value);
          return value;
        }
      }
    }

    // Reset and clear previous value upon change.
    if (m_prev.isPresent())
    {
      set(m_prev.get());
      m_prev = Optional.empty();
    }

    // If no glassValue is present, return the supplier value.
    return m_glassValue.isPresent() ? m_glassValue.get().getValue() : m_supplier.get();
  }

  /**
   * Add a value set based on a trigger.
   *
   * @param value   {@link HALValue} to set.
   * @param trigger {@link BooleanSupplier} when to use.
   */
  public void addSimTrigger(HALValue value, BooleanSupplier trigger)
  {
    var item = new Pair<>(trigger, value);
    if (m_triggerValues.isEmpty())
    {
      m_triggerValues = Optional.of(List.of(item));
    } else
    {
      m_triggerValues.get().add(item);
    }
  }

  /**
   * Get the sensor name.
   *
   * @return Sensor name.
   */
  public String getName()
  {
    return m_name;
  }

  /**
   * Get the sensor data type.
   *
   * @return {@link HALValueType}
   */
  public HALValueType getType()
  {
    return m_type;
  }

  /**
   * Get the default value.
   *
   * @return {@link HALValue} default value.
   */
  public HALValue getDefault()
  {
    return m_defaultValue;
  }

  /**
   * Get the {@link SimValue} for the sensor.
   *
   * @param device    {@link SimDevice} to create the {@link SimValue} for.
   * @param direction {@link Direction} of the {@link SimValue}.
   * @return {@link SimValue} for the sensor.
   */
  public SimValue createValue(SimDevice device, Direction direction)
  {
    var simVal = device.createValue(m_name, direction, m_defaultValue);
    if (direction == Direction.kBidir || direction == Direction.kInput)
    {
      m_glassValue = Optional.of(simVal);
    }
    return simVal;
  }

  /**
   * HALValue type enum.
   */
  public enum HALValueType
  {
    /**
     * Boolean type {@link HALValue#kBoolean}
     */
    kBoolean(HALValue.kBoolean),
    /**
     * Double type type {@link HALValue#kDouble}
     */
    kDouble(HALValue.kDouble),
    /**
     * Enum type type {@link HALValue#kEnum}
     */
    kEnum(HALValue.kEnum),
    /**
     * Int type type {@link HALValue#kInt}
     */
    kInt(HALValue.kInt),
    /**
     * Long type type {@link HALValue#kLong}
     */
    kLong(HALValue.kLong);

    /**
     * {@link HALValue} type
     */
    private final int m_type;

    /**
     * HALValue type enum.
     *
     * @param type HALValue type.
     */
    HALValueType(int type)
    {
      m_type = type;
    }

    /**
     * Get the HALValue type.
     *
     * @return {@link HALValue}
     */
    public int getType()
    {
      return m_type;
    }
  }
}
