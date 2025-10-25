package yams.mechanisms.config;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DriverStation;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;
import java.util.function.LongSupplier;
import yams.motorcontrollers.simulation.Sensor;
import yams.motorcontrollers.simulation.SensorData;

/**
 * Sensor configuration for simulated and real sensors.
 */
public class SensorConfig
{

  /**
   * Sensor name to display in the simulation window.
   */
  private final String           name;
  /**
   * List of {@link SensorData} to display in the simulation window.
   */
  private final List<SensorData> data = new ArrayList<>();
  /**
   * Sensor
   */
  private       Optional<Sensor> sensor = Optional.empty();

  /**
   * Sensor configuration.
   *
   * @param name Name of sensor to display in the simulation window.
   */
  public SensorConfig(String name)
  {
    this.name = name;
  }

  /**
   * Add a field to the sensor.
   *
   * @param name       Name of the field to add.
   * @param supplier   Supplier of the real field value.
   * @param defaultVal Default value of the field.
   * @return {@link SensorConfig}
   */
  public SensorConfig withField(String name, DoubleSupplier supplier, double defaultVal)
  {
    data.add(new SensorData(name, supplier, defaultVal));
    return this;
  }

  /**
   * Add a field to the sensor.
   *
   * @param name       Name of the field to add.
   * @param supplier   Supplier of the real field value.
   * @param defaultVal Default value of the field.
   * @return {@link SensorConfig}
   */
  public SensorConfig withField(String name, IntSupplier supplier, int defaultVal)
  {
    data.add(new SensorData(name, supplier, defaultVal));
    return this;
  }

  /**
   * Add a field to the sensor.
   *
   * @param name       Name of the field to add.
   * @param supplier   Supplier of the real field value.
   * @param defaultVal Default value of the field.
   * @return {@link SensorConfig}
   */
  public SensorConfig withField(String name, BooleanSupplier supplier, boolean defaultVal)
  {
    data.add(new SensorData(name, supplier, defaultVal));
    return this;
  }

  /**
   * Add a field to the sensor.
   *
   * @param name       Name of the field to add.
   * @param supplier   Supplier of the real field value.
   * @param defaultVal Default value of the field.
   * @return {@link SensorConfig}
   */
  public SensorConfig withField(String name, LongSupplier supplier, long defaultVal)
  {
    data.add(new SensorData(name, supplier, defaultVal));
    return this;
  }

  /**
   * Add a simulated value to the sensor at a given match time.
   *
   * @param fieldName Name of the field to change.
   * @param start     {@link Time} at which to start the data simulation.
   * @param end       {@link Time} at which to end the data simulation.
   * @param value     Value to simulate.
   * @return {@link SensorConfig}
   */
  public SensorConfig withSimulatedValue(String fieldName, Time start, Time end, double value)
  {
    for (var field : data)
    {
      if (field.getName().equals(fieldName))
      {
        field.addSimTrigger(SensorData.convert(value), () -> DriverStation.getMatchTime() >= start.in(Seconds) &&
                                                             DriverStation.getMatchTime() <= end.in(Seconds));
      }
    }
    return this;
  }

  /**
   * Add a simulated value to the sensor at a given match time.
   *
   * @param fieldName Name of the field to change.
   * @param start     {@link Time} at which to start the data simulation.
   * @param end       {@link Time} at which to end the data simulation.
   * @param value     Value to simulate.
   * @return {@link SensorConfig}
   */
  public SensorConfig withSimulatedValue(String fieldName, Time start, Time end, int value)
  {
    for (var field : data)
    {
      if (field.getName().equals(fieldName))
      {
        field.addSimTrigger(SensorData.convert(value), () -> DriverStation.getMatchTime() >= start.in(Seconds) &&
                                                             DriverStation.getMatchTime() <= end.in(Seconds));
      }
    }
    return this;
  }

  /**
   * Add a simulated value to the sensor at a given match time.
   *
   * @param fieldName Name of the field to change.
   * @param start     {@link Time} at which to start the data simulation.
   * @param end       {@link Time} at which to end the data simulation.
   * @param value     Value to simulate.
   * @return {@link SensorConfig}
   */
  public SensorConfig withSimulatedValue(String fieldName, Time start, Time end, long value)
  {
    for (var field : data)
    {
      if (field.getName().equals(fieldName))
      {
        field.addSimTrigger(SensorData.convert(value), () -> DriverStation.getMatchTime() >= start.in(Seconds) &&
                                                             DriverStation.getMatchTime() <= end.in(Seconds));
      }
    }
    return this;
  }

  /**
   * Add a simulated value to the sensor at a given match time.
   *
   * @param fieldName Name of the field to change.
   * @param start     {@link Time} at which to start the data simulation.
   * @param end       {@link Time} at which to end the data simulation.
   * @param value     Value to simulate.
   * @return {@link SensorConfig}
   */
  public SensorConfig withSimulatedValue(String fieldName, Time start, Time end, boolean value)
  {
    for (var field : data)
    {
      if (field.getName().equals(fieldName))
      {
        field.addSimTrigger(SensorData.convert(value), () -> DriverStation.getMatchTime() >= start.in(Seconds) &&
                                                             DriverStation.getMatchTime() <= end.in(Seconds));
      }
    }
    return this;
  }

  /**
   * Add a simulated value to the sensor at a given trigger.
   *
   * @param fieldName Name of the field to change.
   * @param trigger   {@link BooleanSupplier} for when data should be simulated.
   * @param value     Value to simulate.
   * @return {@link SensorConfig}
   */
  public SensorConfig withSimulatedValue(String fieldName, BooleanSupplier trigger, double value)
  {
    for (var field : data)
    {
      if (field.getName().equals(fieldName))
      {
        field.addSimTrigger(SensorData.convert(value), trigger);
      }
    }
    return this;
  }

  /**
   * Add a simulated value to the sensor at a given trigger.
   *
   * @param fieldName Name of the field to change.
   * @param trigger   {@link BooleanSupplier} for when data should be simulated.
   * @param value     Value to simulate.
   * @return {@link SensorConfig}
   */
  public SensorConfig withSimulatedValue(String fieldName, BooleanSupplier trigger, int value)
  {
    for (var field : data)
    {
      if (field.getName().equals(fieldName))
      {
        field.addSimTrigger(SensorData.convert(value), trigger);
      }
    }
    return this;
  }

  /**
   * Add a simulated value to the sensor at a given trigger.
   *
   * @param fieldName Name of the field to change.
   * @param trigger   {@link BooleanSupplier} for when data should be simulated.
   * @param value     Value to simulate.
   * @return {@link SensorConfig}
   */
  public SensorConfig withSimulatedValue(String fieldName, BooleanSupplier trigger, long value)
  {
    for (var field : data)
    {
      if (field.getName().equals(fieldName))
      {
        field.addSimTrigger(SensorData.convert(value), trigger);
      }
    }
    return this;
  }

  /**
   * Add a simulated value to the sensor at a given trigger.
   *
   * @param fieldName Name of the field to change.
   * @param trigger   {@link BooleanSupplier} for when data should be simulated.
   * @param value     Value to simulate.
   * @return {@link SensorConfig}
   */
  public SensorConfig withSimulatedValue(String fieldName, BooleanSupplier trigger, boolean value)
  {
    for (var field : data)
    {
      if (field.getName().equals(fieldName))
      {
        field.addSimTrigger(SensorData.convert(value), trigger);
      }
    }
    return this;
  }

  /**
   * Get the {@link Sensor} for this sensor.
   *
   * @return {@link Sensor} for fetching real and simulated values.
   */
  public Sensor getSensor()
  {
    if (sensor.isEmpty())
    {
      sensor = Optional.of(new Sensor(name, data));
    }
    return sensor.get();
  }

  /**
   * Get the name of the sensor.
   *
   * @return Name of the sensor.
   */
  public String getName()
  {
    return name;
  }

  /**
   * Get the list of Fields ({@link SensorData}) for this sensor.
   *
   * @return list of {@link SensorData} for this sensor
   */
  public List<SensorData> getFields()
  {
    return data;
  }

}
