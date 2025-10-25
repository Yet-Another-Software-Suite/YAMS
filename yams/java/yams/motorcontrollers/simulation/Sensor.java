package yams.motorcontrollers.simulation;

import edu.wpi.first.hal.HALValue;
import edu.wpi.first.hal.SimDevice;
import edu.wpi.first.hal.SimDevice.Direction;
import edu.wpi.first.wpilibj.RobotBase;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.stream.Collectors;
import yams.mechanisms.config.SensorConfig;

/**
 * Sensor class using {@link edu.wpi.first.hal.SimDevice}; All fields will use the given supplier on real robots. Fake
 * data is only given when connected to simulation.
 */
public class Sensor
{

  /**
   * Simulated device.
   */
  private final Optional<SimDevice>     m_simDevice;
  /**
   * Sensor name.
   */
  private final String                  m_sensorName;
  /**
   * Simulated data.
   */
  private final Map<String, SensorData> m_simData;

  /**
   * Sensor constructor, for a sensor that will report the real data when connected to the robot or Simulation GUI data
   * when connected to Sim.
   *
   * @param sensorName   Name of the sensor.
   * @param sensorFields List of sensor fields. See {@link SensorData}.
   */
  public Sensor(String sensorName, List<SensorData> sensorFields)
  {
    m_sensorName = sensorName;
    m_simData = sensorFields.stream().collect(Collectors.toMap(SensorData::getName, entry -> entry));
    if (RobotBase.isSimulation())
    {
      m_simDevice = Optional.of(SimDevice.create("Sensor[" + sensorName + "]"));
      for (var field : sensorFields)
      {
        field.createValue(m_simDevice.get(), Direction.kBidir);
      }
    } else
    {
      m_simDevice = Optional.empty();
    }
  }

  /**
   * Sensor simulation constructor.
   *
   * @param cfg {@link SensorConfig} class
   */
  public Sensor(SensorConfig cfg)
  {
    this(cfg.getName(), cfg.getFields());
  }

  /**
   * Get a field from the simulated or real sensor.
   *
   * @param name Name of the field
   * @return {@link SensorData} of the field.
   */
  public SensorData getField(String name)
  {
    if (!m_simData.containsKey(name))
    {
      throw new IllegalArgumentException("Sensor[" + m_sensorName + "." + name + "] does not exist!");
    }
    return m_simData.get(name);
  }

  /**
   * Get a field from the simulated or real sensor.
   *
   * @param name Name of the field
   * @return Value of the field as a double.
   */
  public double getAsDouble(String name)
  {
    return getField(name).getAsDouble();
  }

  /**
   * Get a field from the simulated or real sensor.
   *
   * @param name Name of the field
   * @return Value of the field as an int.
   */
  public int getAsInt(String name)
  {
    return getField(name).getAsInt();
  }

  /**
   * Get a field from the simulated or real sensor.
   *
   * @param name Name of the field
   * @return Value of the field as a boolean.
   */
  public boolean getAsBoolean(String name)
  {
    return getField(name).getAsBoolean();
  }

  /**
   * Get a field from the simulated or real sensor.
   *
   * @param name Name of the field
   * @return Value of the field as a long.
   */
  public long getAsLong(String name)
  {
    return getField(name).getAsLong();
  }

  /**
   * Get the simulated device.
   *
   * @return Simulated device.
   */
  public Optional<SimDevice> getDevice()
  {
    return m_simDevice;
  }

  /**
   * Set a simulated value based on a trigger.
   *
   * @param field   Field name to set.
   * @param value   {@link HALValue} to set.
   * @param trigger {@link BooleanSupplier} when to use.
   */
  public void addSimTrigger(String field, HALValue value, BooleanSupplier trigger)
  {
    getField(field).addSimTrigger(value, trigger);
  }

}
