// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

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
 *
 * <p>
 * {@code Sensor} models a named hardware sensor (such as an encoder, limit switch, or analog
 * input) whose state is consumed by simulation suppliers and mechanism code. Each sensor owns one
 * or more named {@link SensorData} fields that carry typed values (double, int, boolean, or long).
 * </p>
 *
 * <p>
 * On a <b>real robot</b> every field delegates to the real-hardware supplier provided at
 * construction time. In <b>simulation</b>, a {@link edu.wpi.first.hal.SimDevice} is registered
 * with WPILib so the WPILib Glass simulation GUI can read and override individual field values,
 * enabling hardware-in-the-loop-style testing without real hardware.
 * </p>
 *
 * <p>
 * Trigger overrides ({@link #addSimTrigger}) allow simulation suppliers to inject specific sensor
 * readings (e.g. a limit-switch trip) at a given moment during a simulated match, which is useful
 * for automating regression tests.
 * </p>
 *
 * <h2>Key fields and methods</h2>
 * <ul>
 *   <li>{@link #getField(String)} — retrieve a {@link SensorData} field by name.</li>
 *   <li>{@link #getAsDouble(String)}, {@link #getAsInt(String)}, {@link #getAsBoolean(String)},
 *       {@link #getAsLong(String)} — typed convenience accessors that call through to
 *       the underlying field.</li>
 *   <li>{@link #addSimTrigger(String, edu.wpi.first.hal.HALValue, java.util.function.BooleanSupplier)}
 *       — inject a simulated override value whenever a condition is true.</li>
 *   <li>{@link #getDevice()} — returns the underlying {@link edu.wpi.first.hal.SimDevice}
 *       (empty when running on a real robot).</li>
 * </ul>
 *
 * <h2>Example</h2>
 * <pre>{@code
 * // Wrap a real encoder position in a simulated sensor field
 * SensorData posField = new SensorData("position", encoder::getPosition, 0.0);
 * Sensor encoderSensor = new Sensor("ArmEncoder", List.of(posField));
 *
 * // Read the value (returns hardware value on robot, Glass value in sim)
 * double pos = encoderSensor.getAsDouble("position");
 *
 * // Inject a forced value during simulation when a condition is met
 * encoderSensor.addSimTrigger("position",
 *     SensorData.convert(90.0),
 *     () -> DriverStation.isAutonomous());
 * }</pre>
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
