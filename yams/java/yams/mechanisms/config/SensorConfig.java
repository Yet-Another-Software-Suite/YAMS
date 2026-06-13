// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

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
 *
 * <p>{@code SensorConfig} describes a named sensor that can expose one or more typed fields
 * (double, int, boolean, long) to both the real robot and the simulation environment. It is
 * used to wire hardware sensors — such as limit switches, encoders, or custom I/O devices —
 * into YAMS's simulation framework so that the same code path runs identically in simulation
 * and on a real robot.
 *
 * <p>Each field is registered with a live supplier (the real hardware value) and a default
 * value used when simulation overrides are not active. Simulated values can be injected by
 * match-time window or by an arbitrary {@link java.util.function.BooleanSupplier} trigger,
 * allowing a team to script realistic sensor behaviour during automated testing.
 *
 * <p>The finished configuration is converted to a {@link yams.motorcontrollers.simulation.Sensor}
 * via {@link #getSensor()}, which handles the real/simulated value arbitration at runtime.
 *
 * <h2>Example</h2>
 * <pre>{@code
 * import static edu.wpi.first.units.Units.Seconds;
 * import yams.mechanisms.config.SensorConfig;
 *
 * // Hardware limit switch wired to a DIO channel
 * DigitalInput limitSwitch = new DigitalInput(0);
 *
 * SensorConfig forwardLimit = new SensorConfig("ForwardLimitSwitch")
 *     .withField("triggered", limitSwitch::get, false)
 *     // Simulate the switch being pressed between 10 s and 12 s of match time
 *     .withSimulatedValue("triggered", Seconds.of(10), Seconds.of(12), true);
 *
 * // Custom encoder exposing position and velocity
 * SensorConfig encoderConfig = new SensorConfig("ArmEncoder")
 *     .withField("positionRotations", myEncoder::getPosition, 0.0)
 *     .withField("velocityRPM",       myEncoder::getVelocity, 0.0)
 *     // Override position to 2.5 rotations whenever a BooleanSupplier fires
 *     .withSimulatedValue("positionRotations", () -> Robot.isInTest(), 2.5);
 * }</pre>
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
