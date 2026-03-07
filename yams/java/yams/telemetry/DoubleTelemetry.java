// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package yams.telemetry;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.PubSub;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Timer;
import java.util.Optional;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.telemetry.SmartMotorControllerTelemetry.DoubleTelemetryField;

/**
 * Double Telemetry for SmartMotorControllers.
 */
public class DoubleTelemetry
{

  /**
   * Field representing.
   */
  private final DoubleTelemetryField       field;
  /**
   * Network table key.
   */
  private final String                     key;
  /**
   * Tunable?
   */
  private final boolean                    tunable;
  /**
   * Enabled?
   */
  protected     boolean                    enabled      = false;
  /**
   * Unit to display.
   */
  private       String                     unit;
  /**
   * Default value.
   */
  private       double                     defaultValue;
  /**
   * Cached value.
   */
  private       double                     cachedValue;
  /**
   * Publisher.
   */
  private       Optional<DoublePublisher>  publisher    = Optional.empty();
  /**
   * Subscriber.
   */
  private       Optional<DoubleSubscriber> subscriber   = Optional.empty();
  /**
   * Sub publisher.
   */
  private       DoublePublisher            subPublisher = null;
  /**
   * Tuning table
   */
  private       Optional<NetworkTable>     tuningTable  = Optional.empty();
  /**
   * Data table.
   */
  private       Optional<NetworkTable>     dataTable    = Optional.empty();
  /**
   * NT4 Topic of this entry.
   */
  private       DoubleTopic                topic;
  /**
   * {@link DoubleLogEntry} representing this entry.
   */
  private       Optional<DoubleLogEntry>   dataLogEntry = Optional.empty();


  /**
   * Setup double telemetry for a field.
   *
   * @param keyString  Key to use.
   * @param defaultVal Default value.
   * @param field      Field representing.
   * @param tunable    Tunable.
   * @param unit       Unit to display.
   */
  public DoubleTelemetry(String keyString, double defaultVal, DoubleTelemetryField field, boolean tunable, String unit)
  {
    key = keyString;
    cachedValue = defaultValue = defaultVal;
    this.field = field;
    this.tunable = tunable;
    this.unit = unit;
  }

  /**
   * Set default values.
   *
   * @param defaultValue Default for the entry.
   */
  public void setDefaultValue(double defaultValue)
  {
    cachedValue = this.defaultValue = defaultValue;
  }

  /**
   * Setup network tables.
   *
   * @param dataTable   Data tables.
   * @param tuningTable Tuning table.
   */
  public void setupNetworkTables(NetworkTable dataTable, NetworkTable tuningTable)
  {
    this.tuningTable = Optional.ofNullable(tuningTable);
    this.dataTable = Optional.ofNullable(dataTable);
    if (!enabled)
    {return;}
    if (tuningTable != null && tunable)
    {
      topic = tuningTable.getDoubleTopic(key);
      subPublisher = !unit.equals("none") ?
                     topic.publishEx("double", "{\"units\": \"" + unit + "\"}") :
                     topic.publish();
      subscriber = Optional.of(topic.subscribe(defaultValue));
      subPublisher.setDefault(defaultValue);
    } else
    {
      assert dataTable != null;
      topic = dataTable.getDoubleTopic(key);
      publisher = Optional.of(!unit.equals("none") ?
                              topic.publishEx("double", "{\"units\": \"" + unit + "\"}") :
                              topic.publish());
      publisher.get().setDefault(defaultValue);
    }
  }

  /**
   * Setup the {@link edu.wpi.first.util.datalog.DataLog} with this entry.
   *
   * @param prefix The prefix to this entry in {@link edu.wpi.first.util.datalog.DataLog}
   */
  public void setupDataLog(String prefix)
  {
    if (!tunable)
    {
      if (!prefix.endsWith("/"))
      {prefix += "/";}
      prefix += unit + "/";
      dataLogEntry = Optional.of(new DoubleLogEntry(DataLogManager.getLog(),
                                                    prefix + key,
                                                    (long) Timer.getFPGATimestamp()));
    }
  }

  /**
   * Set the unit.
   *
   * @param cfg {@link SmartMotorControllerConfig} used to determine the unit. If the MechanismCircumference is set it
   *            will be in meters, else it will be in degrees.
   * @return {@link DoubleTelemetry} for chaining.
   */
  public DoubleTelemetry transformUnit(SmartMotorControllerConfig cfg)
  {
    switch (unit)
    {
      case "tunable_position":
        unit = cfg.getLinearClosedLoopControllerUse() ? "meter" : "degrees";
        break;
      case "position":
        unit = cfg.getLinearClosedLoopControllerUse() ? "meter" : "rotations";
        break;
      case "tunable_velocity":
        unit = cfg.getLinearClosedLoopControllerUse() ? "meter_per_second"
                                                      : "rotations_per_minute";
        break;
      case "velocity":
        unit = cfg.getLinearClosedLoopControllerUse() ? "meter_per_second"
                                                      : "rotation_per_second";
        break;
      case "tunable_acceleration":
        unit = cfg.getLinearClosedLoopControllerUse() ? "meter_per_second_per_second"
                                                      : "rotations_per_minute_per_second";
        break;
      case "acceleration":
        unit = cfg.getLinearClosedLoopControllerUse() ? "meter_per_second_per_second"
                                                      : "rotation_per_second_per_second";
        break;
    }
    return this;
  }


  /**
   * Setup network tables.
   *
   * @param dataTable Data tables.
   */
  public void setupNetworkTable(NetworkTable dataTable)
  {
    setupNetworkTables(dataTable, null);
  }

  /**
   * Set the value of the publisher, checking to see if the value is the same as the subscriber.
   *
   * @param value Value to set.
   * @return True if value was able to be set.
   */
  public boolean set(double value)
  {
    if (!enabled)
    {return false;}
    if (dataLogEntry.isPresent())
    {
      dataLogEntry.get().append(value, (long) Timer.getFPGATimestamp());
    }
    if (subscriber.isPresent())
    {
      double tuningValue = subscriber.get().get(defaultValue);
      if (tuningValue != value)
      {
        return false;
      }
    }
    if (publisher.isPresent())
    {
      publisher.get().accept(value);
    }
    return true;
  }

  /**
   * Get the value.
   *
   * @return value of telemetry.
   */
  public double get()
  {
    if (!enabled)
    {return defaultValue;}
    if (subscriber.isPresent())
    {
      return subscriber.get().get(defaultValue);
    }
    throw new RuntimeException("Tuning table not configured for " + key + "!");
  }

  /**
   * Check to see if the value has changed.
   *
   * @return True if the value has changed.
   */
  public boolean tunable()
  {
    if (subscriber.isPresent() && tunable && enabled)
    {
      if (subscriber.get().get(defaultValue) != cachedValue)
      {
        cachedValue = subscriber.get().get(defaultValue);
        return true;
      }
      return false;
    }
    return false;
  }

  /**
   * Enable the telemetry.
   */
  public void enable()
  {
    enabled = true;
//    if ((publisher.isEmpty() || subscriber.isEmpty()) && (tuningTable.isPresent() || dataTable.isPresent()))
//    {setupNetworkTables(dataTable.get(), tuningTable.get());}
  }

  /**
   * Disable the telemetry.
   */
  public void disable()
  {
    enabled = false;
  }

  /**
   * Display the telemetry.
   *
   * @param state Enable or disable.
   */
  public void display(boolean state)
  {
    enabled = state;
  }

  /**
   * Get the field.
   *
   * @return field.
   */
  public DoubleTelemetryField getField()
  {
    return field;
  }

  /**
   * Close the telemetry field.
   */
  public void close()
  {
    subscriber.ifPresent(PubSub::close);
    if (subPublisher != null)
    {subPublisher.close();}
    publisher.ifPresent(PubSub::close);
    dataTable.ifPresent(table -> table.getEntry(key).unpublish());
    tuningTable.ifPresent(table -> table.getEntry(key).unpublish());
  }
}
