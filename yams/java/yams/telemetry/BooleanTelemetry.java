// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package yams.telemetry;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.BooleanTopic;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.PubSub;
import java.util.Optional;
import yams.telemetry.SmartMotorControllerTelemetry.BooleanTelemetryField;

/**
 * Boolean Telemetry for SmartMotorControllers.
 */
public class BooleanTelemetry
{

  /**
   * Field representing.
   */
  private final BooleanTelemetryField       field;
  /**
   * Network table key.
   */
  private final String                      key;
  /**
   * Tunable?
   */
  private final boolean                     tunable;
  /**
   * Enabled?
   */
  protected     boolean                     enabled     = false;
  /**
   * Default value.
   */
  private       boolean                     defaultValue;
  /**
   * Cached value.
   */
  private       boolean                     cachedValue;
  /**
   * Publisher.
   */
  private       BooleanPublisher            publisher   = null;
  /**
   * Subscriber.
   */
  private       Optional<BooleanSubscriber> subscriber  = Optional.empty();
  /**
   * Sub publisher.
   */
  private       BooleanPublisher            pubSub      = null;
  /**
   * pub or sub topic.
   */
  private       BooleanTopic                topic;
  /**
   * Tuning table
   */
  private       Optional<NetworkTable>      tuningTable = Optional.empty();
  /**
   * Data table.
   */
  private       Optional<NetworkTable>      dataTable   = Optional.empty();

  /**
   * Setup boolean telemetry for a field.
   *
   * @param keyString  Networks table key.
   * @param defaultVal Default value.
   * @param field      Field representing.
   * @param tunable    Tunable?
   */
  public BooleanTelemetry(String keyString, boolean defaultVal, BooleanTelemetryField field, boolean tunable)
  {
    key = keyString;
    cachedValue = defaultValue = defaultVal;
    this.field = field;
    this.tunable = tunable;

  }

  /**
   * Setup network tables.
   *
   * @param dataTable   Data tables.
   * @param tuningTable Tuning table.
   */
  public void setupNetworkTables(NetworkTable dataTable, NetworkTable tuningTable)
  {
    this.dataTable = Optional.ofNullable(dataTable);
    this.tuningTable = Optional.ofNullable(tuningTable);
    if (tuningTable != null && tunable)
    {
      topic = tuningTable.getBooleanTopic(key);
      pubSub = topic.publish();
      pubSub.setDefault(defaultValue);
      subscriber = Optional.of(topic.subscribe(defaultValue));
    } else
    {
      topic = dataTable.getBooleanTopic(key);
      publisher = topic.publish();
      publisher.setDefault(defaultValue);
    }
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
  public boolean set(boolean value)
  {
    if (subscriber.isPresent())
    {
      boolean tuningValue = subscriber.get().get(defaultValue);
      if (tuningValue != value)
      {
        return false;
      }
    }
    if (publisher != null)
    {
      publisher.accept(value);
    }
    return true;
  }

  /**
   * Get the value.
   *
   * @return Value.
   */
  public boolean get()
  {
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
  public BooleanTelemetryField getField()
  {
    return field;
  }

  /**
   * Set the default value.
   *
   * @param value Default value.
   */
  public void setDefaultValue(boolean value)
  {
    defaultValue = value;
    cachedValue = value;
  }

  /**
   * Close the telemetry field.
   */
  public void close()
  {
    subscriber.ifPresent(PubSub::close);
    if (pubSub != null)
    {pubSub.close();}
    if (publisher != null)
    {publisher.close();}
    dataTable.ifPresent(table -> table.getEntry(key).unpublish());
    tuningTable.ifPresent(table -> table.getEntry(key).unpublish());
  }
}
