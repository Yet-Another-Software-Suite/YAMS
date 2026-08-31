// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package yams.telemetry;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.PubSub;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructArraySubscriber;
import edu.wpi.first.networktables.StructArrayTopic;
import edu.wpi.first.util.datalog.StructArrayLogEntry;
import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Timer;
import java.util.Arrays;
import java.util.Optional;

/**
 * Struct array Telemetry for arbitrary {@link Struct} serializable array types.
 *
 * <p>A lightweight wrapper that publishes a struct-encoded array value (e.g. {@code
 * SwerveModuleState[]}, {@code SwerveModulePosition[]}) to NetworkTables and/or a WPILib DataLog.
 * It mirrors {@link StructTelemetry}, but publishes an array of struct-serializable values instead
 * of a single value.
 *
 * <h2>Example</h2>
 * <pre>{@code
 * // Create and publish a struct array entry for the current module states.
 * StructArrayTelemetry<SwerveModuleState, MyField> states = new StructArrayTelemetry<>(
 *     "states/current",       // NetworkTables key
 *     new SwerveModuleState[0], // default value
 *     MyField.CurrentStates,  // field representing
 *     SwerveModuleState.struct, // struct serializer
 *     false);                 // not tunable
 *
 * NetworkTable driveTable = NetworkTableInstance.getDefault().getTable("Drivetrain");
 * states.enable();
 * states.setupNetworkTable(driveTable);
 *
 * // In periodic:
 * states.set(drivetrain.getModuleStates());
 * }</pre>
 *
 * @param <T> Type of the array elements being published, must be {@link Struct} serializable.
 * @param <F> Enum type identifying which field this telemetry entry represents.
 */
public class StructArrayTelemetry<T, F>
{
  /**
   * Struct serializer for {@link T}.
   */
  private final Struct<T> struct;
  /**
   * Network table key.
   */
  private final String key;
  /**
   * Tunable?
   */
  private final boolean tunable;
  /**
   * Enabled?
   */
  protected boolean enabled = false;
  /**
   * Default value.
   */
  private T[] defaultValue;
  /**
   * Cached value.
   */
  private T[] cachedValue;
  /**
   * Publisher.
   */
  private Optional<StructArrayPublisher<T>> publisher = Optional.empty();
  /**
   * Subscriber.
   */
  private Optional<StructArraySubscriber<T>> subscriber = Optional.empty();
  /**
   * Sub publisher.
   */
  private StructArrayPublisher<T> subPublisher = null;
  /**
   * Tuning table
   */
  private Optional<NetworkTable> tuningTable = Optional.empty();
  /**
   * Data table.
   */
  private Optional<NetworkTable> dataTable = Optional.empty();
  /**
   * NT4 Topic of this entry.
   */
  private StructArrayTopic<T> topic;
  /**
   * {@link StructArrayLogEntry} representing this entry.
   */
  private Optional<StructArrayLogEntry<T>> dataLogEntry = Optional.empty();
  /**
   * Telemetry enum field.
   */
  private F field;

  /**
   * Setup struct array telemetry for a field.
   *
   * @param keyString  Key to use.
   * @param defaultVal Default value.
   * @param field      Field representing.
   * @param struct     {@link Struct} serializer for {@link T}.
   * @param tunable    Tunable.
   */
  public StructArrayTelemetry(
      String keyString, T[] defaultVal, F field, Struct<T> struct, boolean tunable) {
    key = keyString;
    cachedValue = defaultValue = defaultVal;
    this.field = field;
    this.struct = struct;
    this.tunable = tunable;
  }

  /**
   * Set default values.
   *
   * @param defaultValue Default for the entry.
   */
  public void setDefaultValue(T[] defaultValue) {
    cachedValue = this.defaultValue = defaultValue;
  }

  /**
   * Setup network tables.
   *
   * @param dataTable   Data tables.
   * @param tuningTable Tuning table.
   */
  public void setupNetworkTables(NetworkTable dataTable, NetworkTable tuningTable) {
    this.tuningTable = Optional.ofNullable(tuningTable);
    this.dataTable = Optional.ofNullable(dataTable);
    if (!enabled) {
      return;
    }
    if (tuningTable != null && tunable) {
      topic = tuningTable.getStructArrayTopic(key, struct);
      subPublisher = topic.publish();
      subscriber = Optional.of(topic.subscribe(defaultValue));
      subPublisher.setDefault(defaultValue);
    } else {
      assert dataTable != null;
      topic = dataTable.getStructArrayTopic(key, struct);
      publisher = Optional.of(topic.publish());
      publisher.get().setDefault(defaultValue);
    }
  }

  /**
   * Setup the {@link edu.wpi.first.util.datalog.DataLog} with this entry.
   *
   * @param prefix The prefix to this entry in {@link edu.wpi.first.util.datalog.DataLog}
   */
  public void setupDataLog(String prefix) {
    if (!tunable) {
      if (!prefix.endsWith("/")) {
        prefix += "/";
      }
      dataLogEntry = Optional.of(StructArrayLogEntry.create(
          DataLogManager.getLog(), prefix + key, struct, (long) Timer.getFPGATimestamp()));
    }
  }

  /**
   * Setup network tables.
   *
   * @param dataTable Data tables.
   */
  public void setupNetworkTable(NetworkTable dataTable) {
    setupNetworkTables(dataTable, null);
  }

  /**
   * Set the value of the publisher, checking to see if the value is the same as the subscriber.
   *
   * @param value Value to set.
   * @return True if value was able to be set.
   */
  public boolean set(T[] value) {
    if (!enabled) {
      return false;
    }
    if (dataLogEntry.isPresent()) {
      dataLogEntry.get().append(value, (long) Timer.getFPGATimestamp());
    }
    if (subscriber.isPresent()) {
      T[] tuningValue = subscriber.get().get(defaultValue);
      if (!Arrays.equals(tuningValue, value)) {
        return false;
      }
    }
    if (publisher.isPresent()) {
      publisher.get().accept(value);
    }
    return true;
  }

  /**
   * Get the value.
   *
   * @return value of telemetry.
   */
  public T[] get() {
    if (!enabled) {
      return defaultValue;
    }
    if (subscriber.isPresent()) {
      return subscriber.get().get(defaultValue);
    }
    throw new RuntimeException("Tuning table not configured for " + key + "!");
  }

  /**
   * Check to see if the value has changed.
   *
   * @return True if the value has changed.
   */
  public boolean tunable() {
    if (subscriber.isPresent() && tunable && enabled) {
      T[] tuningValue = subscriber.get().get(defaultValue);
      if (!Arrays.equals(tuningValue, cachedValue)) {
        cachedValue = tuningValue;
        return true;
      }
      return false;
    }
    return false;
  }

  /**
   * Enable the telemetry.
   */
  public void enable() {
    enabled = true;
  }

  /**
   * Disable the telemetry.
   */
  public void disable() {
    enabled = false;
  }

  /**
   * Display the telemetry.
   *
   * @param state Enable or disable.
   */
  public void display(boolean state) {
    enabled = state;
  }

  /**
   * Close the telemetry field.
   */
  public void close() {
    subscriber.ifPresent(PubSub::close);
    if (subPublisher != null) {
      subPublisher.close();
    }
    publisher.ifPresent(PubSub::close);
    dataTable.ifPresent(table -> table.getEntry(key).unpublish());
    tuningTable.ifPresent(table -> table.getEntry(key).unpublish());
  }

  /**
   * Get the field.
   *
   * @return field.
   */
  public F getField() {
    return field;
  }
}
