// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package yams.telemetry;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.PubSub;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.networktables.StructSubscriber;
import edu.wpi.first.networktables.StructTopic;
import edu.wpi.first.util.datalog.StructLogEntry;
import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Timer;
import java.util.Optional;

/**
 * Struct Telemetry for arbitrary {@link Struct} serializable types.
 *
 * <p>A lightweight wrapper that publishes a single struct-encoded value (e.g. {@code Pose2d},
 * {@code ChassisSpeeds}, {@code SwerveModuleState}) to NetworkTables and/or a WPILib DataLog. It
 * mirrors {@link DoubleTelemetry} and {@link BooleanTelemetry}, but is generic over the value
 * type {@code T} and requires a {@link Struct} implementation to (de)serialize it.
 *
 * <h2>Example</h2>
 * <pre>{@code
 * // Create and publish a struct entry for the robot pose under the Drivetrain table.
 * StructTelemetry<Pose2d> pose = new StructTelemetry<>(
 *     "pose",                 // NetworkTables key
 *     new Pose2d(),           // default value
 *     Pose2d.struct,          // struct serializer
 *     false);                 // not tunable
 *
 * NetworkTable driveTable = NetworkTableInstance.getDefault().getTable("Drivetrain");
 * pose.enable();
 * pose.setupNetworkTable(driveTable);
 *
 * // In periodic:
 * pose.set(drivetrain.getPose());
 * }</pre>
 *
 * @param <T> Type of the value being published, must be {@link Struct} serializable.
 */
public class StructTelemetry<T, F> {
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
  private T defaultValue;
  /**
   * Cached value.
   */
  private T cachedValue;
  /**
   * Publisher.
   */
  private Optional<StructPublisher<T>> publisher = Optional.empty();
  /**
   * Subscriber.
   */
  private Optional<StructSubscriber<T>> subscriber = Optional.empty();
  /**
   * Sub publisher.
   */
  private StructPublisher<T> subPublisher = null;
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
  private StructTopic<T> topic;
  /**
   * {@link StructLogEntry} representing this entry.
   */
  private Optional<StructLogEntry<T>> dataLogEntry = Optional.empty();
  /**
   * Telemetry enum field.
   */
  private F field;

  /**
   * Setup struct telemetry for a field.
   *
   * @param keyString  Key to use.
   * @param defaultVal Default value.
   * @param field      Field representing.
   * @param struct     {@link Struct} serializer for {@link T}.
   * @param tunable    Tunable.
   */
  public StructTelemetry(
      String keyString, T defaultVal, F field, Struct<T> struct, boolean tunable) {
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
  public void setDefaultValue(T defaultValue) {
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
      topic = tuningTable.getStructTopic(key, struct);
      subPublisher = topic.publish();
      subscriber = Optional.of(topic.subscribe(defaultValue));
      subPublisher.setDefault(defaultValue);
    } else {
      assert dataTable != null;
      topic = dataTable.getStructTopic(key, struct);
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
      dataLogEntry = Optional.of(StructLogEntry.create(
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
  public boolean set(T value) {
    if (!enabled) {
      return false;
    }
    if (dataLogEntry.isPresent()) {
      dataLogEntry.get().append(value, (long) Timer.getFPGATimestamp());
    }
    if (subscriber.isPresent()) {
      T tuningValue = subscriber.get().get(defaultValue);
      if (!tuningValue.equals(value)) {
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
  public T get() {
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
      T tuningValue = subscriber.get().get(defaultValue);
      if (!tuningValue.equals(cachedValue)) {
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
  public F getField() {
    return field;
  }
}
