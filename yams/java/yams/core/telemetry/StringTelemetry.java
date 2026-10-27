// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package yams.core.telemetry;

import java.util.Optional;
import org.wpilib.datalog.StringLogEntry;
import org.wpilib.networktables.NetworkTable;
import org.wpilib.networktables.PubSub;
import org.wpilib.networktables.StringPublisher;
import org.wpilib.networktables.StringSubscriber;
import org.wpilib.networktables.StringTopic;
import org.wpilib.system.DataLogManager;
import org.wpilib.system.Timer;

/**
 * String Telemetry.
 *
 * <p>A lightweight wrapper that publishes a single {@code String} value to NetworkTables and/or a
 * WPILib DataLog, such as the name of the mode a mechanism is in.
 *
 * <h2>Example</h2>
 * <pre>{@code
 * StringTelemetry<MyField> mode = new StringTelemetry<>("mode", "", MyField.Mode, false);
 * mode.enable();
 * mode.setupNetworkTable(NetworkTableInstance.getDefault().getTable("Drive"));
 *
 * // In periodic:
 * mode.set(drive.getModeName());
 * }</pre>
 *
 * @param <F> Enum type identifying which field this telemetry entry represents.
 */
public class StringTelemetry<F> {
  /** Field representing. */
  private final F field;
  /** Network table key. */
  private final String key;
  /** Tunable? */
  private final boolean tunable;
  /** Enabled? */
  protected boolean enabled = false;
  /** Default value. */
  private String defaultValue;
  /** Cached value. */
  private String cachedValue;
  /** Publisher. */
  private StringPublisher publisher = null;
  /** Subscriber. */
  private Optional<StringSubscriber> subscriber = Optional.empty();
  /** Sub publisher. */
  private StringPublisher pubSub = null;
  /** pub or sub topic. */
  private StringTopic topic;
  /** DataLog entry. */
  private Optional<StringLogEntry> dataLogEntry = Optional.empty();
  /** Tuning table */
  private Optional<NetworkTable> tuningTable = Optional.empty();
  /** Data table. */
  private Optional<NetworkTable> dataTable = Optional.empty();

  /**
   * Setup string telemetry for a field.
   *
   * @param keyString  Networks table key.
   * @param defaultVal Default value.
   * @param field      Field representing.
   * @param tunable    Tunable?
   */
  public StringTelemetry(String keyString, String defaultVal, F field, boolean tunable) {
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
  public void setupNetworkTables(NetworkTable dataTable, NetworkTable tuningTable) {
    this.dataTable = Optional.ofNullable(dataTable);
    this.tuningTable = Optional.ofNullable(tuningTable);
    if (tuningTable != null && tunable) {
      topic = tuningTable.getStringTopic(key);
      pubSub = topic.publish();
      pubSub.setDefault(defaultValue);
      subscriber = Optional.of(topic.subscribe(defaultValue));
    } else {
      topic = dataTable.getStringTopic(key);
      publisher = topic.publish();
      publisher.setDefault(defaultValue);
    }
  }

  /**
   * Setup the DataLog entry for this telemetry field.
   *
   * @param prefix Prefix of the entry.
   */
  public void setupDataLog(String prefix) {
    if (!tunable) {
      if (!prefix.endsWith("/")) {
        prefix += "/";
      }
      dataLogEntry = Optional.of(new StringLogEntry(DataLogManager.getLog(), prefix + key, (long) Timer.getTimestamp()));
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
  public boolean set(String value) {
    if (!enabled) {
      return false;
    }
    if (dataLogEntry.isPresent()) {
      dataLogEntry.get().append(value);
    }
    if (subscriber.isPresent()) {
      String tuningValue = subscriber.get().get(defaultValue);
      if (!tuningValue.equals(value)) {
        return false;
      }
    }
    if (publisher != null) {
      publisher.accept(value);
    }
    return true;
  }

  /**
   * Forcibly overwrite the tunable value in NetworkTables, bypassing the subscriber-must-match
   * guard in {@link #set(String)}. The next {@link #tunable()} reports the change, as if it was made
   * on the dashboard.
   *
   * @param value Value to force onto the tuning topic.
   */
  public void forceSet(String value) {
    if (pubSub != null) {
      pubSub.accept(value);
    } else if (publisher != null) {
      publisher.accept(value);
    }
  }

  /**
   * Get the value.
   *
   * @return Value.
   * @throws RuntimeException if this field is not tunable, so no tuning subscriber exists.
   * @throws RuntimeException if {@link #setupNetworkTables(NetworkTable, NetworkTable)} was not
   *                          called with a non-null tuning table, so no tuning subscriber exists.
   */
  public String get() {
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
      String value = subscriber.get().get(defaultValue);
      if (!value.equals(cachedValue)) {
        cachedValue = value;
        return true;
      }
      return false;
    }
    return false;
  }

  /** Enable the telemetry. */
  public void enable() {
    enabled = true;
  }

  /** Disable the telemetry. */
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
   * Whether the telemetry is enabled.
   *
   * @return True if the telemetry is published.
   */
  public boolean isEnabled() {
    return enabled;
  }

  /**
   * Get the field.
   *
   * @return field.
   */
  public F getField() {
    return field;
  }

  /**
   * Set the default value.
   *
   * @param value Default value.
   */
  public void setDefaultValue(String value) {
    defaultValue = value;
    cachedValue = value;
  }

  /** Close the telemetry field. */
  public void close() {
    subscriber.ifPresent(PubSub::close);
    if (pubSub != null) {
      pubSub.close();
    }
    if (publisher != null) {
      publisher.close();
    }
    dataTable.ifPresent(table -> table.getEntry(key).unpublish());
    tuningTable.ifPresent(table -> table.getEntry(key).unpublish());
  }
}
