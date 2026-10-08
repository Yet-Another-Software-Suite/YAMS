// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package yams.commands2.telemetry;

import static org.wpilib.units.Units.MetersPerSecond;
import static org.wpilib.units.Units.RadiansPerSecond;

import java.util.List;
import java.util.Objects;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleConsumer;
import java.util.function.DoublePredicate;
import java.util.function.DoubleSupplier;
import org.wpilib.networktables.BooleanEntry;
import org.wpilib.networktables.DoubleEntry;
import org.wpilib.networktables.NetworkTable;
import org.wpilib.networktables.NetworkTableInstance;
import org.wpilib.networktables.StringPublisher;
import yams.commands2.swerve.SwerveInputStream;

/**
 * Telemetry and live tuning support for {@link SwerveInputStream}.
 *
 * <p>Publishes the current mode and configuration of a {@link SwerveInputStream} to NetworkTables under
 * {@code SwerveInputStream/<name>}, and applies values edited on the dashboard to the stream. Tunable values stay in
 * sync both ways: a dashboard edit is applied to the stream on the next {@link #update()}, and a change made in code,
 * e.g. a binding that changes the translation scale, is published to the dashboard. Invalid dashboard values are
 * replaced with the stream's current value.
 *
 * <p>Usage:
 *
 * <pre>{@code
 * SwerveInputStream driveStream = SwerveInputStream.of(drive, leftY, leftX)
 *     .withControllerRotationAxis(rightX)
 *     .withDeadband(0.05);
 *
 * SwerveInputStreamTelemetry telemetry = new SwerveInputStreamTelemetry(driveStream, "drive");
 * telemetry.update(); // Call once per loop, e.g. in your command's loop before driveStream.get()
 * }</pre>
 */
public class SwerveInputStreamTelemetry implements AutoCloseable {
  /** Stream to monitor and tune. */
  private final SwerveInputStream stream;
  /** Current drive mode publisher. */
  private final StringPublisher modePublisher;
  /** Tunable values, kept in sync between the stream and NetworkTables. */
  private final List<TunableValue> tunableValues;

  /**
   * Create telemetry for a {@link SwerveInputStream} under {@code SwerveInputStream/<name>}.
   *
   * @param stream The {@link SwerveInputStream} to monitor and tune.
   * @param name   The NetworkTables subtable name (e.g., "drive").
   */
  public SwerveInputStreamTelemetry(SwerveInputStream stream, String name) {
    this(stream, NetworkTableInstance.getDefault().getTable("SwerveInputStream").getSubTable(name));
  }

  /**
   * Create telemetry for a {@link SwerveInputStream} in the given table.
   *
   * @param stream The {@link SwerveInputStream} to monitor and tune.
   * @param table  The {@link NetworkTable} to publish to.
   */
  public SwerveInputStreamTelemetry(SwerveInputStream stream, NetworkTable table) {
    this.stream = Objects.requireNonNull(stream, "stream cannot be null");
    Objects.requireNonNull(table, "table cannot be null");
    modePublisher = table.getStringTopic("mode").publish();
    modePublisher.set(stream.getCurrentModeName());
    tunableValues = List.of(
        new TunableDouble(table, "deadband",
                          stream::getAxisDeadband, stream::setAxisDeadband,
                          value -> value >= 0.0 && value < 1.0),
        new TunableDouble(table, "translationScale",
                          stream::getTranslationAxisScale, stream::setTranslationAxisScale,
                          value -> value > 0.0 && value <= 1.0),
        new TunableDouble(table, "rotationScale",
                          stream::getOmegaAxisScale, stream::setOmegaAxisScale,
                          value -> value > 0.0 && value <= 1.0),
        new TunableDouble(table, "maxLinearVelocity",
                          () -> stream.getMaximumChassisLinearVelocity().in(MetersPerSecond),
                          value -> stream.setMaximumChassisLinearVelocity(MetersPerSecond.of(value)),
                          value -> value > 0.0 && Double.isFinite(value)),
        new TunableDouble(table, "maxAngularVelocity",
                          () -> stream.getMaximumChassisAngularVelocity().in(RadiansPerSecond),
                          value -> stream.setMaximumChassisAngularVelocity(RadiansPerSecond.of(value)),
                          value -> value > 0.0 && Double.isFinite(value)),
        new TunableBoolean(table, "translationCube",
                           stream::isTranslationCubeEnabled, stream::setTranslationCubeEnabled),
        new TunableBoolean(table, "rotationCube",
                           stream::isOmegaCubeEnabled, stream::setOmegaCubeEnabled),
        new TunableBoolean(table, "allianceRelative",
                           stream::isAllianceRelativeEnabled, stream::setAllianceRelativeEnabled),
        new TunableBoolean(table, "robotRelative",
                           stream::isRobotRelativeEnabled, stream::setRobotRelativeEnabled));
  }

  /**
   * Publish the current state and apply any live-tuned values.
   *
   * <p>Call this method once per robot loop, e.g. in periodic() or in your command's loop, before reading the stream.
   */
  public void update() {
    modePublisher.set(stream.getCurrentModeName());
    for (TunableValue value : tunableValues) {
      value.update();
    }
  }

  /** Stop publishing this stream's telemetry. */
  @Override
  public void close() {
    modePublisher.close();
    for (TunableValue value : tunableValues) {
      value.close();
    }
  }

  /** A stream value kept in sync with a NetworkTables entry. */
  private interface TunableValue extends AutoCloseable {
    /** Apply a dashboard edit to the stream, or publish a change made in code. */
    void update();

    @Override
    void close();
  }

  /** A tunable {@code double} stream value. */
  private static final class TunableDouble implements TunableValue {
    /** Entry the dashboard edits. */
    private final DoubleEntry     entry;
    /** Reads the value from the stream. */
    private final DoubleSupplier  getter;
    /** Applies a value to the stream. */
    private final DoubleConsumer  setter;
    /** Whether a dashboard value can be applied. */
    private final DoublePredicate valid;
    /** Value last published or applied, to tell dashboard edits apart from changes made in code. */
    private double                lastValue;

    /**
     * Publish a tunable {@code double} stream value.
     *
     * @param table  Table to publish to.
     * @param key    Entry key.
     * @param getter Reads the value from the stream.
     * @param setter Applies a value to the stream.
     * @param valid  Whether a dashboard value can be applied.
     */
    TunableDouble(NetworkTable table, String key, DoubleSupplier getter, DoubleConsumer setter,
                  DoublePredicate valid) {
      this.getter = getter;
      this.setter = setter;
      this.valid = valid;
      lastValue = getter.getAsDouble();
      entry = table.getDoubleTopic(key).getEntry(lastValue);
      entry.set(lastValue);
    }

    @Override
    public void update() {
      double published = entry.get();
      if (published != lastValue && valid.test(published)) {
        setter.accept(published);
      }
      double current = getter.getAsDouble();
      if (current != published) {
        entry.set(current);
      }
      lastValue = current;
    }

    @Override
    public void close() {
      entry.unpublish();
      entry.close();
    }
  }

  /** A tunable {@code boolean} stream value. */
  private static final class TunableBoolean implements TunableValue {
    /** Entry the dashboard edits. */
    private final BooleanEntry    entry;
    /** Reads the value from the stream. */
    private final BooleanSupplier getter;
    /** Applies a value to the stream. */
    private final BooleanSetter   setter;
    /** Value last published or applied, to tell dashboard edits apart from changes made in code. */
    private boolean               lastValue;

    /**
     * Publish a tunable {@code boolean} stream value.
     *
     * @param table  Table to publish to.
     * @param key    Entry key.
     * @param getter Reads the value from the stream.
     * @param setter Applies a value to the stream.
     */
    TunableBoolean(NetworkTable table, String key, BooleanSupplier getter, BooleanSetter setter) {
      this.getter = getter;
      this.setter = setter;
      lastValue = getter.getAsBoolean();
      entry = table.getBooleanTopic(key).getEntry(lastValue);
      entry.set(lastValue);
    }

    @Override
    public void update() {
      boolean published = entry.get();
      if (published != lastValue) {
        setter.accept(published);
      }
      boolean current = getter.getAsBoolean();
      if (current != published) {
        entry.set(current);
      }
      lastValue = current;
    }

    @Override
    public void close() {
      entry.unpublish();
      entry.close();
    }
  }

  /** Applies a {@code boolean} value to the stream. */
  @FunctionalInterface
  private interface BooleanSetter {
    /**
     * Apply the value.
     *
     * @param value Value to apply.
     */
    void accept(boolean value);
  }
}
