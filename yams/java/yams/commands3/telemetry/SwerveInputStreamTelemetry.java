// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package yams.commands3.telemetry;

import static org.wpilib.units.Units.MetersPerSecond;
import static org.wpilib.units.Units.RadiansPerSecond;

import java.util.ArrayList;
import java.util.List;
import java.util.Objects;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleConsumer;
import java.util.function.DoublePredicate;
import java.util.function.DoubleSupplier;
import org.wpilib.command3.Command;
import org.wpilib.command3.Scheduler;
import org.wpilib.networktables.BooleanEntry;
import org.wpilib.networktables.BooleanPublisher;
import org.wpilib.networktables.DoubleEntry;
import org.wpilib.networktables.DoublePublisher;
import org.wpilib.networktables.NetworkTable;
import org.wpilib.networktables.NetworkTableInstance;
import org.wpilib.networktables.PubSub;
import org.wpilib.networktables.StringPublisher;
import org.wpilib.tunable.Tunables;
import yams.commands3.swerve.SwerveInputStream;
import yams.core.telemetry.NetworkTablesBackends;
import yams.core.telemetry.enums.TelemetryVerbosity;

/**
 * Telemetry and live tuning for a {@link SwerveInputStream}, created by
 * {@link SwerveInputStream#withTelemetry(String, TelemetryVerbosity)} and published every time the stream is read.
 *
 * <ul>
 *   <li>{@link TelemetryVerbosity#LOW}: the current drive mode, under {@code SwerveInputStream/<name>}.</li>
 *   <li>{@link TelemetryVerbosity#MID}: also the stream's configuration (deadband, scales, maximum velocities, cubing,
 *   alliance and robot relative), read-only.</li>
 *   <li>{@link TelemetryVerbosity#HIGH}: also editable copies of the configuration under
 *   {@code Tuning/SwerveInputStream/<name>}, and a {@code Live Tuning} command there that applies them to the stream
 *   every loop while it runs.</li>
 * </ul>
 *
 * <p>While live tuning, values stay in sync both ways: a dashboard edit is applied to the stream, and a change made in
 * code, e.g. a binding that changes the translation scale, is published to the dashboard. Invalid dashboard values are
 * replaced with the stream's current value.
 */
public class SwerveInputStreamTelemetry implements AutoCloseable {
  /** Stream to monitor and tune. */
  private final SwerveInputStream      stream;
  /** Current drive mode publisher. */
  private final StringPublisher        modePublisher;
  /** Read-only configuration, published at {@link TelemetryVerbosity#MID} and above. */
  private final List<Runnable>         configPublishers = new ArrayList<>();
  /** NetworkTables publishers to close with this telemetry. */
  private final List<PubSub>           pubSubs          = new ArrayList<>();
  /** Editable configuration, at {@link TelemetryVerbosity#HIGH}. */
  private final List<TunableValue>     tunableValues    = new ArrayList<>();
  /** Command that applies the editable configuration while it runs, at {@link TelemetryVerbosity#HIGH}. */
  private final Optional<Command>      liveTuningCommand;
  /** Path {@link #liveTuningCommand} is published to with {@link Tunables}. */
  private final String                 liveTuningPath;

  /**
   * Publish telemetry for a {@link SwerveInputStream}. Use
   * {@link SwerveInputStream#withTelemetry(String, TelemetryVerbosity)} rather than calling this directly.
   *
   * @param stream    The {@link SwerveInputStream} to monitor and tune.
   * @param name      Name of the stream in NetworkTables (e.g., "drive").
   * @param verbosity {@link TelemetryVerbosity} to publish at.
   */
  public SwerveInputStreamTelemetry(SwerveInputStream stream, String name, TelemetryVerbosity verbosity) {
    this.stream = Objects.requireNonNull(stream, "stream cannot be null");
    Objects.requireNonNull(name, "name cannot be null");
    Objects.requireNonNull(verbosity, "verbosity cannot be null");
    NetworkTableInstance instance = NetworkTableInstance.getDefault();
    NetworkTable dataTable = instance.getTable("SwerveInputStream").getSubTable(name);
    modePublisher = track(dataTable.getStringTopic("mode").publish());
    liveTuningPath = "Tuning/SwerveInputStream/" + name + "/Live Tuning";

    if (verbosity != TelemetryVerbosity.LOW) {
      publishConfig(dataTable);
    }
    if (verbosity == TelemetryVerbosity.HIGH) {
      addTunableValues(instance.getTable("Tuning").getSubTable("SwerveInputStream").getSubTable(name));
      // No requirements, so tuning does not interrupt the command driving with the stream.
      Command command = Command.noRequirements(coroutine -> {
        while (true) {
          applyTuningValues();
          coroutine.yield();
        }
      }).named("Live Tuning");
      NetworkTablesBackends.ensureTuningTunableBackend();
      Tunables.publish(liveTuningPath, new CommandTunable(command));
      liveTuningCommand = Optional.of(command);
    } else {
      liveTuningCommand = Optional.empty();
    }
    updateTelemetry();
  }

  /**
   * Publish the stream's current mode and, at {@link TelemetryVerbosity#MID} and above, its configuration. Called by
   * {@link SwerveInputStream#get()}.
   */
  public void updateTelemetry() {
    modePublisher.set(stream.getCurrentModeName());
    for (Runnable publisher : configPublishers) {
      publisher.run();
    }
  }

  /**
   * Apply values edited on the dashboard to the stream, and publish changes made to the stream in code. The
   * {@link #getLiveTuningCommand() Live Tuning} command calls this every loop. Does nothing below
   * {@link TelemetryVerbosity#HIGH}.
   */
  public void applyTuningValues() {
    for (TunableValue value : tunableValues) {
      value.update();
    }
  }

  /**
   * The {@code Live Tuning} command published to the dashboard, which applies dashboard edits while it runs.
   *
   * @return The command at {@link TelemetryVerbosity#HIGH}, otherwise empty.
   */
  public Optional<Command> getLiveTuningCommand() {
    return liveTuningCommand;
  }

  /** Stop publishing this stream's telemetry, canceling and removing the {@code Live Tuning} command. */
  @Override
  public void close() {
    liveTuningCommand.ifPresent(command -> {
      Scheduler.getDefault().cancel(command);
      Tunables.remove(liveTuningPath);
    });
    for (TunableValue value : tunableValues) {
      value.close();
    }
    for (PubSub pubSub : pubSubs) {
      pubSub.close();
    }
  }

  private <T extends PubSub> T track(T pubSub) {
    pubSubs.add(pubSub);
    return pubSub;
  }

  private void publishConfig(NetworkTable table) {
    publishDouble(table, "deadband", stream::getAxisDeadband);
    publishDouble(table, "translationScale", stream::getTranslationAxisScale);
    publishDouble(table, "rotationScale", stream::getOmegaAxisScale);
    publishDouble(table, "maxLinearVelocity", () -> stream.getMaximumChassisLinearVelocity().in(MetersPerSecond));
    publishDouble(table, "maxAngularVelocity", () -> stream.getMaximumChassisAngularVelocity().in(RadiansPerSecond));
    publishBoolean(table, "translationCube", stream::isTranslationCubeEnabled);
    publishBoolean(table, "rotationCube", stream::isOmegaCubeEnabled);
    publishBoolean(table, "allianceRelative", stream::isAllianceRelativeEnabled);
    publishBoolean(table, "robotRelative", stream::isRobotRelativeEnabled);
  }

  private void publishDouble(NetworkTable table, String key, DoubleSupplier value) {
    DoublePublisher publisher = track(table.getDoubleTopic(key).publish());
    configPublishers.add(() -> publisher.set(value.getAsDouble()));
  }

  private void publishBoolean(NetworkTable table, String key, BooleanSupplier value) {
    BooleanPublisher publisher = track(table.getBooleanTopic(key).publish());
    configPublishers.add(() -> publisher.set(value.getAsBoolean()));
  }

  private void addTunableValues(NetworkTable table) {
    tunableValues.add(new TunableDouble(table, "deadband",
                                        stream::getAxisDeadband, stream::setAxisDeadband,
                                        value -> value >= 0.0 && value < 1.0));
    tunableValues.add(new TunableDouble(table, "translationScale",
                                        stream::getTranslationAxisScale, stream::setTranslationAxisScale,
                                        value -> value > 0.0 && value <= 1.0));
    tunableValues.add(new TunableDouble(table, "rotationScale",
                                        stream::getOmegaAxisScale, stream::setOmegaAxisScale,
                                        value -> value > 0.0 && value <= 1.0));
    tunableValues.add(new TunableDouble(table, "maxLinearVelocity",
                                        () -> stream.getMaximumChassisLinearVelocity().in(MetersPerSecond),
                                        value -> stream.setMaximumChassisLinearVelocity(MetersPerSecond.of(value)),
                                        value -> value > 0.0 && Double.isFinite(value)));
    tunableValues.add(new TunableDouble(table, "maxAngularVelocity",
                                        () -> stream.getMaximumChassisAngularVelocity().in(RadiansPerSecond),
                                        value -> stream.setMaximumChassisAngularVelocity(RadiansPerSecond.of(value)),
                                        value -> value > 0.0 && Double.isFinite(value)));
    tunableValues.add(new TunableBoolean(table, "translationCube",
                                         stream::isTranslationCubeEnabled, stream::setTranslationCubeEnabled));
    tunableValues.add(new TunableBoolean(table, "rotationCube",
                                         stream::isOmegaCubeEnabled, stream::setOmegaCubeEnabled));
    tunableValues.add(new TunableBoolean(table, "allianceRelative",
                                         stream::isAllianceRelativeEnabled, stream::setAllianceRelativeEnabled));
    tunableValues.add(new TunableBoolean(table, "robotRelative",
                                         stream::isRobotRelativeEnabled, stream::setRobotRelativeEnabled));
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
