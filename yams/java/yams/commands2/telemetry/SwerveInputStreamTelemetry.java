// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package yams.commands2.telemetry;

import static org.wpilib.units.Units.MetersPerSecond;
import static org.wpilib.units.Units.Radians;
import static org.wpilib.units.Units.RadiansPerSecond;

import java.util.EnumMap;
import java.util.Map;
import java.util.Objects;
import java.util.Optional;
import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import org.wpilib.networktables.NetworkTable;
import org.wpilib.networktables.NetworkTableInstance;
import org.wpilib.tunable.Tunables;
import yams.commands2.swerve.SwerveInputStream;
import yams.core.telemetry.BooleanTelemetry;
import yams.core.telemetry.DoubleTelemetry;
import yams.core.telemetry.NetworkTablesBackends;
import yams.core.telemetry.StringTelemetry;
import yams.core.telemetry.enums.TelemetryVerbosity;

/**
 * Telemetry and live tuning for a {@link SwerveInputStream}, created by
 * {@link SwerveInputStream#withTelemetry(String, TelemetryVerbosity)} and published every time the stream is read.
 *
 * <ul>
 *   <li>{@link TelemetryVerbosity#LOW}: the current drive mode, under {@code SwerveInputStream/<name>}.</li>
 *   <li>{@link TelemetryVerbosity#MID}: also the stream's configuration (deadband, scales, maximum velocities, cubing,
 *   alliance and robot relative), read-only, and the rotation controller's {@code measurement} and {@code setpoint}
 *   under {@code rotationController}, in radians in the gyro's frame. Those are what heading, aim and translation only
 *   control give the controller, and are {@code NaN} while the stream isn't using it.</li>
 *   <li>{@link TelemetryVerbosity#HIGH}: also editable copies of the configuration under
 *   {@code Tuning/SwerveInputStream/<name>}, and a {@code Live Tuning} command there that applies them to the stream
 *   every loop while it runs.</li>
 * </ul>
 *
 * <p>Each value is a {@link DoubleTelemetryField}, {@link BooleanTelemetryField} or {@link StringTelemetryField}, which
 * gives its key and whether it is tunable.
 *
 * <p>While live tuning, values stay in sync both ways: a dashboard edit is applied to the stream, and a change made in
 * code, e.g. a binding that changes the translation scale, is published to the dashboard. Invalid dashboard values are
 * replaced with the stream's current value.
 */
public class SwerveInputStreamTelemetry implements AutoCloseable {
  /** Stream to monitor and tune. */
  private final SwerveInputStream                                                   stream;
  /** Double telemetry fields that will be outputted. */
  private final Map<DoubleTelemetryField, DoubleTelemetry<DoubleTelemetryField>>    doubleFields = new EnumMap<>(DoubleTelemetryField.class);
  /** Boolean telemetry fields that will be outputted. */
  private final Map<BooleanTelemetryField, BooleanTelemetry<BooleanTelemetryField>> boolFields   = new EnumMap<>(BooleanTelemetryField.class);
  /** String telemetry fields that will be outputted. */
  private final Map<StringTelemetryField, StringTelemetry<StringTelemetryField>>    stringFields = new EnumMap<>(StringTelemetryField.class);
  /** Command that applies the editable configuration while it runs, at {@link TelemetryVerbosity#HIGH}. */
  private final Optional<Command>                                                   liveTuningCommand;
  /** Path {@link #liveTuningCommand} is published to with {@link Tunables}. */
  private final String                                                              liveTuningPath;

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
    for (DoubleTelemetryField field : DoubleTelemetryField.values()) {
      doubleFields.put(field, field.create());
    }
    for (BooleanTelemetryField field : BooleanTelemetryField.values()) {
      boolFields.put(field, field.create());
    }
    for (StringTelemetryField field : StringTelemetryField.values()) {
      stringFields.put(field, field.create());
    }
    switch (verbosity) {
      case HIGH:
        doubleFields.get(DoubleTelemetryField.TunableDeadband).enable();
        doubleFields.get(DoubleTelemetryField.TunableTranslationScale).enable();
        doubleFields.get(DoubleTelemetryField.TunableRotationScale).enable();
        doubleFields.get(DoubleTelemetryField.TunableMaxLinearVelocity).enable();
        doubleFields.get(DoubleTelemetryField.TunableMaxAngularVelocity).enable();
        boolFields.get(BooleanTelemetryField.TunableTranslationCube).enable();
        boolFields.get(BooleanTelemetryField.TunableRotationCube).enable();
        boolFields.get(BooleanTelemetryField.TunableAllianceRelative).enable();
        boolFields.get(BooleanTelemetryField.TunableRobotRelative).enable();
      case MID:
        doubleFields.get(DoubleTelemetryField.Deadband).enable();
        doubleFields.get(DoubleTelemetryField.TranslationScale).enable();
        doubleFields.get(DoubleTelemetryField.RotationScale).enable();
        doubleFields.get(DoubleTelemetryField.MaxLinearVelocity).enable();
        doubleFields.get(DoubleTelemetryField.MaxAngularVelocity).enable();
        doubleFields.get(DoubleTelemetryField.RotationControllerMeasurement).enable();
        doubleFields.get(DoubleTelemetryField.RotationControllerSetpoint).enable();
        boolFields.get(BooleanTelemetryField.TranslationCube).enable();
        boolFields.get(BooleanTelemetryField.RotationCube).enable();
        boolFields.get(BooleanTelemetryField.AllianceRelative).enable();
        boolFields.get(BooleanTelemetryField.RobotRelative).enable();
      case LOW:
        stringFields.get(StringTelemetryField.Mode).enable();
    }

    NetworkTableInstance instance    = NetworkTableInstance.getDefault();
    NetworkTable         dataTable   = instance.getTable("SwerveInputStream").getSubTable(name);
    NetworkTable         tuningTable = instance.getTable("Tuning").getSubTable("SwerveInputStream").getSubTable(name);
    for (DoubleTelemetry<DoubleTelemetryField> dt : doubleFields.values()) {
      if (dt.isEnabled()) {
        dt.setupNetworkTables(dataTable, tuningTable);
        // Skip a value an earlier telemetry left on the dashboard, so the stream's own value is published instead.
        dt.tunable();
      }
    }
    for (BooleanTelemetry<BooleanTelemetryField> bt : boolFields.values()) {
      if (bt.isEnabled()) {
        bt.setupNetworkTables(dataTable, tuningTable);
        bt.tunable();
      }
    }
    for (StringTelemetry<StringTelemetryField> st : stringFields.values()) {
      if (st.isEnabled()) {
        st.setupNetworkTables(dataTable, tuningTable);
      }
    }

    liveTuningPath = "Tuning/SwerveInputStream/" + name + "/Live Tuning";
    if (verbosity == TelemetryVerbosity.HIGH) {
      // Publish the stream's values to the Tuning table.
      applyTuningValues();
      // No requirements, so tuning does not interrupt the command driving with the stream.
      Command command = Commands.run(this::applyTuningValues).withName("Live Tuning");
      NetworkTablesBackends.ensureTuningTunableBackend();
      Tunables.publish(liveTuningPath, command);
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
    for (StringTelemetry<StringTelemetryField> st : stringFields.values()) {
      if (!st.isEnabled()) {
        continue;
      }
      switch (st.getField()) {
        case Mode -> st.set(stream.getCurrentModeName());
      }
    }
    for (DoubleTelemetry<DoubleTelemetryField> dt : doubleFields.values()) {
      if (!dt.isEnabled()) {
        continue;
      }
      switch (dt.getField()) {
        case Deadband -> dt.set(stream.getAxisDeadband());
        case TranslationScale -> dt.set(stream.getTranslationAxisScale());
        case RotationScale -> dt.set(stream.getOmegaAxisScale());
        case MaxLinearVelocity -> dt.set(stream.getMaximumChassisLinearVelocity().in(MetersPerSecond));
        case MaxAngularVelocity -> dt.set(stream.getMaximumChassisAngularVelocity().in(RadiansPerSecond));
        case RotationControllerMeasurement ->
            dt.set(stream.getRotationControllerMeasurement().map(angle -> angle.in(Radians)).orElse(Double.NaN));
        case RotationControllerSetpoint ->
            dt.set(stream.getRotationControllerSetpoint().map(angle -> angle.in(Radians)).orElse(Double.NaN));
        default -> {
          // Tunable fields are published by applyTuningValues.
        }
      }
    }
    for (BooleanTelemetry<BooleanTelemetryField> bt : boolFields.values()) {
      if (!bt.isEnabled()) {
        continue;
      }
      switch (bt.getField()) {
        case TranslationCube -> bt.set(stream.isTranslationCubeEnabled());
        case RotationCube -> bt.set(stream.isOmegaCubeEnabled());
        case AllianceRelative -> bt.set(stream.isAllianceRelativeEnabled());
        case RobotRelative -> bt.set(stream.isRobotRelativeEnabled());
        default -> {
          // Tunable fields are published by applyTuningValues.
        }
      }
    }
  }

  /**
   * Apply values edited on the dashboard to the stream, and publish changes made to the stream in code. The
   * {@link #getLiveTuningCommand() Live Tuning} command calls this every loop. Does nothing below
   * {@link TelemetryVerbosity#HIGH}.
   */
  public void applyTuningValues() {
    for (DoubleTelemetry<DoubleTelemetryField> dt : doubleFields.values()) {
      if (!dt.isEnabled()) {
        continue;
      }
      switch (dt.getField()) {
        case TunableDeadband -> {
          if (dt.tunable() && dt.get() >= 0.0 && dt.get() < 1.0) {
            stream.setAxisDeadband(dt.get());
          }
          publishStreamValue(dt, stream.getAxisDeadband());
        }
        case TunableTranslationScale -> {
          if (dt.tunable() && dt.get() > 0.0 && dt.get() <= 1.0) {
            stream.setTranslationAxisScale(dt.get());
          }
          publishStreamValue(dt, stream.getTranslationAxisScale());
        }
        case TunableRotationScale -> {
          if (dt.tunable() && dt.get() > 0.0 && dt.get() <= 1.0) {
            stream.setOmegaAxisScale(dt.get());
          }
          publishStreamValue(dt, stream.getOmegaAxisScale());
        }
        case TunableMaxLinearVelocity -> {
          if (dt.tunable() && dt.get() > 0.0 && Double.isFinite(dt.get())) {
            stream.setMaximumChassisLinearVelocity(MetersPerSecond.of(dt.get()));
          }
          publishStreamValue(dt, stream.getMaximumChassisLinearVelocity().in(MetersPerSecond));
        }
        case TunableMaxAngularVelocity -> {
          if (dt.tunable() && dt.get() > 0.0 && Double.isFinite(dt.get())) {
            stream.setMaximumChassisAngularVelocity(RadiansPerSecond.of(dt.get()));
          }
          publishStreamValue(dt, stream.getMaximumChassisAngularVelocity().in(RadiansPerSecond));
        }
        default -> {
          // Read-only fields are published by updateTelemetry.
        }
      }
    }
    for (BooleanTelemetry<BooleanTelemetryField> bt : boolFields.values()) {
      if (!bt.isEnabled()) {
        continue;
      }
      switch (bt.getField()) {
        case TunableTranslationCube -> {
          if (bt.tunable()) {
            stream.setTranslationCubeEnabled(bt.get());
          }
          publishStreamValue(bt, stream.isTranslationCubeEnabled());
        }
        case TunableRotationCube -> {
          if (bt.tunable()) {
            stream.setOmegaCubeEnabled(bt.get());
          }
          publishStreamValue(bt, stream.isOmegaCubeEnabled());
        }
        case TunableAllianceRelative -> {
          if (bt.tunable()) {
            stream.setAllianceRelativeEnabled(bt.get());
          }
          publishStreamValue(bt, stream.isAllianceRelativeEnabled());
        }
        case TunableRobotRelative -> {
          if (bt.tunable()) {
            stream.setRobotRelativeEnabled(bt.get());
          }
          publishStreamValue(bt, stream.isRobotRelativeEnabled());
        }
        default -> {
          // Read-only fields are published by updateTelemetry.
        }
      }
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
      command.cancel();
      Tunables.remove(liveTuningPath);
    });
    for (DoubleTelemetry<DoubleTelemetryField> dt : doubleFields.values()) {
      dt.close();
    }
    for (BooleanTelemetry<BooleanTelemetryField> bt : boolFields.values()) {
      bt.close();
    }
    for (StringTelemetry<StringTelemetryField> st : stringFields.values()) {
      st.close();
    }
  }

  /**
   * Publish the stream's value of a tunable field if the dashboard shows another, as after a change made in code or an
   * invalid edit. The change is marked as seen, so it isn't applied back to the stream as a dashboard edit.
   *
   * @param dt          Tunable field.
   * @param streamValue The stream's value.
   */
  private static void publishStreamValue(DoubleTelemetry<DoubleTelemetryField> dt, double streamValue) {
    if (dt.get() != streamValue) {
      dt.forceSet(streamValue);
      dt.tunable();
    }
  }

  /**
   * Publish the stream's value of a tunable field if the dashboard shows another, as after a change made in code. The
   * change is marked as seen, so it isn't applied back to the stream as a dashboard edit.
   *
   * @param bt          Tunable field.
   * @param streamValue The stream's value.
   */
  private static void publishStreamValue(BooleanTelemetry<BooleanTelemetryField> bt, boolean streamValue) {
    if (bt.get() != streamValue) {
      bt.forceSet(streamValue);
      bt.tunable();
    }
  }

  /** Double telemetry field for {@link SwerveInputStream}s. */
  public enum DoubleTelemetryField {
    /** Joystick axis deadband. */
    Deadband("deadband", 0, false, "none"),
    /** Translation axis scale. */
    TranslationScale("translationScale", 1, false, "none"),
    /** Rotation axis scale. */
    RotationScale("rotationScale", 1, false, "none"),
    /** Maximum chassis linear velocity, in meters per second. */
    MaxLinearVelocity("maxLinearVelocity", 0, false, "meters_per_second"),
    /** Maximum chassis angular velocity, in radians per second. */
    MaxAngularVelocity("maxAngularVelocity", 0, false, "radians_per_second"),
    /**
     * Heading given to the rotation controller as its measurement, in radians in the gyro's frame. {@code NaN} while
     * the stream isn't using the controller.
     */
    RotationControllerMeasurement("rotationController/measurement", Double.NaN, false, "radians"),
    /**
     * Heading given to the rotation controller as its setpoint, in radians in the gyro's frame. {@code NaN} while the
     * stream isn't using the controller.
     */
    RotationControllerSetpoint("rotationController/setpoint", Double.NaN, false, "radians"),
    /** Tunable joystick axis deadband, from 0 up to but not including 1. */
    TunableDeadband("deadband", 0, true, "none"),
    /** Tunable translation axis scale, above 0 and up to 1. */
    TunableTranslationScale("translationScale", 1, true, "none"),
    /** Tunable rotation axis scale, above 0 and up to 1. */
    TunableRotationScale("rotationScale", 1, true, "none"),
    /** Tunable maximum chassis linear velocity, in meters per second. */
    TunableMaxLinearVelocity("maxLinearVelocity", 0, true, "meters_per_second"),
    /** Tunable maximum chassis angular velocity, in radians per second. */
    TunableMaxAngularVelocity("maxAngularVelocity", 0, true, "radians_per_second");

    /** Default value of the double telemetry field. */
    private final double  defaultVal;
    /** Key that the telemetry is stored at. */
    private final String  key;
    /** Tunable field? */
    private final boolean tunable;
    /** Unit of the telemetry field. */
    private final String  unit;

    /**
     * Create a double telemetry field.
     *
     * @param fieldName    NT Field Name
     * @param defaultValue Default value
     * @param tunable      Tunable places it only in the Tuning Table.
     * @param unit         Unit of the telemetry field.
     */
    DoubleTelemetryField(String fieldName, double defaultValue, boolean tunable, String unit) {
      key = fieldName;
      defaultVal = defaultValue;
      this.tunable = tunable;
      this.unit = unit;
    }

    /**
     * Create a {@link DoubleTelemetry} object for non-static usage.
     *
     * @return {@link DoubleTelemetry}
     */
    public DoubleTelemetry<DoubleTelemetryField> create() {
      return new DoubleTelemetry<>(key, defaultVal, this, tunable, unit);
    }
  }

  /** Boolean telemetry field for {@link SwerveInputStream}s. */
  public enum BooleanTelemetryField {
    /** Translation axes are cubed. */
    TranslationCube("translationCube", false, false),
    /** Rotation axis is cubed. */
    RotationCube("rotationCube", false, false),
    /** Driving is relative to the alliance. */
    AllianceRelative("allianceRelative", false, false),
    /** Driving is relative to the robot. */
    RobotRelative("robotRelative", false, false),
    /** Tunable translation axis cubing. */
    TunableTranslationCube("translationCube", false, true),
    /** Tunable rotation axis cubing. */
    TunableRotationCube("rotationCube", false, true),
    /** Tunable alliance relative driving. */
    TunableAllianceRelative("allianceRelative", false, true),
    /** Tunable robot relative driving. */
    TunableRobotRelative("robotRelative", false, true);

    /** Key that the telemetry is stored at. */
    private final String  key;
    /** Default value of the boolean telemetry field. */
    private final boolean defaultVal;
    /** Tunable field? */
    private final boolean tunable;

    /**
     * Create a boolean telemetry field.
     *
     * @param fieldName    NT Field Name
     * @param defaultValue Default value
     * @param tunable      Tunable places it only in the Tuning Table.
     */
    BooleanTelemetryField(String fieldName, boolean defaultValue, boolean tunable) {
      key = fieldName;
      defaultVal = defaultValue;
      this.tunable = tunable;
    }

    /**
     * Create a {@link BooleanTelemetry} object for non-static usage.
     *
     * @return {@link BooleanTelemetry}
     */
    public BooleanTelemetry<BooleanTelemetryField> create() {
      return new BooleanTelemetry<>(key, defaultVal, this, tunable);
    }
  }

  /** String telemetry field for {@link SwerveInputStream}s. */
  public enum StringTelemetryField {
    /** Name of the stream's current drive mode. */
    Mode("mode", "", false);

    /** Key that the telemetry is stored at. */
    private final String  key;
    /** Default value of the string telemetry field. */
    private final String  defaultVal;
    /** Tunable field? */
    private final boolean tunable;

    /**
     * Create a string telemetry field.
     *
     * @param fieldName    NT Field Name
     * @param defaultValue Default value
     * @param tunable      Tunable places it only in the Tuning Table.
     */
    StringTelemetryField(String fieldName, String defaultValue, boolean tunable) {
      key = fieldName;
      defaultVal = defaultValue;
      this.tunable = tunable;
    }

    /**
     * Create a {@link StringTelemetry} object for non-static usage.
     *
     * @return {@link StringTelemetry}
     */
    public StringTelemetry<StringTelemetryField> create() {
      return new StringTelemetry<>(key, defaultVal, this, tunable);
    }
  }
}
