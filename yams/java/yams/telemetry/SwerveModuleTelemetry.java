// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package yams.telemetry;

import static org.wpilib.units.Units.Degrees;

import org.wpilib.math.kinematics.SwerveModuleVelocity;
import org.wpilib.networktables.NetworkTable;
import org.wpilib.networktables.NetworkTableInstance;
import org.wpilib.util.struct.Struct;
import java.util.Map;
import yams.mechanisms.swerve.SwerveModule;

/**
 * Swerve module telemetry.
 *
 * <p>Publishes the module's raw absolute encoder angle (without offsets applied) and current
 * {@link SwerveModuleVelocity} to NetworkTables and/or a WPILib DataLog, and wires the drive/azimuth
 * {@link yams.motorcontrollers.SmartMotorController}s' own telemetry under the same module
 * subtable.
 *
 * <p>This class is managed internally by {@link SwerveModule}. You do not normally instantiate
 * it directly; instead configure telemetry through {@link
 * yams.mechanisms.config.SwerveModuleConfig#withTelemetry(String, SwerveModuleTelemetryConfig)}
 * or {@link yams.mechanisms.config.SwerveModuleConfig#withTelemetry(String,
 * yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity)} before constructing the
 * module.
 */
public class SwerveModuleTelemetry {
  private final SwerveModuleTelemetryConfig m_config;
  private Map<DoubleTelemetryField, DoubleTelemetry<DoubleTelemetryField>> m_doubleTelemetry;
  private Map<StructTelemetryField, StructTelemetry<?, StructTelemetryField>> m_structTelemetry;
  private NetworkTable m_dataNt;
  private NetworkTable m_tuningNt;

  /**
   * Create SwerveModule telemetry for logging in NetworkTables and DataLog.
   *
   * @param config {@link SwerveModuleTelemetryConfig} to apply.
   */
  public SwerveModuleTelemetry(SwerveModuleTelemetryConfig config) {
    m_config = config;
  }

  /**
   * Setup telemetry for the module.
   *
   * @param mechName Telemetry name of the parent {@link yams.mechanisms.swerve.SwerveDrive}.
   * @param module   {@link SwerveModule} to use for telemetry.
   */
  public void setupTelemetry(String mechName, SwerveModule module) {
    m_dataNt = NetworkTableInstance.getDefault()
                                   .getTable("Mechanisms")
                                   .getSubTable(mechName)
                                   .getSubTable("modules")
                                   .getSubTable(module.getName());
    m_tuningNt = NetworkTableInstance.getDefault()
                                     .getTable("Tuning")
                                     .getSubTable(mechName)
                                     .getSubTable("modules")
                                     .getSubTable(module.getName());
    m_doubleTelemetry = m_config.getDoubleFields();
    m_structTelemetry = m_config.getStructFields();
    for (Map.Entry<DoubleTelemetryField, DoubleTelemetry<DoubleTelemetryField>> entry :
        m_doubleTelemetry.entrySet()) {
      var dt = entry.getValue();
      if (!dt.enabled) {
        continue;
      }
      if (m_config.getNT4Enabled()) {
        dt.setupNetworkTables(m_dataNt, m_tuningNt);
      }
      m_config.getDataLogName().ifPresent(dt::setupDataLog);
    }
    for (Map.Entry<StructTelemetryField, StructTelemetry<?, StructTelemetryField>> entry :
        m_structTelemetry.entrySet()) {
      var stt = entry.getValue();
      if (!stt.enabled) {
        continue;
      }
      if (m_config.getNT4Enabled()) {
        stt.setupNetworkTables(m_dataNt, m_tuningNt);
      }
      m_config.getDataLogName().ifPresent(stt::setupDataLog);
    }
    module.getDriveMotorController().setupTelemetry(m_dataNt, m_tuningNt);
    module.getAzimuthMotorController().setupTelemetry(m_dataNt, m_tuningNt);
  }

  /**
   * Publish telemetry to NetworkTables and DataLog.
   *
   * @param module {@link SwerveModule} to publish telemetry from.
   */
  @SuppressWarnings("unchecked")
  public void publish(SwerveModule module) {
    for (Map.Entry<DoubleTelemetryField, DoubleTelemetry<DoubleTelemetryField>> entry :
        m_doubleTelemetry.entrySet()) {
      var dt = entry.getValue();
      if (!dt.enabled) {
        continue;
      }
      switch (dt.getField()) {
        case AbsoluteEncoder -> dt.set(module.getRawAbsoluteEncoderAngle().in(Degrees));
      }
    }
    for (Map.Entry<StructTelemetryField, StructTelemetry<?, StructTelemetryField>> entry :
        m_structTelemetry.entrySet()) {
      var stt = entry.getValue();
      if (!stt.enabled) {
        continue;
      }
      switch (stt.getField()) {
        case State -> ((StructTelemetry<SwerveModuleVelocity, StructTelemetryField>) stt)
            .set(module.getState());
      }
    }
  }

  /**
   * Close and unpublish telemetry.
   */
  public void close() {
    if (m_doubleTelemetry != null) {
      for (Map.Entry<DoubleTelemetryField, DoubleTelemetry<DoubleTelemetryField>> entry :
          m_doubleTelemetry.entrySet()) {
        entry.getValue().close();
      }
    }
    if (m_structTelemetry != null) {
      for (Map.Entry<StructTelemetryField, StructTelemetry<?, StructTelemetryField>> entry :
          m_structTelemetry.entrySet()) {
        entry.getValue().close();
      }
    }
  }

  /**
   * Double telemetry field for {@link SwerveModule}s.
   */
  public enum DoubleTelemetryField {
    /**
     * Absolute encoder angle, without offsets applied, in degrees.
     */
    AbsoluteEncoder("encoder", 0, false, "degrees");

    /**
     * Default value of the double telemetry field.
     */
    private final double defaultVal;
    /**
     * Key that the telemetry is stored at.
     */
    private final String key;
    /**
     * Tunable field?
     */
    private final boolean tunable;
    /**
     * Unit of the telemetry field.
     */
    private final String unit;

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

  /**
   * Struct telemetry field for {@link SwerveModule}s.
   */
  public enum StructTelemetryField {
    /**
     * Measured {@link SwerveModuleVelocity} of the module.
     */
    State("state", SwerveModuleVelocity.struct, new SwerveModuleVelocity(), false);

    /**
     * Key that the telemetry is stored at.
     */
    private final String key;
    /**
     * {@link Struct} serializer for the field's value type.
     */
    private final Struct<?> struct;
    /**
     * Default value of the struct telemetry field.
     */
    private final Object defaultValue;
    /**
     * Tunable field?
     */
    private final boolean tunable;

    /**
     * Create a struct telemetry field.
     *
     * @param fieldName    Field for {@link org.wpilib.networktables.NetworkTable}
     * @param struct       {@link Struct} serializer for the field's value type {@link T}.
     * @param defaultValue Default value in NT.
     * @param tunable      Tunable field.
     * @param <T>          Type of the field's value.
     */
    <T> StructTelemetryField(String fieldName, Struct<T> struct, T defaultValue, boolean tunable) {
      key = fieldName;
      this.struct = struct;
      this.defaultValue = defaultValue;
      this.tunable = tunable;
    }

    /**
     * Create a {@link StructTelemetry} object for non-static usage.
     *
     * @param <T> Type of the field's value, must match the type this field was declared with.
     * @return {@link StructTelemetry}
     */
    @SuppressWarnings("unchecked")
    public <T> StructTelemetry<T, StructTelemetryField> create() {
      return new StructTelemetry<>(key, (T) defaultValue, this, (Struct<T>) struct, tunable);
    }
  }
}
