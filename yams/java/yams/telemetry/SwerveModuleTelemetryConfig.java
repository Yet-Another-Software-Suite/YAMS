// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package yams.telemetry;

import java.util.Arrays;
import java.util.Map;
import java.util.Optional;
import java.util.stream.Collectors;
import yams.mechanisms.swerve.SwerveModule;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.telemetry.SwerveModuleTelemetry.DoubleTelemetryField;
import yams.telemetry.SwerveModuleTelemetry.StructTelemetryField;

/**
 * Swerve module telemetry configuration.
 *
 * <p>Use this builder to select exactly which fields are published to NetworkTables and/or
 * DataLog. Every field is disabled by default; call the individual {@code with*()} methods to
 * opt in, or use {@link #withTelemetryVerbosity} to enable a predefined set.
 *
 * <h2>Example</h2>
 * <pre>{@code
 * SwerveModuleTelemetryConfig telemetryCfg =
 *     new SwerveModuleTelemetryConfig()
 *         // choose a verbosity preset (LOW / MID / HIGH), or enable fields individually below
 *         .withTelemetryVerbosity(TelemetryVerbosity.HIGH)
 *         .withAbsoluteEncoder()
 *         .withState()
 *         // disable NT4 output (e.g. during competition) and log to DataLog instead
 *         .withoutNetworkTables()
 *         .withDataLogName("swerve/modules/frontleft");
 * }</pre>
 */
public class SwerveModuleTelemetryConfig
{
  /**
   * DataLog entry name
   */
  private Optional<String> dataLogName  = Optional.empty();
  /**
   * Enable telemetry over network tables.
   */
  private boolean          NT4Telemetry = true;
  /**
   * {@link StructTelemetryField}s to enable or disable.
   */
  private final Map<StructTelemetryField, StructTelemetry<?, StructTelemetryField>> structFields =
      Arrays.stream(StructTelemetryField.values())
            .collect(Collectors.toMap(e -> e, StructTelemetryField::create));
  /**
   * {@link DoubleTelemetryField}s to enable or disable.
   */
  private final Map<DoubleTelemetryField, DoubleTelemetry<DoubleTelemetryField>> doubleFields =
      Arrays.stream(DoubleTelemetryField.values())
            .collect(Collectors.toMap(e -> e, DoubleTelemetryField::create));

  /**
   * Default constructor.
   */
  public SwerveModuleTelemetryConfig()
  {
  }

  /**
   * Constructor with verbosity preset.
   * @param verbosity {@link TelemetryVerbosity} to use by default.
   */
  public SwerveModuleTelemetryConfig(TelemetryVerbosity verbosity)
  {
    withTelemetryVerbosity(verbosity);
  }


  /**
   * Set up a DataLog entry for this {@link SwerveModule}
   *
   * @param dataLogName DataLog entry name
   * @return {@link SwerveModuleTelemetryConfig} for chaining.
   */
  public SwerveModuleTelemetryConfig withDataLogName(String dataLogName)
  {
    this.dataLogName = Optional.ofNullable(dataLogName);
    return this;
  }

  /**
   * Enable or disable NT4 Telemetry. This will not create NT4 entries and is generally only advisable during
   * competition matches.
   *
   * @param NT4Telemetry NT4 Boolean
   * @return {@link SwerveModuleTelemetryConfig} for chaining.
   */
  public SwerveModuleTelemetryConfig withNetworkTables(boolean NT4Telemetry)
  {
    this.NT4Telemetry = NT4Telemetry;
    return this;
  }

  /**
   * Disable NetworkTable output.
   *
   * @return {@link SwerveModuleTelemetryConfig} for chaining.
   */
  public SwerveModuleTelemetryConfig withoutNetworkTables()
  {
    this.NT4Telemetry = false;
    return this;
  }

  /**
   * Setup with {@link TelemetryVerbosity}
   *
   * @param verbosity {@link TelemetryVerbosity} to use.
   * @return {@link SwerveModuleTelemetryConfig} for chaining.
   */
  public SwerveModuleTelemetryConfig withTelemetryVerbosity(TelemetryVerbosity verbosity)
  {
    switch (verbosity)
    {
      case HIGH:
        structFields.get(StructTelemetryField.State).enable();
      case MID:
      case LOW:
        doubleFields.get(DoubleTelemetryField.AbsoluteEncoder).enable();
    }
    return this;
  }

  /**
   * Get the entry name for the {@link SwerveModule} in DataLog.
   *
   * @return DataLog entry name.
   */
  public Optional<String> getDataLogName()
  {
    return dataLogName;
  }

  /**
   * Log telemetry to NT4?
   *
   * @return should Telemetry be sent to NT4.
   */
  public boolean getNT4Enabled()
  {
    return NT4Telemetry;
  }

  /**
   * Get the configured double fields.
   *
   * @return Configured {@link DoubleTelemetry} for each {@link DoubleTelemetryField}
   */
  public Map<DoubleTelemetryField, DoubleTelemetry<DoubleTelemetryField>> getDoubleFields()
  {
    return doubleFields;
  }

  /**
   * Get the configured struct fields.
   *
   * @return Configured {@link StructTelemetry} for each {@link StructTelemetryField}
   */
  public Map<StructTelemetryField, StructTelemetry<?, StructTelemetryField>> getStructFields()
  {
    return structFields;
  }

  /**
   * Enables the absolute encoder angle logging.
   *
   * @return {@link SwerveModuleTelemetryConfig} for chaining.
   */
  public SwerveModuleTelemetryConfig withAbsoluteEncoder()
  {
    doubleFields.get(DoubleTelemetryField.AbsoluteEncoder).enable();
    return this;
  }

  /**
   * Enables the {@link edu.wpi.first.math.kinematics.SwerveModuleState} logging.
   *
   * @return {@link SwerveModuleTelemetryConfig} for chaining.
   */
  public SwerveModuleTelemetryConfig withState()
  {
    structFields.get(StructTelemetryField.State).enable();
    return this;
  }

  /**
   * Escape hatch for unimplemented fields which should be enabled or disabled.
   *
   * @param field Field to configure
   * @param value Enable on true, Disable on false.
   * @return {@link SwerveModuleTelemetryConfig} for chaining
   */
  public SwerveModuleTelemetryConfig withCustom(DoubleTelemetryField field, boolean value)
  {
    if (value)
    {doubleFields.get(field).enable();} else
    {doubleFields.get(field).disable();}
    return this;
  }

  /**
   * Escape hatch for unimplemented fields which should be enabled or disabled.
   *
   * @param field Field to configure
   * @param value Enable on true, Disable on false.
   * @return {@link SwerveModuleTelemetryConfig} for chaining
   */
  public SwerveModuleTelemetryConfig withCustom(DoubleTelemetryField[] field, boolean value)
  {
    for (DoubleTelemetryField field1 : field)
    {
      withCustom(field1, value);
    }
    return this;
  }

  /**
   * Escape hatch for unimplemented fields which should be enabled or disabled.
   *
   * @param field Field to configure
   * @param value Enable on true, Disable on false.
   * @return {@link SwerveModuleTelemetryConfig} for chaining
   */
  public SwerveModuleTelemetryConfig withCustom(StructTelemetryField field, boolean value)
  {
    if (value)
    {structFields.get(field).enable();} else
    {structFields.get(field).disable();}
    return this;
  }

  /**
   * Escape hatch for unimplemented fields which should be enabled or disabled.
   *
   * @param field Field to configure
   * @param value Enable on true, Disable on false.
   * @return {@link SwerveModuleTelemetryConfig} for chaining
   */
  public SwerveModuleTelemetryConfig withCustom(StructTelemetryField[] field, boolean value)
  {
    for (StructTelemetryField field1 : field)
    {
      withCustom(field1, value);
    }
    return this;
  }
}
