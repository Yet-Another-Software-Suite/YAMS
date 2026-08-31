// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package yams.telemetry;

import java.util.Arrays;
import java.util.Map;
import java.util.Optional;
import java.util.stream.Collectors;
import yams.mechanisms.swerve.SwerveDrive;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.telemetry.SwerveDriveTelemetry.DoubleTelemetryField;
import yams.telemetry.SwerveDriveTelemetry.StructArrayTelemetryField;
import yams.telemetry.SwerveDriveTelemetry.StructTelemetryField;

/**
 * Swerve drive telemetry configuration.
 *
 * <p>Use this builder to select exactly which fields are published to NetworkTables and/or
 * DataLog. Every field is disabled by default; call the individual {@code with*()} methods to
 * opt in, or use {@link #withTelemetryVerbosity} to enable a predefined set.
 *
 * <h2>Example</h2>
 * <pre>{@code
 * SwerveDriveTelemetryConfig telemetryCfg =
 *     new SwerveDriveTelemetryConfig()
 *         // choose a verbosity preset (LOW / MID / HIGH), or enable fields individually below
 *         .withTelemetryVerbosity(TelemetryVerbosity.HIGH)
 *         // optionally enable individual fields on top of the preset
 *         .withPose()
 *         .withGyro()
 *         .withCurrentRobotRelativeChassisSpeeds()
 *         .withFieldRelativeChassisSpeeds()
 *         .withDesiredRobotRelativeChassisSpeeds()
 *         .withDesiredModuleStates()
 *         .withCurrentModuleStates()
 *         // disable NT4 output (e.g. during competition) and log to DataLog instead
 *         .withoutNetworkTables()
 *         .withDataLogName("swerve");
 * }</pre>
 */
public class SwerveDriveTelemetryConfig
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
   * {@link StructArrayTelemetryField}s to enable or disable.
   */
  private final Map<StructArrayTelemetryField, StructArrayTelemetry<?, StructArrayTelemetryField>> structArrayFields =
      Arrays.stream(StructArrayTelemetryField.values())
            .collect(Collectors.toMap(e -> e, StructArrayTelemetryField::create));
  /**
   * {@link DoubleTelemetryField}s to enable or disable.
   */
  private final Map<DoubleTelemetryField, DoubleTelemetry<DoubleTelemetryField>> doubleFields =
      Arrays.stream(DoubleTelemetryField.values())
            .collect(Collectors.toMap(e -> e, DoubleTelemetryField::create));

  /**
   * Default constructor
   */
  public SwerveDriveTelemetryConfig()
  {}

  /**
   * Constructor with verbosity preset.
   * @param verbosity {@link TelemetryVerbosity} to use.
   */
  public SwerveDriveTelemetryConfig(TelemetryVerbosity verbosity)
  {
    withTelemetryVerbosity(verbosity);
  }

  /**
   * Set up a DataLog entry for this {@link SwerveDrive}
   *
   * @param dataLogName DataLog entry name
   * @return {@link SwerveDriveTelemetryConfig} for chaining.
   */
  public SwerveDriveTelemetryConfig withDataLogName(String dataLogName)
  {
    this.dataLogName = Optional.ofNullable(dataLogName);
    return this;
  }

  /**
   * Enable or disable NT4 Telemetry. This will not create NT4 entries and is generally only advisable during
   * competition matches.
   *
   * @param NT4Telemetry NT4 Boolean
   * @return {@link SwerveDriveTelemetryConfig} for chaining.
   */
  public SwerveDriveTelemetryConfig withNetworkTables(boolean NT4Telemetry)
  {
    this.NT4Telemetry = NT4Telemetry;
    return this;
  }

  /**
   * Disable NetworkTable output.
   *
   * @return {@link SwerveDriveTelemetryConfig} for chaining.
   */
  public SwerveDriveTelemetryConfig withoutNetworkTables()
  {
    this.NT4Telemetry = false;
    return this;
  }

  /**
   * Setup with {@link TelemetryVerbosity}
   *
   * @param verbosity {@link TelemetryVerbosity} to use.
   * @return {@link SwerveDriveTelemetryConfig} for chaining.
   */
  public SwerveDriveTelemetryConfig withTelemetryVerbosity(TelemetryVerbosity verbosity)
  {
    switch (verbosity)
    {
      case HIGH:
        structFields.get(StructTelemetryField.DesiredRobotRelativeChassisSpeeds).enable();
        structArrayFields.get(StructArrayTelemetryField.DesiredModuleStates).enable();
        doubleFields.get(DoubleTelemetryField.TranslationP).enable();
        doubleFields.get(DoubleTelemetryField.TranslationI).enable();
        doubleFields.get(DoubleTelemetryField.TranslationD).enable();
        doubleFields.get(DoubleTelemetryField.RotationP).enable();
        doubleFields.get(DoubleTelemetryField.RotationI).enable();
        doubleFields.get(DoubleTelemetryField.RotationD).enable();
      case MID:
        structFields.get(StructTelemetryField.CurrentRobotRelativeChassisSpeeds).enable();
        structFields.get(StructTelemetryField.FieldRelativeChassisSpeeds).enable();
        structArrayFields.get(StructArrayTelemetryField.CurrentModuleStates).enable();
      case LOW:
        structFields.get(StructTelemetryField.Pose).enable();
        doubleFields.get(DoubleTelemetryField.Gyro).enable();
    }
    return this;
  }

  /**
   * Get the entry name for the {@link SwerveDrive} in DataLog.
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
   * Get the configured struct fields.
   *
   * @return Configured {@link StructTelemetry} for each {@link StructTelemetryField}
   */
  public Map<StructTelemetryField, StructTelemetry<?, StructTelemetryField>> getStructFields()
  {
    return structFields;
  }

  /**
   * Get the configured struct array fields.
   *
   * @return Configured {@link StructArrayTelemetry} for each {@link StructArrayTelemetryField}
   */
  public Map<StructArrayTelemetryField, StructArrayTelemetry<?, StructArrayTelemetryField>> getStructArrayFields()
  {
    return structArrayFields;
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
   * Enables the pose logging.
   *
   * @return {@link SwerveDriveTelemetryConfig} for chaining.
   */
  public SwerveDriveTelemetryConfig withPose()
  {
    structFields.get(StructTelemetryField.Pose).enable();
    return this;
  }

  /**
   * Enables the gyro angle logging.
   *
   * @return {@link SwerveDriveTelemetryConfig} for chaining.
   */
  public SwerveDriveTelemetryConfig withGyro()
  {
    doubleFields.get(DoubleTelemetryField.Gyro).enable();
    return this;
  }

  /**
   * Enables the last-commanded desired robot relative chassis speeds logging.
   *
   * @return {@link SwerveDriveTelemetryConfig} for chaining.
   */
  public SwerveDriveTelemetryConfig withDesiredRobotRelativeChassisSpeeds()
  {
    structFields.get(StructTelemetryField.DesiredRobotRelativeChassisSpeeds).enable();
    return this;
  }

  /**
   * Enables the measured robot relative chassis speeds logging.
   *
   * @return {@link SwerveDriveTelemetryConfig} for chaining.
   */
  public SwerveDriveTelemetryConfig withCurrentRobotRelativeChassisSpeeds()
  {
    structFields.get(StructTelemetryField.CurrentRobotRelativeChassisSpeeds).enable();
    return this;
  }

  /**
   * Enables the measured field relative chassis speeds logging.
   *
   * @return {@link SwerveDriveTelemetryConfig} for chaining.
   */
  public SwerveDriveTelemetryConfig withFieldRelativeChassisSpeeds()
  {
    structFields.get(StructTelemetryField.FieldRelativeChassisSpeeds).enable();
    return this;
  }

  /**
   * Enables the last-commanded desired module states logging.
   *
   * @return {@link SwerveDriveTelemetryConfig} for chaining.
   */
  public SwerveDriveTelemetryConfig withDesiredModuleStates()
  {
    structArrayFields.get(StructArrayTelemetryField.DesiredModuleStates).enable();
    return this;
  }

  /**
   * Enables the measured module states logging.
   *
   * @return {@link SwerveDriveTelemetryConfig} for chaining.
   */
  public SwerveDriveTelemetryConfig withCurrentModuleStates()
  {
    structArrayFields.get(StructArrayTelemetryField.CurrentModuleStates).enable();
    return this;
  }

  /**
   * Escape hatch for unimplemented fields which should be enabled or disabled.
   *
   * @param field Field to configure
   * @param value Enable on true, Disable on false.
   * @return {@link SwerveDriveTelemetryConfig} for chaining
   */
  public SwerveDriveTelemetryConfig withCustom(DoubleTelemetryField field, boolean value)
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
   * @return {@link SwerveDriveTelemetryConfig} for chaining
   */
  public SwerveDriveTelemetryConfig withCustom(StructTelemetryField field, boolean value)
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
   * @return {@link SwerveDriveTelemetryConfig} for chaining
   */
  public SwerveDriveTelemetryConfig withCustom(DoubleTelemetryField[] field, boolean value)
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
   * @return {@link SwerveDriveTelemetryConfig} for chaining
   */
  public SwerveDriveTelemetryConfig withCustom(StructTelemetryField[] field, boolean value)
  {
    for (StructTelemetryField field1 : field)
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
   * @return {@link SwerveDriveTelemetryConfig} for chaining
   */
  public SwerveDriveTelemetryConfig withCustom(StructArrayTelemetryField field, boolean value)
  {
    if (value)
    {structArrayFields.get(field).enable();} else
    {structArrayFields.get(field).disable();}
    return this;
  }

  /**
   * Escape hatch for unimplemented fields which should be enabled or disabled.
   *
   * @param field Field to configure
   * @param value Enable on true, Disable on false.
   * @return {@link SwerveDriveTelemetryConfig} for chaining
   */
  public SwerveDriveTelemetryConfig withCustom(StructArrayTelemetryField[] field, boolean value)
  {
    for (StructArrayTelemetryField field1 : field)
    {
      withCustom(field1, value);
    }
    return this;
  }
}
