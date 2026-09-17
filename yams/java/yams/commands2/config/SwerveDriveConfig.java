// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package yams.commands2.config;

import org.wpilib.command2.Subsystem;
import yams.core.mechanisms.swerve.SwerveModule;

/**
 * Command-based extension of {@link yams.core.mechanisms.config.SwerveDriveConfig} that adds the
 * {@link Subsystem} binding that core must not depend on.
 */
public class SwerveDriveConfig extends yams.core.mechanisms.config.SwerveDriveConfig {
  /** Swerve drive subsystem. */
  private Subsystem subsystem;

  /**
   * Create the {@link SwerveDriveConfig} for the {@link yams.commands2.swerve.SwerveDrive}
   *
   * @param modules         {@link SwerveModule}s for the swerve drive.
   * @param swerveSubsystem SwerveDrive subsystem.
   */
  public SwerveDriveConfig(Subsystem swerveSubsystem, SwerveModule... modules) {
    super(modules);
    this.subsystem = swerveSubsystem;
  }

  /**
   * Create the {@link SwerveDriveConfig} for the {@link yams.commands2.swerve.SwerveDrive}
   *
   * @implNote Must define a Subsystem with {@link #withSubsystem(Subsystem)} and modules with
   *           {@link #withModules(SwerveModule...)}
   */
  public SwerveDriveConfig() {
    super();
  }

  private SwerveDriveConfig(SwerveDriveConfig cfg) {
    super(cfg);
  }

  /**
   * Clone the {@link SwerveDriveConfig} without modules, subsystem, telemetry name, gyro supplier,
   * gyro angular velocity supplier, gyro offset, and gyro inversion.
   *
   * @return New {@link SwerveDriveConfig}
   */
  @Override
  public SwerveDriveConfig clone() {
    return new SwerveDriveConfig(this);
  }

  /**
   * Define a {@link Subsystem} for the {@link yams.commands2.swerve.SwerveDrive}
   *
   * @param subsystem {@link Subsystem} for the swerve drive.
   * @return {@link SwerveDriveConfig} for chaining.
   */
  public SwerveDriveConfig withSubsystem(Subsystem subsystem) {
    this.subsystem = subsystem;
    return this;
  }

  /**
   * Get the swerve drive subsystem.
   *
   * @return Swerve drive subsystem.
   */
  public Subsystem getSubsystem() {
    return subsystem;
  }
}
