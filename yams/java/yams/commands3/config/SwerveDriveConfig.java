// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package yams.commands3.config;

import org.wpilib.command3.Mechanism;
import yams.core.mechanisms.swerve.SwerveModule;

/**
 * Command-based extension of {@link yams.core.mechanisms.config.SwerveDriveConfig} that adds the
 * {@link Mechanism} binding.
 *
 * <h2>Usage Example</h2>
 * <pre>{@code
 * // Build each SwerveModule (see SwerveModuleConfig for per-module setup), then:
 * SwerveDriveConfig config = new SwerveDriveConfig(this, fl, fr, bl, br)
 *     .withGyro(gyro.getYaw().asSupplier())
 *     .withMaximumChassisSpeed(MetersPerSecond.of(4.5), DegreesPerSecond.of(360))
 *     .withTranslationController(new PIDController(1.0, 0, 0))
 *     .withRotationController(new PIDController(1.0, 0, 0))
 *     .withStartingPose(new Pose2d());
 * SwerveDrive drive = new SwerveDrive(config);
 * }</pre>
 */
public class SwerveDriveConfig extends yams.core.mechanisms.config.SwerveDriveConfig {
  /** Swerve drive mechanism. */
  private Mechanism mechanism;

  /**
   * Create the {@link SwerveDriveConfig} for the {@link yams.commands3.swerve.SwerveDrive}
   *
   * @param modules       {@link SwerveModule}s for the swerve drive.
   * @param swerveMechanism SwerveDrive mechanism.
   */
  public SwerveDriveConfig(Mechanism swerveMechanism, SwerveModule... modules) {
    super(modules);
    this.mechanism = swerveMechanism;
  }

  /**
   * Create the {@link SwerveDriveConfig} for the {@link yams.commands3.swerve.SwerveDrive}
   *
   * @implNote Must define a Mechanism with {@link #withMechanism(Mechanism)} and modules with
   *           {@link #withModules(SwerveModule...)}
   */
  public SwerveDriveConfig() {
    super();
  }

  private SwerveDriveConfig(SwerveDriveConfig cfg) {
    super(cfg);
  }

  /**
   * Clone the {@link SwerveDriveConfig} without modules, mechanism, telemetry name, gyro supplier,
   * gyro angular velocity supplier, gyro offset, and gyro inversion.
   *
   * @return New {@link SwerveDriveConfig}
   */
  @Override
  public SwerveDriveConfig clone() {
    return new SwerveDriveConfig(this);
  }

  /**
   * Define a {@link Mechanism} for the {@link yams.commands3.swerve.SwerveDrive}
   *
   * @param mechanism {@link Mechanism} for the swerve drive.
   * @return {@link SwerveDriveConfig} for chaining.
   */
  public SwerveDriveConfig withMechanism(Mechanism mechanism) {
    this.mechanism = mechanism;
    return this;
  }

  /**
   * Get the swerve drive mechanism.
   *
   * @return Swerve drive mechanism.
   */
  public Mechanism getMechanism() {
    return mechanism;
  }
}
