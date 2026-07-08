// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package yams.mechanisms.swerve;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.RobotBase;
import yams.exceptions.SmartMotorControllerConfigurationException;
import yams.mechanisms.config.SwerveModuleConfig;
import yams.motorcontrollers.SmartMotorController;
import yams.telemetry.MechanismTelemetry;

import java.util.function.Supplier;

/**
 * Swerve Module
 *
 * <p>
 * {@link SwerveModule} coordinates the drive motor (velocity control) and the azimuth (steer)
 * motor (position control) that make up one corner of a swerve drivetrain.  On construction it
 * reads the absolute encoder and seeds the azimuth relative encoder so the wheel starts at the
 * correct angle.  Each periodic cycle you call {@link #setSwerveModuleState(SwerveModuleState)} to
 * command both motors, and {@link #getState()} / {@link #getPosition()} to read back the current
 * wheel velocity and heading.
 * </p>
 *
 * <h2>Typical usage</h2>
 * <p>
 * In almost every case you should <b>not</b> instantiate {@link SwerveModule} directly.  Instead,
 * pass one {@link yams.mechanisms.config.SwerveModuleConfig} per corner to
 * {@link yams.mechanisms.config.SwerveDriveConfig} and let
 * {@link yams.mechanisms.swerve.SwerveDrive} create and manage the modules internally:
 * </p>
 * <pre>{@code
 * SwerveDriveConfig driveConfig = new SwerveDriveConfig()
 *     .withSwerveModuleConfig(frontLeft, frontRight, backLeft, backRight);
 *
 * SwerveDrive swerveDrive = new SwerveDrive(driveConfig);
 * }</pre>
 *
 * <h2>Direct instantiation (advanced)</h2>
 * <p>
 * If you need direct access to a module — for example when writing unit tests or custom
 * characterisation routines — you can construct one from a fully-configured
 * {@link yams.mechanisms.config.SwerveModuleConfig}:
 * </p>
 * <pre>{@code
 * // Assumes 'frontLeftConfig' has already been built with drive/steer motors,
 * // wheel radius, module location, absolute encoder offset, and telemetry name.
 * SwerveModule frontLeft = new SwerveModule(frontLeftConfig);
 *
 * // Command a specific state (angle + speed)
 * SwerveModuleState desiredState = new SwerveModuleState(1.5, Rotation2d.fromDegrees(45));
 * frontLeft.setSwerveModuleState(desiredState);
 *
 * // Read back current state
 * SwerveModuleState current = frontLeft.getState();
 * }</pre>
 */
public class SwerveModule {
  /**
   * Drive motor controller.
   */
  protected final SmartMotorController m_driveMotorController;
  /**
   * Azimuth motor controller.
   */
  protected final SmartMotorController m_azimuthMotorController;
  /**
   * Swerve module configuration.
   */
  private final SwerveModuleConfig m_config;
  /**
   * Mechanism Telemetry
   */
  private final MechanismTelemetry m_telemetry = new MechanismTelemetry();
  /**
   * Azimuth absolute encoder field.
   */
  private final DoublePublisher m_azimuthAbsoluteEncoderTelemetry;
  /**
   * Absolute encoder angle without any offsets applied.
   */
  private final Supplier<Angle> m_azimuthEncoderWithoutOffsets;

  /**
   * Create a SwerveModule.
   *
   * @param config {@link SwerveModuleConfig} for the module.
   */
  public SwerveModule(SwerveModuleConfig config) {
    m_config = config;
    m_driveMotorController = config.getDriveMotor();
    m_azimuthMotorController = config.getAzimuthMotor();
    if (m_config.getTelemetryName().isEmpty()) {
      throw new IllegalArgumentException("SwerveModuleConfig must have a telemetry name!");
    }
    if (m_config.getLocation().isEmpty()) {
      throw new IllegalArgumentException("SwerveModuleConfig must have a position!");
    }
    if (m_azimuthMotorController.getConfig().getExternalEncoder().isPresent() && !m_azimuthMotorController.getConfig().getUseExternalFeedback()) {
      throw new SmartMotorControllerConfigurationException("External encoder cannot be used without external feedback", "External encoder could not be used", "withUseExternalFeedbackEncoder(true)");
    }
    m_telemetry.setupTelemetry("swerve/" + getName() + "/drive", m_driveMotorController);
    m_telemetry.setupTelemetry("swerve/" + getName() + "/azimuth", m_azimuthMotorController);
    var encoderTopic = m_telemetry.getDataTable().getDoubleTopic("encoder");
    encoderTopic.setProperties("{\"units\": \"degrees\"}");
    m_azimuthAbsoluteEncoderTelemetry = encoderTopic.publish();
    seedAzimuthEncoder();
    m_azimuthEncoderWithoutOffsets = config.getRawAbsoluteEncoderAngle();
  }

  /**
   * Seed the azimuth encoder with the absolute encoder angle.
   */
  public void seedAzimuthEncoder() {
    if (RobotBase.isReal() && (m_azimuthMotorController.getConfig().getExternalEncoder().isEmpty() || !m_azimuthMotorController.getConfig().getUseExternalFeedback())) {
      m_azimuthMotorController.setEncoderPosition(m_config.getAbsoluteEncoderAngle());
    }
  }

  /**
   * Get the name of the module.
   *
   * @return Name of the module.
   */
  public String getName() {
    return m_config.getTelemetryName().orElseThrow();
  }

  /**
   * Get the {@link SwerveModuleConfig} for the module.
   *
   * @return {@link SwerveModuleConfig} for the module.
   */
  public SwerveModuleConfig getConfig() {
    return m_config;
  }

  /**
   * Set the {@link SwerveModuleState} of the module.
   *
   * @param state State to set.
   * @return The optimized {@link SwerveModuleState}.
   */
  public SwerveModuleState setSwerveModuleState(SwerveModuleState state) {
    state = m_config.getOptimizedState(state);
    m_driveMotorController.setVelocity(MetersPerSecond.of(state.speedMetersPerSecond));
    m_azimuthMotorController.setPosition(state.angle.getMeasure());
    return state;
  }

  /**
   * Get the {@link SwerveModuleState} of the module.
   *
   * @return {@link SwerveModuleState} of the module.
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(m_driveMotorController.getMeasurementVelocity(), new Rotation2d(m_azimuthMotorController.getMechanismPosition()));
  }

  /**
   * Get the {@link SwerveModulePosition} of the module.
   *
   * @return {@link SwerveModulePosition} of the module.
   */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(m_driveMotorController.getMeasurementPosition(), new Rotation2d(m_azimuthMotorController.getMechanismPosition()));
  }

  /**
   * Update the telemetry of the module.
   */
  public void updateTelemetry() {
    m_driveMotorController.updateTelemetry();
    m_azimuthMotorController.updateTelemetry();
    m_azimuthAbsoluteEncoderTelemetry.set(m_azimuthEncoderWithoutOffsets.get().in(Degrees));
    m_telemetry.updateLoopTime();
  }

  /**
   * Update the simulation of the module.
   */
  public void simIterate() {
    m_driveMotorController.simIterate();
    m_azimuthMotorController.simIterate();
  }

  /**
   * Get the azimuth {@link SmartMotorController}.
   *
   * @return Azimuth {@link SmartMotorController}.
   */
  public SmartMotorController getAzimuthMotorController() {
    return m_azimuthMotorController;
  }

  /**
   * Get the drive {@link SmartMotorController}.
   *
   * @return Drive {@link SmartMotorController}.
   */
  public SmartMotorController getDriveMotorController() {
    return m_driveMotorController;
  }
}
