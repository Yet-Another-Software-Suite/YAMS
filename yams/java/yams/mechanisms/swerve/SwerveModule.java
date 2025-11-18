package yams.mechanisms.swerve;

import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.RobotBase;
import yams.exceptions.SmartMotorControllerConfigurationException;
import yams.mechanisms.config.SwerveModuleConfig;
import yams.motorcontrollers.SmartMotorController;
import yams.telemetry.MechanismTelemetry;

/**
 * Swerve Module
 */
public class SwerveModule
{

  /**
   * Drive motor controller.
   */
  private final SmartMotorController m_dirveMotorController;
  /**
   * Azimuth motor controller.
   */
  private final SmartMotorController m_azimuthMotorController;
  /**
   * Swerve module configuration.
   */
  private final SwerveModuleConfig   m_config;
  /**
   * Mechanism Telemetry
   */
  private final MechanismTelemetry   m_telemetry = new MechanismTelemetry();

  /**
   * Create a SwerveModule.
   *
   * @param config {@link SwerveModuleConfig} for the module.
   */
  public SwerveModule(SwerveModuleConfig config)
  {
    m_config = config;
    m_dirveMotorController = config.getDriveMotor();
    m_azimuthMotorController = config.getAzimuthMotor();
    if (m_config.getTelemetryName().isEmpty())
    {
      throw new IllegalArgumentException("SwerveModuleConfig must have a telemetry name!");
    }
    if (m_config.getLocation().isEmpty())
    {
      throw new IllegalArgumentException("SwerveModuleConfig must have a position!");
    }
    if (m_azimuthMotorController.getConfig().getExternalEncoder().isPresent() &&
        !m_azimuthMotorController.getConfig().getUseExternalFeedback())
    {
      throw new SmartMotorControllerConfigurationException("External encoder cannot be used without external feedback",
                                                           "External encoder could not be used",
                                                           "withUseExternalFeedbackEncoder(true)");
    }
    m_telemetry.setupTelemetry("swerve/" + getName() + "/drive", m_dirveMotorController);
    m_telemetry.setupTelemetry("swerve/" + getName() + "/azimuth", m_azimuthMotorController);
    seedAzimuthEncoder();
  }

  /**
   * Seed the azimuth encoder with the absolute encoder angle.
   */
  public void seedAzimuthEncoder()
  {
    if (RobotBase.isReal() && (m_azimuthMotorController.getConfig().getExternalEncoder().isEmpty() ||
                               !m_azimuthMotorController.getConfig().getUseExternalFeedback()))
    {
      m_azimuthMotorController.setEncoderPosition(m_config.getAbsoluteEncoderAngle());
    }
  }

  /**
   * Get the name of the module.
   *
   * @return Name of the module.
   */
  public String getName()
  {
    return m_config.getTelemetryName().orElse("SwerveModule");
  }

  /**
   * Get the {@link SwerveModuleConfig} for the module.
   *
   * @return {@link SwerveModuleConfig} for the module.
   */
  public SwerveModuleConfig getConfig()
  {
    return m_config;
  }

  /**
   * Set the {@link SwerveModuleState} of the module.
   *
   * @param state State to set.
   */
  public void setSwerveModuleState(SwerveModuleState state)
  {
    state = m_config.getOptimizedState(state);
    m_dirveMotorController.setVelocity(MetersPerSecond.of(state.speedMetersPerSecond));
    m_azimuthMotorController.setPosition(state.angle.getMeasure());
  }

  /**
   * Get the {@link SwerveModuleState} of the module.
   *
   * @return {@link SwerveModuleState} of the module.
   */
  public SwerveModuleState getState()
  {
    return new SwerveModuleState(m_dirveMotorController.getMeasurementVelocity(),
                                 new Rotation2d(m_azimuthMotorController.getMechanismPosition()));
  }

  /**
   * Get the {@link SwerveModulePosition} of the module.
   *
   * @return {@link SwerveModulePosition} of the module.
   */
  public SwerveModulePosition getPosition()
  {
    return new SwerveModulePosition(m_dirveMotorController.getMeasurementPosition(),
                                    new Rotation2d(m_azimuthMotorController.getMechanismPosition()));
  }

  /**
   * Update the telemetry of the module.
   */
  public void updateTelemetry()
  {
    m_dirveMotorController.updateTelemetry();
    m_azimuthMotorController.updateTelemetry();
    m_telemetry.updateLoopTime();
  }

  /**
   * Update the simulation of the module.
   */
  public void simIterate()
  {
    m_dirveMotorController.simIterate();
    m_azimuthMotorController.simIterate();
  }
}
