package yams.mechanisms.swerve;

import static edu.wpi.first.hal.FRCNetComm.tInstances.kRobotDriveSwerve_YAGSL;
import static edu.wpi.first.hal.FRCNetComm.tResourceType.kResourceType_RobotDrive;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import java.util.Arrays;
import java.util.function.Supplier;
import yams.mechanisms.config.SwerveDriveConfig;
import yams.telemetry.MechanismTelemetry;

public class SwerveDrive
{

  /**
   * The modules of the drive.
   */
  private final SwerveModule[]                          m_modules;
  /**
   * The config for the drive.
   */
  private       SwerveDriveConfig                       m_config;
  /**
   * The pose estimator for the drive.
   */
  private final SwerveDrivePoseEstimator                m_poseEstimator;
  /**
   * The kinematics for the drive.
   */
  private final SwerveDriveKinematics                   m_kinematics;
  /**
   * Mechanism telemetry.
   */
  private       MechanismTelemetry                      m_telemetry    = new MechanismTelemetry();
  /**
   * Desired swerve module states.
   */
  private final StructArrayPublisher<SwerveModuleState> m_desiredModuleStatesPublisher;
  /**
   * Current swerve module states.
   */
  private final StructArrayPublisher<SwerveModuleState> m_currentModuleStatesPublisher;
  /**
   * Desired robot relative chassis speeds.
   */
  private final StructPublisher<ChassisSpeeds>          m_desiredRobotRelativeChassisSpeedsPublisher;
  /**
   * Current robot relative chassis speeds.
   */
  private final StructPublisher<ChassisSpeeds>          m_currentRobotRelativeChassisSpeedsPublisher;
  /**
   * Field relative chassis speeds.
   */
  private final StructPublisher<ChassisSpeeds>          m_fieldRelativeChassisSpeedsPublisher;
  /**
   * Pose of the robot.
   */
  private final StructPublisher<Pose2d>                 m_posePublisher;
  /**
   * Gyro angle.
   */
  private final DoublePublisher                         m_gyroPublisher;
  /**
   * Simulated Gyro Angle. Used for simulation purposes only. Not used in real robot code.
   */
  private       Angle                                   m_simGyroAngle = Rotations.of(0);
  /**
   * Timer for simulation purposes only. Not used in real robot code.
   */
  private final Timer                                   m_simTimer     = new Timer();

  /**
   * Cube the {@link Translation2d} magnitude given in Polar coordinates.
   *
   * @param translation {@link Translation2d} to manipulate.
   * @return Cubed magnitude from {@link Translation2d}.
   */
  public static Translation2d cubeTranslation(Translation2d translation)
  {
    if (Math.hypot(translation.getX(), translation.getY()) <= 1.0E-6)
    {
      return translation;
    }
    return new Translation2d(Math.pow(translation.getNorm(), 3), translation.getAngle());
  }

  /**
   * Scale the {@link Translation2d} Polar coordinate magnitude.
   *
   * @param translation {@link Translation2d} to use.
   * @param scalar      Multiplier for the Polar coordinate magnitude to use.
   * @return {@link Translation2d} scaled by given magnitude scalar.
   */
  public static Translation2d scaleTranslation(Translation2d translation, double scalar)
  {
    if (Math.hypot(translation.getX(), translation.getY()) <= 1.0E-6)
    {
      return translation;
    }
    return new Translation2d(translation.getNorm() * scalar, translation.getAngle());
  }

  /**
   * Create a SwerveDrive.
   *
   * @param config {@link SwerveDriveConfig} for the drive.
   */
  public SwerveDrive(SwerveDriveConfig config)
  {
    m_config = config;
    m_modules = config.getModules();
    m_kinematics = new SwerveDriveKinematics(Arrays.stream(m_modules)
                                                   .map(module -> module.getConfig().getLocation().orElseThrow())
                                                   .toArray(Translation2d[]::new));
    m_poseEstimator = new SwerveDrivePoseEstimator(m_kinematics,
                                                   new Rotation2d(getGyroAngle()),
                                                   getModulePositions(),
                                                   m_config.getInitialPose());
    m_telemetry.setupTelemetry(getName());
    var desiredModuleStatesTopic = m_telemetry.getDataTable()
                                              .getStructArrayTopic("states/desired", SwerveModuleState.struct);
    var currentModuleStatesTopic = m_telemetry.getDataTable()
                                              .getStructArrayTopic("states/current", SwerveModuleState.struct);
    var poseTopic = m_telemetry.getDataTable().getStructTopic("pose", Pose2d.struct);
    var gyroTopic = m_telemetry.getDataTable().getDoubleTopic("gyro");
    gyroTopic.setProperties("{\"unit\":\"degrees\"}");
    var desiredRobotRelativeChassisSpeedsTopic = m_telemetry.getDataTable()
                                                            .getStructTopic("chassis/desired", ChassisSpeeds.struct);
    var fieldRelativeChassisSpeedsTopic = m_telemetry.getDataTable()
                                                     .getStructTopic("chassis/field", ChassisSpeeds.struct);
    var currentRobotRelativeChassisSpeedsTopic = m_telemetry.getDataTable()
                                                            .getStructTopic("chassis/current", ChassisSpeeds.struct);
    m_gyroPublisher = gyroTopic.publish();
    m_currentRobotRelativeChassisSpeedsPublisher = currentRobotRelativeChassisSpeedsTopic.publish();
    m_fieldRelativeChassisSpeedsPublisher = fieldRelativeChassisSpeedsTopic.publish();
    m_desiredRobotRelativeChassisSpeedsPublisher = desiredRobotRelativeChassisSpeedsTopic.publish();
    m_posePublisher = poseTopic.publish();
    m_desiredModuleStatesPublisher = desiredModuleStatesTopic.publish();
    m_currentModuleStatesPublisher = currentModuleStatesTopic.publish();

    // Report as YAGSL bc this will become apart of YAGSL in 2027...
    HAL.report(kResourceType_RobotDrive, kRobotDriveSwerve_YAGSL);
  }


  /**
   * Create a {@link RunCommand} to drive the swerve drive with robot relative chassis speeds.
   *
   * @param robotRelativeChassisSpeeds {@link Supplier<ChassisSpeeds>} for the robot relative chassis speeds. Could also
   *                                   use {@link yams.mechanisms.swerve.utility.SwerveInputStream}
   * @return {@link RunCommand} to drive the swerve drive.
   */
  public Command drive(Supplier<ChassisSpeeds> robotRelativeChassisSpeeds)
  {
    return Commands.run(() -> setRobotRelativeChassisSpeeds(robotRelativeChassisSpeeds.get()),
                        m_config.getSubsystem()).withName("Drive");
  }

  /**
   * Get the Gyro Angle.
   */
  public Angle getGyroAngle()
  {
    return RobotBase.isReal() ? m_config.getGyroAngle() : m_simGyroAngle;
  }

  /**
   * Point all modules toward the robot center, thus making the robot very difficult to move. Forcing the robot to keep
   * the current pose.
   */
  public void lockPose()
  {
    // Sets states
    for (SwerveModule swerveModule : m_modules)
    {
      SwerveModuleState desiredState =
          new SwerveModuleState(0, swerveModule.getConfig().getLocation().orElseThrow().getAngle());
      swerveModule.setSwerveModuleState(desiredState);
    }

    // Update kinematics because we are not using setModuleStates
    m_desiredRobotRelativeChassisSpeedsPublisher.accept(new ChassisSpeeds());
    m_desiredModuleStatesPublisher.accept(m_kinematics.toSwerveModuleStates(new ChassisSpeeds()));
  }

  /**
   * Set robot relative chassis speeds.
   *
   * @param robotRelativeChassisSpeeds Robot relative chassis speeds.
   */
  public void setRobotRelativeChassisSpeeds(ChassisSpeeds robotRelativeChassisSpeeds)
  {
    robotRelativeChassisSpeeds = m_config.optimizeRobotRelativeChassisSpeeds(robotRelativeChassisSpeeds);
    var states = m_config.getCenterOfRotation().isPresent() ?
                 m_kinematics.toSwerveModuleStates(robotRelativeChassisSpeeds, m_config.getCenterOfRotation().get()) :
                 m_kinematics.toSwerveModuleStates(robotRelativeChassisSpeeds);
    for (int i = 0; i < states.length; i++)
    {
      m_modules[i].setSwerveModuleState(states[i]);
    }
    m_desiredModuleStatesPublisher.accept(states);
    m_desiredRobotRelativeChassisSpeedsPublisher.accept(robotRelativeChassisSpeeds);
  }

  /**
   * Set field relative chassis speeds.
   *
   * @param fieldRelativeChassisSpeeds Field relative chassis speeds.
   */
  public void setFieldRelativeChassisSpeeds(ChassisSpeeds fieldRelativeChassisSpeeds)
  {
    setRobotRelativeChassisSpeeds(ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeChassisSpeeds,
                                                                        new Rotation2d(getGyroAngle())));
  }

  /**
   * Gets the measured pose (position and rotation) of the robot, as reported by odometry.
   *
   * @return The robot's pose
   */
  public Pose2d getPose()
  {
    return m_poseEstimator.getEstimatedPosition();
  }

  /**
   * Resets the gyro angle to zero and resets odometry to the same position, but facing toward 0 (red alliance
   * station).
   */
  public void zeroGyro()
  {
    m_config.withGyroOffset(getGyroAngle().plus(m_config.getGyroOffset()));
    resetOdometry(new Pose2d(getPose().getTranslation(), new Rotation2d()));
  }

  /**
   * Get the name of the drive.
   *
   * @return Name of the drive.
   */
  public String getName()
  {
    return "swerve";
  }

  /**
   * Resets odometry to the given pose. Gyro angle and module positions do not need to be reset when calling this
   * method. However, if either gyro angle or module position is reset, this must be called in order for odometry to
   * keep working.
   *
   * @param pose The pose to set the odometry to. Field relative, blue-origin where 0deg is facing towards RED
   *             alliance.
   */
  public void resetOdometry(Pose2d pose)
  {
    m_poseEstimator.resetPosition(new Rotation2d(getGyroAngle()), getModulePositions(), pose);
    ChassisSpeeds robotRelativeSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(new ChassisSpeeds(0, 0, 0),
                                                                              new Rotation2d(getGyroAngle()));
    m_desiredModuleStatesPublisher.accept(m_kinematics.toSwerveModuleStates(robotRelativeSpeeds));
  }

  /**
   * Resets the azimuth PID controller.
   */
  public void resetAzimuthPID()
  {
    m_config.getRotationPID().reset();
  }

  /**
   * Resets the translation PID controller.
   */
  public void resetTranslationPID()
  {
    m_config.getTranslationPID().reset();
  }

  /**
   * Add a vision measurement to the {@link SwerveDrivePoseEstimator} and update the gyro reading with the given
   * timestamp of the vision measurement.
   *
   * @param robotPose                Robot {@link Pose2d} as measured by vision.
   * @param timestamp                Timestamp the measurement was taken as time since startup, should be taken from
   *                                 {@link Timer#getFPGATimestamp()} or similar sources.
   * @param visionMeasurementStdDevs Vision measurement standard deviation that will be sent to the
   *                                 {@link SwerveDrivePoseEstimator}.The standard deviation of the vision measurement,
   *                                 for best accuracy calculate the standard deviation at 2 or more points and fit a
   *                                 line to it with the calculated optimal standard deviation. (Units should be meters
   *                                 per pixel). By optimizing this you can get * vision accurate to inches instead of
   *                                 feet.
   */
  public void addVisionMeasurement(Pose2d robotPose, double timestamp,
                                   Matrix<N3, N1> visionMeasurementStdDevs)
  {
    m_poseEstimator.addVisionMeasurement(robotPose, timestamp, visionMeasurementStdDevs);
  }

  /**
   * Sets the pose estimator's trust of global measurements. This might be used to change trust in vision measurements
   * after the autonomous period, or to change trust as distance to a vision target increases.
   *
   * @param visionMeasurementStdDevs Standard deviations of the vision measurements. Increase these numbers to trust
   *                                 global measurements from vision less. This matrix is in the form [x, y, theta],
   *                                 with units in meters and radians.
   */
  public void setVisionMeasurementStdDevs(Matrix<N3, N1> visionMeasurementStdDevs)
  {
    m_poseEstimator.setVisionMeasurementStdDevs(visionMeasurementStdDevs);
  }

  /**
   * Add a vision measurement to the {@link SwerveDrivePoseEstimator} and update the gyro reading with the given
   * timestamp of the vision measurement.
   *
   * @param robotPose Robot {@link Pose2d} as measured by vision.
   * @param timestamp Timestamp the measurement was taken as time since startup, should be taken from
   *                  {@link Timer#getFPGATimestamp()} or similar sources.
   */
  public void addVisionMeasurement(Pose2d robotPose, double timestamp)
  {
    m_poseEstimator.addVisionMeasurement(robotPose, timestamp);
  }

  /**
   * Update the telemetry of the drive.
   */
  public void updateTelemetry()
  {
    m_poseEstimator.update(new Rotation2d(getGyroAngle()), getModulePositions());
    m_gyroPublisher.accept(getGyroAngle().in(Degrees));
    m_currentModuleStatesPublisher.accept(getModuleStates());
    m_posePublisher.accept(getPose());
    m_currentRobotRelativeChassisSpeedsPublisher.accept(getRobotRelativeSpeed());
    m_fieldRelativeChassisSpeedsPublisher.accept(getFieldRelativeSpeed());
    Arrays.stream(m_modules).forEach(SwerveModule::updateTelemetry);
  }

  /**
   * Simulate the drive, updating the gyroscope based off of module states.
   */
  public void simIterate()
  {
    if (!m_simTimer.isRunning())
    {m_simTimer.start();}
    Arrays.stream(m_modules).forEach(SwerveModule::simIterate);
    m_simGyroAngle = m_simGyroAngle.plus(Radians.of(
        m_kinematics.toChassisSpeeds(getModuleStates()).omegaRadiansPerSecond * m_simTimer.get()));
    m_simTimer.reset();
  }

  /**
   * Get the robot relative speed of the drive.
   *
   * @return Robot relative speed of the drive.
   */
  public ChassisSpeeds getRobotRelativeSpeed()
  {
    return m_kinematics.toChassisSpeeds(getModuleStates());
  }

  /**
   * Get the field relative speed of the drive.
   *
   * @return Field relative speed of the drive.
   */
  public ChassisSpeeds getFieldRelativeSpeed()
  {
    return ChassisSpeeds.fromRobotRelativeSpeeds(getRobotRelativeSpeed(), new Rotation2d(getGyroAngle()));
  }

  /**
   * Get the {@link SwerveModulePosition} of the modules.
   *
   * @return {@link SwerveModulePosition} of the modules.
   */
  public SwerveModulePosition[] getModulePositions()
  {
    return Arrays.stream(m_modules)
                 .map(SwerveModule::getPosition)
                 .toArray(SwerveModulePosition[]::new);
  }

  /**
   * Get the {@link SwerveModuleState} of the modules.
   *
   * @return {@link SwerveModuleState} of the modules.
   */
  public SwerveModuleState[] getModuleStates()
  {
    return Arrays.stream(m_modules)
                 .map(SwerveModule::getState)
                 .toArray(SwerveModuleState[]::new);
  }

  /**
   * Get the {@link SwerveDriveConfig} of the drive.
   *
   * @return {@link SwerveDriveConfig} of the drive.
   */
  public SwerveDriveConfig getConfig()
  {
    return m_config;
  }
}
