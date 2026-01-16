package yams.mechanisms.swerve;

import static edu.wpi.first.hal.FRCNetComm.tInstances.kRobotDriveSwerve_YAGSL;
import static edu.wpi.first.hal.FRCNetComm.tResourceType.kResourceType_RobotDrive;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Second;

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
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import java.util.Arrays;
import java.util.function.Supplier;
import yams.mechanisms.config.SwerveDriveConfig;
import yams.telemetry.MechanismTelemetry;

/**
 * Swerve Drive mechanism
 */
public class SwerveDrive
{

  /**
   * The modules of the drive.
   */
  private final SwerveModule[]                          m_modules;
  /**
   * The pose estimator for the drive.
   */
  private final SwerveDrivePoseEstimator                m_poseEstimator;
  /**
   * The kinematics for the drive.
   */
  private final SwerveDriveKinematics                   m_kinematics;
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
   * Timer for simulation purposes only. Not used in real robot code.
   */
  private final Timer              m_simTimer     = new Timer();
  /**
   * The config for the drive.
   */
  private final SwerveDriveConfig  m_config;
  /**
   * Mechanism telemetry.
   */
  private final MechanismTelemetry m_telemetry    = new MechanismTelemetry();
  /**
   * Simulated Gyro Angle. Used for simulation purposes only. Not used in real robot code.
   */
  private       Angle              m_simGyroAngle = Rotations.of(0);

  /**
   * Create a SwerveDrive.
   *
   * @param config {@link SwerveDriveConfig} for the drive.
   */
  public SwerveDrive(SwerveDriveConfig config)
  {
    m_config = config;
    m_modules = config.getModules();
    m_kinematics = getKinematics();
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
   * @implNote Not compatible with AdvantageKit
   */
  public Command drive(Supplier<ChassisSpeeds> robotRelativeChassisSpeeds)
  {
    return Commands.run(() -> setRobotRelativeChassisSpeeds(robotRelativeChassisSpeeds.get()),
                        m_config.getSubsystem()).withName("Drive");
  }

  /**
   * Get the Gyro Angle.
   * @return Gyro angle, or maple sim odometry gyro angle.
   */
  public Angle getGyroAngle()
  {
    if (RobotBase.isSimulation())
    {
//      if (m_config.getMapleDriveSim().isPresent())
//      {
//        return m_config.getMapleDriveSim().get().getOdometryEstimatedPose().getRotation().getMeasure();
//      }
      return m_simGyroAngle;
    }
    return m_config.getGyroAngle();
  }

  /**
   * Point all modules toward the robot center, thus making the robot very difficult to move. Forcing the robot to keep
   * the current pose.
   * @implNote Not compatible with AdvantageKit
   */
  public void lockPose()
  {
    // Sets states
    SwerveModuleState[] desiredStates = new SwerveModuleState[m_modules.length];
    for (int i = 0; i < m_modules.length; i++)
    {
      desiredStates[i] =
          new SwerveModuleState(0, m_modules[i].getConfig().getLocation().orElseThrow().getAngle());
    }
    setSwerveModuleStates(desiredStates);

    // Update kinematics because we are not using setModuleStates
    m_desiredRobotRelativeChassisSpeedsPublisher.accept(new ChassisSpeeds());
  }

  /**
   * Set the {@link SwerveModuleState}s of the swerve drive directly.
   *
   * @param states {@link SwerveModuleState}s to use, must be the same count as the swerve drive is configured order is
   *               Clockwise from FL.
   * @implNote Not compatible with AdvantageKit if MapleSim is defined.
   */
  public void setSwerveModuleStates(SwerveModuleState[] states)
  {
    for (int i = 0; i < states.length; i++)
    {
      // if MapleSim is configured, run the swerve states through it.
//      if (RobotBase.isSimulation() && m_config.getMapleDriveSim().isPresent())
//      {
//        m_config.getMapleDriveSim().get().runSwerveStates(states);
//      }
      m_modules[i].setSwerveModuleState(states[i]);
    }
    m_desiredModuleStatesPublisher.accept(states);
  }

  /**
   * Get the {@link SwerveModuleState}s of the swerve drive given a robot relative chassis speed..
   *
   * @param robotRelativeChassisSpeeds Robot relative {@link ChassisSpeeds}.
   * @return {@link SwerveModuleState}s of the swerve drive.
   */
  public SwerveModuleState[] getStateFromRobotRelativeChassisSpeeds(ChassisSpeeds robotRelativeChassisSpeeds)
  {
    robotRelativeChassisSpeeds = m_config.optimizeRobotRelativeChassisSpeeds(robotRelativeChassisSpeeds);
    return m_config.getCenterOfRotation().isPresent() ?
           m_kinematics.toSwerveModuleStates(robotRelativeChassisSpeeds, m_config.getCenterOfRotation().get()) :
           m_kinematics.toSwerveModuleStates(robotRelativeChassisSpeeds);
  }

  /**
   * Set robot relative chassis speeds.
   *
   * @param robotRelativeChassisSpeeds Robot relative chassis speeds.
   */
  public void setRobotRelativeChassisSpeeds(ChassisSpeeds robotRelativeChassisSpeeds)
  {
    setSwerveModuleStates(getStateFromRobotRelativeChassisSpeeds(robotRelativeChassisSpeeds));
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
//    if (RobotBase.isSimulation() && m_config.getMapleDriveSim().isPresent())
//    {
//      return m_config.getMapleDriveSim().get().getOdometryEstimatedPose();
//    }
    return m_poseEstimator.getEstimatedPosition();
  }

  /**
   * Create the {@link SwerveDriveKinematics} so you can recreate a new {@link SwerveDrivePoseEstimator}.
   *
   * @return {@link SwerveDriveKinematics}
   */
  public SwerveDriveKinematics getKinematics()
  {
    return new SwerveDriveKinematics(Arrays.stream(m_modules)
                                           .map(module -> module.getConfig().getLocation().orElseThrow())
                                           .toArray(Translation2d[]::new));
  }

//  /**
//   * Gets the actual pose in the {@link org.ironmaple.simulation.SimulatedArena} from MapleSim.
//   *
//   * @return the robot's real pose.
//   * @implNote Not compatible with AdvantageKit
//   */
//  public Pose2d getMapleSimPose()
//  {
//    if (RobotBase.isSimulation())
//    {
//      return m_config.getMapleDriveSim().get().getActualPoseInSimulationWorld();
//    }
//    throw new IllegalStateException("getMapleSimPose() is only available in simulation.");
//  }

  /**
   * Resets the gyro angle to zero and resets odometry to the same position, but facing toward 0 (red alliance
   * station).
   * @implNote Not compatible with AdvantageKit
   */
  public void zeroGyro()
  {
    m_config.withGyroOffset(getGyroAngle().plus(m_config.getGyroOffset()));
    // If in sim reset to the simulated drive.
//    resetOdometry(
//        RobotBase.isSimulation() ? getMapleSimPose() : new Pose2d(getPose().getTranslation(), Rotation2d.kZero));
      resetOdometry(new Pose2d(getPose().getTranslation(), Rotation2d.kZero));
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
//    if (RobotBase.isSimulation() && m_config.getMapleDriveSim().isPresent())
//    {
//      m_config.getMapleDriveSim().get().resetOdometry(pose);
//      m_config.getMapleDriveSim().get().setSimulationWorldPose(pose);
//    }
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
   * Get the {@link Distance} from the given pose to the robot.
   *
   * @param pose {@link Pose2d} to get the distance from.
   * @return {@link Distance} from the given pose to the robot.
   */
  public Distance getDistanceFromPose(Pose2d pose)
  {
    return Meters.of(getPose().getTranslation().getDistance(pose.getTranslation()));
  }

  /**
   * Get the angle difference between the robot's current pose and the given pose.
   *
   * @param pose {@link Pose2d} to get the angle difference from.
   * @return {@link Angle} difference between the robot's current pose and the given pose.
   */
  public Angle getAngleDifferenceFromPose(Pose2d pose)
  {
    return getPose().minus(pose).getRotation().getMeasure();
  }

  /**
   * Drive the robot to the given pose.
   *
   * @param pose {@link Pose2d} to drive the robot to. Field relative, blue-origin where 0deg is facing towards RED
   * @return {@link Command} to drive the robot to the given pose.
   * @implNote Not compatible with AdvantageKit
   */
  public Command driveToPose(Pose2d pose)
  {
    return Commands.runOnce(() -> {
      resetTranslationPID();
      resetAzimuthPID();
    }).andThen(drive(() -> {
      var azimuthPID        = m_config.getRotationPID();
      var translationPID    = m_config.getTranslationPID();
      var distance          = getDistanceFromPose(pose);
      var angleDifference   = getAngleDifferenceFromPose(pose);
      var translationScalar = translationPID.calculate(distance.in(Meters), 0);
      var currentPose       = getPose();
      var poseDifference    = currentPose.minus(pose);
      return ChassisSpeeds.fromFieldRelativeSpeeds(poseDifference.getMeasureX().per(Second).times(translationScalar),
                                                   poseDifference.getMeasureY().per(Second).times(translationScalar),
                                                   RadiansPerSecond.of(azimuthPID.calculate(currentPose.getRotation()
                                                                                                       .getRadians(),
                                                                                            pose.getRotation()
                                                                                                .getRadians())),
                                                   new Rotation2d(getGyroAngle()));
    })).withName("Drive to Pose");
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
   * Update the {@link SwerveDrivePoseEstimator} with the current gyro angle and {@link SwerveModulePosition}
   */
  private void updatePoseEstimator()
  {
    m_poseEstimator.update(new Rotation2d(getGyroAngle()), getModulePositions());
  }

  /**
   * Update the telemetry and {@link SwerveDrivePoseEstimator} of the drive.
   */
  public void updateTelemetry()
  {
    updatePoseEstimator();
    m_gyroPublisher.accept(getGyroAngle().in(Degrees));
    m_currentModuleStatesPublisher.accept(getModuleStates());
    m_posePublisher.accept(getPose());
    m_currentRobotRelativeChassisSpeedsPublisher.accept(getRobotRelativeSpeed());
    m_fieldRelativeChassisSpeedsPublisher.accept(getFieldRelativeSpeed());
    Arrays.stream(m_modules).forEach(SwerveModule::updateTelemetry);
    m_telemetry.updateLoopTime();
  }

  /**
   * Simulate the drive, updating the gyroscope based off of module states.
   */
  public void simIterate()
  {
    // If MapleSim is configured, update it.
//    if (m_config.getMapleDriveSim().isPresent())
//    {
//      m_config.getMapleDriveSim().get().periodic();
//    }
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
    // If MapleSim is configured, return the simulated positions.
//    if (RobotBase.isSimulation() && m_config.getMapleDriveSim().isPresent())
//    {
//      return m_config.getMapleDriveSim().get().getLatestModulePositions();
//    }
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
    // If MapleSim is configured, return the simulated states.
//    if (RobotBase.isSimulation() && m_config.getMapleDriveSim().isPresent())
//    {
//      return m_config.getMapleDriveSim().get().getMeasuredStates();
//    }
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
