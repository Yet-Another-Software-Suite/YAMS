// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package yams.mechanisms.swerve;

import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.Radians;
import static org.wpilib.units.Units.RadiansPerSecond;
import static org.wpilib.units.Units.Rotations;
import static org.wpilib.units.Units.Second;
import static org.wpilib.units.Units.Seconds;

import org.wpilib.hardware.hal.HAL;
import org.wpilib.math.linalg.Matrix;
import org.wpilib.math.estimator.SwerveDrivePoseEstimator;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.math.kinematics.SwerveDriveKinematics;
import org.wpilib.math.kinematics.SwerveModulePosition;
import org.wpilib.math.kinematics.SwerveModuleVelocity;
import org.wpilib.math.numbers.N1;
import org.wpilib.math.numbers.N3;
import org.wpilib.networktables.DoublePublisher;
import org.wpilib.networktables.StructArrayPublisher;
import org.wpilib.networktables.StructPublisher;
import org.wpilib.units.VoltageUnit;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.Distance;
import org.wpilib.units.measure.Time;
import org.wpilib.units.measure.Velocity;
import org.wpilib.units.measure.Voltage;
import org.wpilib.framework.RobotBase;
import org.wpilib.system.Timer;
import org.wpilib.smartdashboard.Field2d;
import org.wpilib.smartdashboard.SmartDashboard;
import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import org.wpilib.command2.RunCommand;
import java.util.Arrays;
import java.util.Optional;
import java.util.function.Supplier;
import yams.mechanisms.config.SwerveDriveConfig;
import yams.motorcontrollers.SmartMotorController;
import yams.telemetry.MechanismTelemetry;

/**
 * Swerve Drive mechanism
 *
 * <h2>Usage Example</h2>
 * <p>
 * The typical pattern is to wrap {@code SwerveDrive} inside a WPILib {@code SubsystemBase}.
 * Build a {@link yams.mechanisms.config.SwerveDriveConfig} (see its class-level doc for the full
 * module-construction example), then instantiate {@code SwerveDrive} once in the subsystem
 * constructor.
 * </p>
 * <p>
 * {@link yams.mechanisms.swerve.utility.SwerveInputStream} is the <b>recommended</b> way to
 * convert raw joystick axes into field-relative {@link org.wpilib.math.kinematics.ChassisVelocities}
 * before passing them to {@link #drive(java.util.function.Supplier)}.
 * </p>
 * <pre>{@code
 * public class SwerveSubsystem extends SubsystemBase {
 *
 *     private final SwerveDrive drive;
 *
 *     public SwerveSubsystem() {
 *         // Build SwerveDriveConfig — see SwerveDriveConfig class doc for the full example.
 *         Pigeon2 gyro = new Pigeon2(14);
 *         SwerveDriveConfig config = new SwerveDriveConfig(this, fl, fr, bl, br)
 *             .withGyro(gyro.getYaw().asSupplier())
 *             .withMaximumChassisSpeed(MetersPerSecond.of(4.5), DegreesPerSecond.of(360))
 *             .withTranslationController(new PIDController(1.0, 0, 0))
 *             .withRotationController(new PIDController(1.0, 0, 0))
 *             .withStartingPose(new Pose2d());
 *
 *         drive = new SwerveDrive(config);
 *     }
 *
 *     // Convert joystick input to a field-relative drive command using SwerveInputStream.
 *     public Command driveWithJoystick(CommandXboxController controller) {
 *         SwerveInputStream inputStream = new SwerveInputStream(
 *                 drive,
 *                 controller::getLeftX,
 *                 controller::getLeftY,
 *                 controller::getRightX)
 *             .withMaximumLinearVelocity(MetersPerSecond.of(4.5))
 *             .withMaximumAngularVelocity(DegreesPerSecond.of(360))
 *             .withDeadband(0.05)
 *             .withCubeTranslationControllerAxis()
 *             .withAllianceRelativeControl();
 *
 *         // drive() accepts a robot-relative ChassisVelocities supplier; SwerveInputStream handles
 *         // the field-to-robot conversion internally when alliance-relative control is enabled.
 *         return drive.drive(inputStream);
 *     }
 *
 *     // Alternatively, pass field-relative speeds directly.
 *     public Command driveFieldRelative(Supplier<ChassisVelocities> speedsSupplier) {
 *         return run(() -> drive.setFieldRelativeChassisSpeeds(speedsSupplier.get()))
 *             .withName("Field Oriented Drive");
 *     }
 *
 *     // Read the fused odometry pose at any time.
 *     public Pose2d getRobotPose() {
 *         return drive.getPose();
 *     }
 *
 *     // Update odometry and publish telemetry every robot loop (20 ms).
 *     {@literal @}Override
 *     public void periodic() {
 *         drive.updateTelemetry();
 *     }
 *
 *     // Advance the simulated gyro and modules every simulation loop.
 *     {@literal @}Override
 *     public void simulationPeriodic() {
 *         drive.simIterate();
 *     }
 * }
 * }</pre>
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
  private final StructArrayPublisher<SwerveModuleVelocity> m_desiredModuleStatesPublisher;
  /**
   * Current swerve module states.
   */
  private final StructArrayPublisher<SwerveModuleVelocity> m_currentModuleStatesPublisher;
  /**
   * Desired robot relative chassis speeds.
   */
  private final StructPublisher<ChassisVelocities>          m_desiredRobotRelativeChassisSpeedsPublisher;
  /**
   * Current robot relative chassis speeds.
   */
  private final StructPublisher<ChassisVelocities>          m_currentRobotRelativeChassisSpeedsPublisher;
  /**
   * Field relative chassis speeds.
   */
  private final StructPublisher<ChassisVelocities>          m_fieldRelativeChassisSpeedsPublisher;
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
  private final Timer                                   m_simTimer     = new Timer();
  /**
   * The config for the drive.
   */
  private final SwerveDriveConfig                       m_config;
  /**
   * Mechanism telemetry.
   */
  private final MechanismTelemetry                      m_telemetry    = new MechanismTelemetry();
  /**
   * Simulated Gyro Angle. Used for simulation purposes only. Not used in real robot code.
   */
  private       Angle                                   m_simGyroAngle = Rotations.of(0);
  /**
   * Field to display the robot's pose.
   */
  private       Field2d                                 m_field2d      = new Field2d();
  /**
   * Last-commanded desired module states; cached and published from updateTelemetry.
   */
  private       SwerveModuleVelocity[]                     m_desiredModuleStates;
  /**
   * Last-commanded desired robot-relative chassis speeds; cached and published from updateTelemetry.
   */
  private       ChassisVelocities                           m_desiredChassisSpeeds = new ChassisVelocities();

  /**
   * Create a SwerveDrive.
   *
   * @param config {@link SwerveDriveConfig} for the drive.
   */
  public SwerveDrive(SwerveDriveConfig config)
  {
    m_config = config;
    m_modules = config.getModules();
    m_desiredModuleStates = new SwerveModuleVelocity[m_modules.length];
    Arrays.fill(m_desiredModuleStates, new SwerveModuleVelocity());
    m_kinematics = getKinematics();
    m_poseEstimator = new SwerveDrivePoseEstimator(m_kinematics,
                                                   new Rotation2d(getGyroAngle()),
                                                   getModulePositions(),
                                                   m_config.getInitialPose());
    m_telemetry.setupTelemetry(getName());
    var desiredModuleStatesTopic = m_telemetry.getDataTable()
                                              .getStructArrayTopic("states/desired", SwerveModuleVelocity.struct);
    var currentModuleStatesTopic = m_telemetry.getDataTable()
                                              .getStructArrayTopic("states/current", SwerveModuleVelocity.struct);
    var poseTopic = m_telemetry.getDataTable().getStructTopic("pose", Pose2d.struct);
    var gyroTopic = m_telemetry.getDataTable().getDoubleTopic("gyro");
    gyroTopic.setProperties("{\"units\": \"degrees\"}");
    var desiredRobotRelativeChassisSpeedsTopic = m_telemetry.getDataTable()
                                                            .getStructTopic("chassis/desired", ChassisVelocities.struct);
    var fieldRelativeChassisSpeedsTopic = m_telemetry.getDataTable()
                                                     .getStructTopic("chassis/field", ChassisVelocities.struct);
    var currentRobotRelativeChassisSpeedsTopic = m_telemetry.getDataTable()
                                                            .getStructTopic("chassis/current", ChassisVelocities.struct);
    m_gyroPublisher = gyroTopic.publish();
    m_currentRobotRelativeChassisSpeedsPublisher = currentRobotRelativeChassisSpeedsTopic.publish();
    m_fieldRelativeChassisSpeedsPublisher = fieldRelativeChassisSpeedsTopic.publish();
    m_desiredRobotRelativeChassisSpeedsPublisher = desiredRobotRelativeChassisSpeedsTopic.publish();
    m_posePublisher = poseTopic.publish();
    m_desiredModuleStatesPublisher = desiredModuleStatesTopic.publish();
    m_currentModuleStatesPublisher = currentModuleStatesTopic.publish();
    m_field2d.setRobotPose(getPose());
    SmartDashboard.putData("Mechanisms/"+getName()+"/field", m_field2d);
    // Report as YAGSL bc this will become apart of YAGSL in 2027...
    HAL.reportUsage("RobotDrive", "YAGSL");
  }

  /**
   * Create a {@link RunCommand} to drive the swerve drive with robot relative chassis speeds.
   *
   * @param robotRelativeChassisSpeeds {@link Supplier<ChassisVelocities>} for the robot relative chassis speeds. Could also
   *                                   use {@link yams.mechanisms.swerve.utility.SwerveInputStream}
   * @return {@link RunCommand} to drive the swerve drive.
   * @implNote Not compatible with AdvantageKit
   */
  public Command drive(Supplier<ChassisVelocities> robotRelativeChassisSpeeds)
  {
    return Commands.run(() -> setRobotRelativeChassisSpeeds(robotRelativeChassisSpeeds.get()),
                        m_config.getSubsystem()).withName("Drive");
  }

  /**
   * Get the Gyro Angle.
   *
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
   *
   * @implNote Not compatible with AdvantageKit
   */
  public void lockPose()
  {
    // Sets states
    SwerveModuleVelocity[] desiredStates = new SwerveModuleVelocity[m_modules.length];
    for (int i = 0; i < m_modules.length; i++)
    {
      desiredStates[i] =
          new SwerveModuleVelocity(0, m_modules[i].getConfig().getLocation().orElseThrow().getAngle().orElse(new Rotation2d()));
    }
    setSwerveModuleStates(desiredStates);
    m_desiredChassisSpeeds = new ChassisVelocities();
  }

  /**
   * Set the {@link SwerveModuleVelocity}s of the swerve drive directly.
   *
   * @param states {@link SwerveModuleVelocity}s to use, must be the same count as the swerve drive is configured order is
   *               Clockwise from FL.
   * @implNote Not compatible with AdvantageKit if MapleSim is defined.
   */
  public void setSwerveModuleStates(SwerveModuleVelocity[] states)
  {
    for (int i = 0; i < states.length; i++)
    {
      // if MapleSim is configured, run the swerve states through it.
//      if (RobotBase.isSimulation() && m_config.getMapleDriveSim().isPresent())
//      {
//        m_config.getMapleDriveSim().get().runSwerveStates(states);
//      }
        m_desiredModuleStates[i] = m_modules[i].setSwerveModuleState(states[i]);
    }
  }

  /**
   * Get the {@link SwerveModuleVelocity}s of the swerve drive given a robot relative chassis speed..
   *
   * @param robotRelativeChassisSpeeds Robot relative {@link ChassisVelocities}.
   * @return {@link SwerveModuleVelocity}s of the swerve drive.
   */
  public SwerveModuleVelocity[] getStateFromRobotRelativeChassisSpeeds(ChassisVelocities robotRelativeChassisSpeeds)
  {
    robotRelativeChassisSpeeds = m_config.optimizeRobotRelativeChassisSpeeds(robotRelativeChassisSpeeds);
    return m_config.getCenterOfRotation().isPresent() ?
           m_kinematics.toSwerveModuleVelocities(robotRelativeChassisSpeeds, m_config.getCenterOfRotation().get()) :
           m_kinematics.toSwerveModuleVelocities(robotRelativeChassisSpeeds);
  }

  /**
   * Set robot relative chassis speeds.
   *
   * @param robotRelativeChassisSpeeds Robot relative chassis speeds.
   */
  public void setRobotRelativeChassisSpeeds(ChassisVelocities robotRelativeChassisSpeeds)
  {
    m_desiredChassisSpeeds = robotRelativeChassisSpeeds;
    setSwerveModuleStates(getStateFromRobotRelativeChassisSpeeds(robotRelativeChassisSpeeds));
  }

  /**
   * Set field relative chassis speeds.
   *
   * @param fieldRelativeChassisSpeeds Field relative chassis speeds.
   */
  public void setFieldRelativeChassisSpeeds(ChassisVelocities fieldRelativeChassisSpeeds)
  {
    setRobotRelativeChassisSpeeds(fieldRelativeChassisSpeeds.toRobotRelative(new Rotation2d(getGyroAngle())));
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
   *
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
    m_desiredChassisSpeeds = new ChassisVelocities();
    m_desiredModuleStates = m_kinematics.toSwerveModuleVelocities(new ChassisVelocities());
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
      return new ChassisVelocities(poseDifference.getMeasureX().per(Second).times(translationScalar),
                                   poseDifference.getMeasureY().per(Second).times(translationScalar),
                                   RadiansPerSecond.of(azimuthPID.calculate(currentPose.getRotation().getRadians(),
                                                                            pose.getRotation().getRadians())))
          .toRobotRelative(new Rotation2d(getGyroAngle()));
    })).withName("Drive to Pose");
  }

  /**
   * Add a vision measurement to the {@link SwerveDrivePoseEstimator} and update the gyro reading with the given
   * timestamp of the vision measurement.
   *
   * @param robotPose                Robot {@link Pose2d} as measured by vision.
   * @param timestamp                Timestamp the measurement was taken as time since startup, should be taken from
   *                                 {@link Timer#getTimestamp()} or similar sources.
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
   *                  {@link Timer#getTimestamp()} or similar sources.
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
    Pose2d             robotPose     = getPose();
    SwerveModuleVelocity[] currentStates = getModuleStates();

    m_gyroPublisher.accept(getGyroAngle().in(Degrees));
    m_desiredModuleStatesPublisher.accept(m_desiredModuleStates);
    m_currentModuleStatesPublisher.accept(currentStates);
    m_posePublisher.accept(robotPose);
    m_desiredRobotRelativeChassisSpeedsPublisher.accept(m_desiredChassisSpeeds);
    m_currentRobotRelativeChassisSpeedsPublisher.accept(getRobotRelativeSpeed());
    m_fieldRelativeChassisSpeedsPublisher.accept(getFieldRelativeSpeed());

    Arrays.stream(m_modules).forEach(SwerveModule::updateTelemetry);
    m_telemetry.updateLoopTime();

    m_field2d.setRobotPose(robotPose);
    Pose2d[] modulePoses = new Pose2d[m_modules.length];
    for (int i = 0; i < m_modules.length; i++)
    {
      Translation2d location          = m_modules[i].getConfig().getLocation().orElseThrow();
      Translation2d rotated           = location.rotateBy(robotPose.getRotation());
      Translation2d moduleTranslation = robotPose.getTranslation().plus(rotated);
      Rotation2d    moduleHeading     = robotPose.getRotation().plus(currentStates[i].angle);
      modulePoses[i] = new Pose2d(moduleTranslation, moduleHeading);
    }
    m_field2d.getObject("modules").setPoses(modulePoses);
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
        m_kinematics.toChassisVelocities(getModuleStates()).omega * m_simTimer.get()));
    m_simTimer.reset();
  }

  /**
   * Get the robot relative speed of the drive.
   *
   * @return Robot relative speed of the drive.
   */
  public ChassisVelocities getRobotRelativeSpeed()
  {
    return m_kinematics.toChassisVelocities(getModuleStates());
  }

  /**
   * Get the field relative speed of the drive.
   *
   * @return Field relative speed of the drive.
   */
  public ChassisVelocities getFieldRelativeSpeed()
  {
    return getRobotRelativeSpeed().toFieldRelative(new Rotation2d(getGyroAngle()));
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
   * Get the {@link SwerveModuleVelocity} of the modules.
   *
   * @return {@link SwerveModuleVelocity} of the modules.
   */
  public SwerveModuleVelocity[] getModuleStates()
  {
    // If MapleSim is configured, return the simulated states.
//    if (RobotBase.isSimulation() && m_config.getMapleDriveSim().isPresent())
//    {
//      return m_config.getMapleDriveSim().get().getMeasuredStates();
//    }
    return Arrays.stream(m_modules)
                 .map(SwerveModule::getState)
                 .toArray(SwerveModuleVelocity[]::new);
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


  /**
   * Get a module by its name.
   *
   * @param moduleName Name of the module.
   * @return {@link SwerveModule} with the given name if it exists.
   */
  public Optional<SwerveModule> getModule(String moduleName)
  {
    Optional<SwerveModule> module = Optional.empty();
    for (var mod : m_modules)
    {
      if (mod.getName().equals(moduleName))
      {
        module = Optional.of(mod);
        break;
      }
    }
    return module;
  }
}
