// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package yams.core.mechanisms.swerve;

import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.Radians;
import static org.wpilib.units.Units.RadiansPerSecond;
import static org.wpilib.units.Units.Rotations;
import static org.wpilib.units.Units.Second;

import java.util.Arrays;
import java.util.Optional;
import org.wpilib.framework.RobotBase;
import org.wpilib.hardware.hal.HAL;
import org.wpilib.math.controller.PIDController;
import org.wpilib.math.estimator.SwerveDrivePoseEstimator;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.geometry.Twist2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.math.kinematics.SwerveDriveKinematics;
import org.wpilib.math.kinematics.SwerveModulePosition;
import org.wpilib.math.kinematics.SwerveModuleVelocity;
import org.wpilib.math.linalg.Matrix;
import org.wpilib.math.numbers.N1;
import org.wpilib.math.numbers.N3;
import org.wpilib.smartdashboard.Field2d;
import org.wpilib.system.Timer;
import org.wpilib.tunable.Tunables;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.Distance;
import org.wpilib.units.measure.Force;
import yams.core.exceptions.SwerveDriveConfigurationException;
import yams.core.mechanisms.config.SwerveDriveConfig;
import yams.core.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.core.telemetry.MechanismTelemetry;
import yams.core.telemetry.NetworkTablesBackends;
import yams.core.telemetry.SwerveDriveTelemetry;
import yams.core.telemetry.SwerveDriveTelemetryConfig;

/**
 * Swerve Drive mechanism.
 *
 * <p>This core class holds swerve state, odometry, and physics simulation only. Command
 * factories such as {@code drive()} live on {@link yams.commands2.swerve.SwerveDrive}, which
 * extends this class. See that class's Javadoc for a full usage example.
 */
public class SwerveDrive {
  /**
   * The modules of the drive.
   */
  private final SwerveModule[] m_modules;
  /**
   * The pose estimator for the drive.
   */
  private final SwerveDrivePoseEstimator m_poseEstimator;
  /**
   * The kinematics for the drive.
   */
  private final SwerveDriveKinematics m_kinematics;
  /**
   * Timer for simulation purposes only. Not used in real robot code.
   */
  private final Timer m_simTimer = new Timer();
  /**
   * The config for the drive.
   */
  private final SwerveDriveConfig m_config;
  /**
   * Mechanism telemetry, used for the loop time and the {@link Field2d}.
   */
  private final MechanismTelemetry m_telemetry = new MechanismTelemetry();
  /**
   * Pose, chassis speeds, gyro, and auto-align tuning telemetry.
   */
  private SwerveDriveTelemetry m_swerveTelemetry;
  /**
   * Simulated Gyro Angle. Used for simulation purposes only. Not used in real robot code.
   */
  private Angle m_simGyroAngle = Rotations.of(0);
  /**
   * Field to display the robot's pose.
   */
  private Field2d m_field2d = new Field2d();
  /**
   * Last-commanded desired module states; cached and published from updateTelemetry.
   */
  private SwerveModuleVelocity[] m_desiredModuleStates;

  /**
   * Last-commanded desired robot-relative chassis speeds; cached and published from
   * updateTelemetry.
   */
  private ChassisVelocities m_desiredChassisSpeeds = new ChassisVelocities();

  /** The simulated pose of the robot if the {@link SwerveModuleVelocity}s were met perfectly. */
  private Pose2d m_simPose = new Pose2d();

  /**
   * Create a SwerveDrive.
   *
   * @param config {@link SwerveDriveConfig} for the drive.
   * @implNote Protected so only {@link yams.commands2.swerve.SwerveDrive} can construct this.
   */
  protected SwerveDrive(SwerveDriveConfig config) {
    m_config = config;
    m_modules = config.getModules();
    m_desiredModuleStates = new SwerveModuleVelocity[m_modules.length];
    m_simPose = config.getInitialPose();
    Arrays.fill(m_desiredModuleStates, new SwerveModuleVelocity());
    m_kinematics = getKinematics();
    m_poseEstimator = new SwerveDrivePoseEstimator(m_kinematics, new Rotation2d(getGyroAngle()), getModulePositions(), m_config.getInitialPose());
    // Start with the gyro reading the starting pose's heading so field relative driving matches it.
    resetOdometry(m_config.getInitialPose());
    setupTelemetry();
  }

  /**
   * Setup telemetry for the drive; the {@link SwerveDriveTelemetry} config used is either the one
   * supplied via {@link SwerveDriveConfig#withTelemetry(String, SwerveDriveTelemetryConfig)} or a
   * default built from {@link SwerveDriveConfig#getTelemetryVerbosity()} (defaulting to {@link
   * TelemetryVerbosity#HIGH}).
   */
  private void setupTelemetry() {
    var cfg = m_config.getSwerveDriveTelemetryConfig().orElseGet(() -> new SwerveDriveTelemetryConfig().withTelemetryVerbosity(m_config.getTelemetryVerbosity().orElse(TelemetryVerbosity.HIGH)));
    if (cfg.getDataLogName().isPresent()) {
      m_telemetry.setupTelemetry(getName(), cfg.getDataLogName().get());
    } else {
      m_telemetry.setupTelemetry(getName());
    }

    m_swerveTelemetry = new SwerveDriveTelemetry(cfg);
    m_swerveTelemetry.setupTelemetry(this);
    m_field2d.setRobotPose(getPose());
    NetworkTablesBackends.ensureMechanismsTunableBackend();
    Tunables.publish("Mechanisms/" + getName() + "/field", m_field2d);
    // Report as YAGSL bc this will become apart of YAGSL in 2027...
    HAL.reportUsage("RobotDrive", "YAGSL");
  }

  /**
   * Reset the auto-align PID controllers and pull the live-tuned values (from NetworkTables) into
   * this {@link SwerveDrive}. Intended to be wired into a periodic/scheduled callback by the
   * command layer, e.g. via {@code yams.commands2.swerve.SwerveDrive}'s drive-to-pose tuning
   * command.
   */
  public void startDriveToPoseTuning() {
    resetTranslationPID();
    resetRotationPID();
  }

  /**
   * Apply the live-tuned values (from NetworkTables) to this {@link SwerveDrive}'s drive-to-pose
   * controllers. Intended to be wired into a periodic/scheduled callback by the command layer.
   */
  public void applyDriveToPoseTuningValues() {
    m_swerveTelemetry.applyTuningValues(this);
  }

  /**
   * Get the Gyro Angle.
   *
   * @return Gyro angle, or maple sim odometry gyro angle.
   */
  public Angle getGyroAngle() {
    if (RobotBase.isSimulation()) {
      //      if (m_config.getMapleDriveSim().isPresent())
      //      {
      //        return
      //
      // m_config.getMapleDriveSim().get().getOdometryEstimatedPose().getRotation().getMeasure();
      //      }
      return m_simGyroAngle;
    }
    return m_config.getGyroAngle();
  }

  /**
   * Get the simulated pose of the robot, assuming the {@link SwerveModuleVelocity}s commanded to
   * the modules are met perfectly. Useful for feeding a simulated vision system with ground-truth
   * poses under perfect-world conditions.
   *
   * @return Simulated {@link Pose2d} of the robot. Only updated in simulation by {@link
   *         #simIterate()}; on a real robot this remains the configured starting pose.
   */
  public Pose2d getSimPose() {
    return m_simPose;
  }

  /**
   * Point all modules toward the robot center, thus making the robot very difficult to move.
   * Forcing the robot to keep the current pose.
   *
   * @implNote Not compatible with AdvantageKit
   */
  public void lockPose() {
    // Sets states
    SwerveModuleVelocity[] desiredStates = new SwerveModuleVelocity[m_modules.length];
    for (int i = 0; i < m_modules.length; i++) {
      desiredStates[i] = new SwerveModuleVelocity(0, m_modules[i].getConfig().getLocation().orElseThrow().getAngle().orElse(new Rotation2d()));
    }
    setSwerveModuleStates(desiredStates);
    m_desiredChassisSpeeds = new ChassisVelocities();
  }

  /**
   * Set the {@link SwerveModuleVelocity}s of the swerve drive directly.
   *
   * @param states {@link SwerveModuleVelocity}s to use, must be the same count as the swerve
   *               drive is configured order is
   *               Clockwise from FL.
   * @implNote Not compatible with AdvantageKit if MapleSim is defined.
   */
  public void setSwerveModuleStates(SwerveModuleVelocity[] states) {
    for (int i = 0; i < states.length; i++) {
      // if MapleSim is configured, run the swerve states through it.
      //      if (RobotBase.isSimulation() && m_config.getMapleDriveSim().isPresent())
      //      {
      //        m_config.getMapleDriveSim().get().runSwerveStates(states);
      //      }
      m_desiredModuleStates[i] = m_modules[i].setSwerveModuleState(states[i]);
    }
  }

  /**
   * Set the {@link SwerveModuleVelocity}s of the swerve drive directly, with an additional drive
   * wheel feedforward {@link Force} applied per module, e.g. from a PathPlanner set-point
   * generator.
   *
   * @param states            {@link SwerveModuleVelocity}s to use, must be the same count as the
   *                          swerve
   *                          drive is configured order is Clockwise from FL.
   * @param feedforwardForces Feedforward {@link Force}s to apply, one per module in the same FL,
   *                          FR, BL, BR order as {@code states}.
   * @implNote Not compatible with AdvantageKit if MapleSim is defined.
   */
  public void setSwerveModuleStates(SwerveModuleVelocity[] states, Force[] feedforwardForces) {
    for (int i = 0; i < states.length; i++) {
      m_desiredModuleStates[i] = m_modules[i].setSwerveModuleState(states[i], feedforwardForces[i]);
    }
  }

  /**
   * Get the {@link SwerveModuleVelocity}s of the swerve drive given a robot relative chassis
   * speed..
   *
   * @param robotRelativeChassisSpeeds Robot relative {@link ChassisVelocities}.
   * @return {@link SwerveModuleVelocity}s of the swerve drive.
   */
  public SwerveModuleVelocity[] getStateFromRobotRelativeChassisSpeeds(ChassisVelocities robotRelativeChassisSpeeds) {
    robotRelativeChassisSpeeds = m_config.optimizeRobotRelativeChassisSpeeds(robotRelativeChassisSpeeds);
    return m_config.getCenterOfRotation().isPresent() ? m_kinematics.toSwerveModuleVelocities(robotRelativeChassisSpeeds, m_config.getCenterOfRotation().get()) : m_kinematics.toSwerveModuleVelocities(robotRelativeChassisSpeeds);
  }

  /**
   * Get the {@link ChassisVelocities} based off the {@link SwerveModuleVelocity}s.
   *
   * @param states {@link SwerveModuleVelocity}s to use, must be the same count as the swerve
   *               drive is.
   *
   * @return {@link ChassisVelocities} based off the {@link SwerveModuleVelocity}s.
   */
  public ChassisVelocities getRobotRelativeChassisSpeedsFromState(SwerveModuleVelocity[] states) {
    return m_kinematics.toChassisVelocities(states);
  }

  /**
   * Set robot relative chassis speeds.
   *
   * @param robotRelativeChassisSpeeds Robot relative chassis speeds.
   */
  public void setRobotRelativeChassisSpeeds(ChassisVelocities robotRelativeChassisSpeeds) {
    setRobotRelativeChassisSpeeds(robotRelativeChassisSpeeds, new Force[0]);
  }

  /**
   * Set robot relative chassis speeds, with an additional drive wheel feedforward {@link Force}
   * applied per module, e.g. from a PathPlanner set-point generator.
   *
   * @param robotRelativeChassisSpeeds Robot relative chassis speeds.
   * @param feedforwardForces          Feedforward {@link Force}s to apply, one per module in FL,
   *                                   FR, BL, BR
   *                                   order. Pass an empty array to apply no feedforward.
   */
  public void setRobotRelativeChassisSpeeds(ChassisVelocities robotRelativeChassisSpeeds, Force[] feedforwardForces) {
    m_desiredChassisSpeeds = robotRelativeChassisSpeeds;
    SwerveModuleVelocity[] states = getStateFromRobotRelativeChassisSpeeds(robotRelativeChassisSpeeds);
    if (feedforwardForces.length == 0) {
      setSwerveModuleStates(states);
      return;
    }
    if (feedforwardForces.length != states.length) {
      throw new IllegalArgumentException("feedforwardForces must be empty or have one entry per module (" + states.length + "), in FL, FR, BL, BR order.");
    }
    setSwerveModuleStates(states, feedforwardForces);
  }

  /**
   * Set field relative chassis speeds.
   *
   * @param fieldRelativeChassisSpeeds Field relative chassis speeds.
   */
  public void setFieldRelativeChassisSpeeds(ChassisVelocities fieldRelativeChassisSpeeds) {
    setRobotRelativeChassisSpeeds(fieldRelativeChassisSpeeds.toRobotRelative(new Rotation2d(getGyroAngle())));
  }

  /**
   * Gets the measured pose (position and rotation) of the robot, as reported by odometry.
   *
   * @return The robot's pose
   */
  public Pose2d getPose() {
    //    if (RobotBase.isSimulation() && m_config.getMapleDriveSim().isPresent())
    //    {
    //      return m_config.getMapleDriveSim().get().getOdometryEstimatedPose();
    //    }
    return m_poseEstimator.getEstimatedPosition();
  }

  /**
   * Create the {@link SwerveDriveKinematics} so you can recreate a new {@link
   * SwerveDrivePoseEstimator}.
   *
   * @return {@link SwerveDriveKinematics}
   */
  public SwerveDriveKinematics getKinematics() {
    return new SwerveDriveKinematics(Arrays.stream(m_modules).map(module -> module.getConfig().getLocation().orElseThrow()).toArray(Translation2d[]::new));
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
   * Resets the gyro angle to zero and resets odometry to the same position, but facing toward 0
   * (red alliance station).
   *
   * @implNote Not compatible with AdvantageKit
   */
  public void zeroGyro() {
    // resetOdometry also sets the gyro to read the new heading.
    resetOdometry(new Pose2d(getPose().getTranslation(), Rotation2d.ZERO));
  }

  /**
   * Make {@link #getGyroAngle()} read the given heading from now on. On a real robot the gyro offset
   * is adjusted; in simulation the simulated gyro angle is set directly.
   *
   * @param heading Heading the gyro should report.
   */
  private void setGyroAngle(Angle heading) {
    if (RobotBase.isSimulation()) {
      m_simGyroAngle = heading;
      return;
    }
    m_config.withGyroOffset(getGyroAngle().plus(m_config.getGyroOffset()).minus(heading));
  }

  /**
   * Get the name of the drive.
   *
   * @return Name of the drive.
   */
  public String getName() {
    return m_config.getTelemetryName();
  }

  /**
   * Resets odometry to the given pose, and sets the gyro to read the pose's heading. Field relative
   * driving and heading control use the gyro, so keeping it aligned with the pose makes them agree
   * with the reset pose. Module positions do not need to be reset when calling this method.
   *
   * @param pose The pose to set the odometry to. Field relative, blue-origin where 0deg is facing
   *             towards RED alliance.
   */
  public void resetOdometry(Pose2d pose) {
    //    if (RobotBase.isSimulation() && m_config.getMapleDriveSim().isPresent())
    //    {
    //      m_config.getMapleDriveSim().get().resetOdometry(pose);
    //      m_config.getMapleDriveSim().get().setSimulationWorldPose(pose);
    //    }
    setGyroAngle(pose.getRotation().getMeasure());
    m_poseEstimator.resetPosition(new Rotation2d(getGyroAngle()), getModulePositions(), pose);
    m_desiredChassisSpeeds = new ChassisVelocities();
    m_desiredModuleStates = m_kinematics.toSwerveModuleVelocities(new ChassisVelocities());
    m_simPose = pose;
  }

  /** Resets the auto-align rotational PID controller, if one is configured. */
  public void resetRotationPID() {
    m_config.getRotationPID().ifPresent(PIDController::reset);
  }

  /** Resets the auto-align translation PID controller, if one is configured. */
  public void resetTranslationPID() {
    m_config.getTranslationPID().ifPresent(PIDController::reset);
  }

  /**
   * Set the auto-align rotational PID controller.
   *
   * @param controller {@link PIDController} to use, units given is in Radians.
   */
  public void setRotationPID(PIDController controller) {
    var currentRotationPID = m_config.getRotationPID();
    if (currentRotationPID.isEmpty() || currentRotationPID.get().getP() != controller.getP() || currentRotationPID.get().getI() != controller.getI() || currentRotationPID.get().getD() != controller.getD()) {
      controller.reset();
      m_config.withRotationController(controller);
    }
  }

  /**
   * Sets the auto-align translation PID controller.
   *
   * @param controller {@link PIDController} to reset, Units given is in Meters.
   */
  public void setTranslationPID(PIDController controller) {
    var currentTranslationPID = m_config.getTranslationPID();
    if (currentTranslationPID.isEmpty() || currentTranslationPID.get().getP() != controller.getP() || currentTranslationPID.get().getI() != controller.getI() || currentTranslationPID.get().getD() != controller.getD()) {
      controller.reset();
      m_config.withTranslationController(controller);
    }
  }

  /**
   * Get the {@link Distance} from the given pose to the robot.
   *
   * @param pose {@link Pose2d} to get the distance from.
   * @return {@link Distance} from the given pose to the robot.
   */
  public Distance getDistanceFromPose(Pose2d pose) {
    return Meters.of(getPose().getTranslation().getDistance(pose.getTranslation()));
  }

  /**
   * Get the angle difference between the robot's current pose and the given pose.
   *
   * @param pose {@link Pose2d} to get the angle difference from.
   * @return {@link Angle} difference between the robot's current pose and the given pose.
   */
  public Angle getAngleDifferenceFromPose(Pose2d pose) {
    return getPose().minus(pose).getRotation().getMeasure();
  }

  /**
   * Drive to the target pose, primarily for use in Live Tuning, could also be used for setpoint
   * commands.
   *
   * @param targetPose Pose to drive towards.
   * @implNote Remember to call {@link #resetRotationPID()} and {@link #resetTranslationPID()}
   *           before calling this method in a loop.
   * @return robot-relative {@link ChassisVelocities} to drive the robot to the given pose.
   * @throws SwerveDriveConfigurationException if the translation or rotation PID controller is not
   *                                           configured.
   */
  public ChassisVelocities driveToPoseSetpoint(Pose2d targetPose) {
    var rotationPID = m_config.getRotationPID().orElseThrow(() -> new SwerveDriveConfigurationException("No rotation PID controller configured", "Cannot drive to pose", "withRotationController(PIDController)"));
    var translationPID = m_config.getTranslationPID().orElseThrow(() -> new SwerveDriveConfigurationException("No translation PID controller configured", "Cannot drive to pose", "withTranslationController(PIDController)"));
    var distance = getDistanceFromPose(targetPose);
    var translationScalar = translationPID.calculate(distance.in(Meters), 0);
    var currentPose = getPose();
    // Plain field-frame translation delta (not Pose2d.minus(), which expresses the result in
    // targetPose's rotated frame and would skew the commanded direction whenever targetPose's
    // heading is non-zero).
    var translationDifference = currentPose.getTranslation().minus(targetPose.getTranslation());
    return new ChassisVelocities(translationDifference.getMeasureX().per(Second).times(translationScalar), translationDifference.getMeasureY().per(Second).times(translationScalar), RadiansPerSecond.of(rotationPID.calculate(currentPose.getRotation()
        .getRadians(), targetPose.getRotation().getRadians()))).toRobotRelative(new Rotation2d(getGyroAngle()));
  }

  /**
   * Add a vision measurement to the {@link SwerveDrivePoseEstimator} and update the gyro reading
   * with the given timestamp of the vision measurement.
   *
   * @param robotPose                Robot {@link Pose2d} as measured by vision.
   * @param timestamp                Timestamp the measurement was taken as time since startup,
   *                                 should be taken from
   *                                 {@link Timer#getTimestamp()} or similar sources.
   * @param visionMeasurementStdDevs Vision measurement standard deviation that will be sent to the
   *                                 {@link SwerveDrivePoseEstimator}.The standard deviation of the
   *                                 vision measurement, for best accuracy calculate the standard deviation at 2 or more points and
   *                                 fit a line to it with the calculated optimal standard deviation. (Units should be meters per
   *                                 pixel). By optimizing this you can get * vision accurate to inches instead of feet.
   */
  public void addVisionMeasurement(Pose2d robotPose, double timestamp, Matrix<N3, N1> visionMeasurementStdDevs) {
    m_poseEstimator.addVisionMeasurement(robotPose, timestamp, visionMeasurementStdDevs);
  }

  /**
   * Sets the pose estimator's trust of global measurements. This might be used to change trust in
   * vision measurements after the autonomous period, or to change trust as distance to a vision
   * target increases.
   *
   * @param visionMeasurementStdDevs Standard deviations of the vision measurements. Increase these
   *                                 numbers to trust global measurements from vision less. This
   *                                 matrix is in the form [x, y, theta], with units in meters and radians.
   */
  public void setVisionMeasurementStdDevs(Matrix<N3, N1> visionMeasurementStdDevs) {
    m_poseEstimator.setVisionMeasurementStdDevs(visionMeasurementStdDevs);
  }

  /**
   * Add a vision measurement to the {@link SwerveDrivePoseEstimator} and update the gyro reading
   * with the given timestamp of the vision measurement.
   *
   * @param robotPose Robot {@link Pose2d} as measured by vision.
   * @param timestamp Timestamp the measurement was taken as time since startup, should be taken
   *                  from {@link Timer#getTimestamp()} or similar sources.
   */
  public void addVisionMeasurement(Pose2d robotPose, double timestamp) {
    m_poseEstimator.addVisionMeasurement(robotPose, timestamp);
  }

  /**
   * Update the {@link SwerveDrivePoseEstimator} with the current gyro angle and {@link
   * SwerveModulePosition}
   */
  private void updatePoseEstimator() {
    m_poseEstimator.update(new Rotation2d(getGyroAngle()), getModulePositions());
  }

  /** Update the telemetry and {@link SwerveDrivePoseEstimator} of the drive. */
  public void updateTelemetry() {
    updatePoseEstimator();
    Pose2d robotPose = getPose();
    SwerveModuleVelocity[] currentStates = getModuleStates();

    m_swerveTelemetry.publish(this);

    Arrays.stream(m_modules).forEach(SwerveModule::updateTelemetry);
    m_telemetry.updateLoopTime();

    m_field2d.setRobotPose(robotPose);
    Pose2d[] modulePoses = new Pose2d[m_modules.length];
    for (int i = 0; i < m_modules.length; i++) {
      Translation2d location = m_modules[i].getConfig().getLocation().orElseThrow();
      Translation2d rotated = location.rotateBy(robotPose.getRotation());
      Translation2d moduleTranslation = robotPose.getTranslation().plus(rotated);
      Rotation2d moduleHeading = robotPose.getRotation().plus(currentStates[i].angle);
      modulePoses[i] = new Pose2d(moduleTranslation, moduleHeading);
    }
    m_field2d.getObject("modules").setPoses(modulePoses);
  }

  /** Simulate the drive, updating the gyroscope based off of module states. */
  public void simIterate() {
    // If MapleSim is configured, update it.
    //    if (m_config.getMapleDriveSim().isPresent())
    //    {
    //      m_config.getMapleDriveSim().get().periodic();
    //    }
    if (!m_simTimer.isRunning()) {
      m_simTimer.start();
    }
    Arrays.stream(m_modules).forEach(SwerveModule::simIterate);
    ChassisVelocities desired = m_kinematics.toChassisVelocities(m_desiredModuleStates);

    var dt = m_simTimer.get();
    Twist2d twist = new Twist2d(desired.vx * dt, desired.vy * dt, desired.omega * dt);
    m_simPose = m_simPose.plus(twist.exp());
    m_simGyroAngle = m_simGyroAngle.plus(Radians.of(m_kinematics.toChassisVelocities(getModuleStates()).omega * dt));
    m_simTimer.reset();
  }

  /**
   * Get the robot relative speed of the drive.
   *
   * @return Robot relative speed of the drive.
   */
  public ChassisVelocities getRobotRelativeSpeed() {
    return m_kinematics.toChassisVelocities(getModuleStates());
  }

  /**
   * Get the field relative speed of the drive.
   *
   * @return Field relative speed of the drive.
   */
  public ChassisVelocities getFieldRelativeSpeed() {
    return getRobotRelativeSpeed().toFieldRelative(new Rotation2d(getGyroAngle()));
  }

  /**
   * Get the {@link SwerveModulePosition} of the modules.
   *
   * @return {@link SwerveModulePosition} of the modules.
   */
  public SwerveModulePosition[] getModulePositions() {
    // If MapleSim is configured, return the simulated positions.
    //    if (RobotBase.isSimulation() && m_config.getMapleDriveSim().isPresent())
    //    {
    //      return m_config.getMapleDriveSim().get().getLatestModulePositions();
    //    }
    return Arrays.stream(m_modules).map(SwerveModule::getPosition).toArray(SwerveModulePosition[]::new);
  }

  /**
   * Get the {@link SwerveModuleVelocity} of the modules.
   *
   * @return {@link SwerveModuleVelocity} of the modules.
   */
  public SwerveModuleVelocity[] getModuleStates() {
    // If MapleSim is configured, return the simulated states.
    //    if (RobotBase.isSimulation() && m_config.getMapleDriveSim().isPresent())
    //    {
    //      return m_config.getMapleDriveSim().get().getMeasuredStates();
    //    }
    return Arrays.stream(m_modules).map(SwerveModule::getState).toArray(SwerveModuleVelocity[]::new);
  }

  /**
   * Get the {@link SwerveDriveConfig} of the drive.
   *
   * @return {@link SwerveDriveConfig} of the drive.
   */
  public SwerveDriveConfig getConfig() {
    return m_config;
  }

  /**
   * Get the {@link Field2d} used to display the robot's pose, so callers (e.g. vision subsystems)
   * can publish additional {@link org.wpilib.smartdashboard.FieldObject2d}s onto the
   * same field widget instead of creating their own.
   *
   * @return {@link Field2d} of the drive.
   */
  public Field2d getField2d() {
    return m_field2d;
  }

  /**
   * Get the last-commanded desired robot-relative {@link ChassisVelocities} of the drive. This is
   * the value cached by {@link #setRobotRelativeChassisSpeeds(ChassisVelocities)} (which
   * {@link #setFieldRelativeChassisSpeeds(ChassisVelocities)} and
   * {@link yams.commands2.swerve.SwerveDrive#drive(java.util.function.Supplier)} funnel through)
   * and published on every {@link #updateTelemetry()} call.
   *
   * @implNote It is a setpoint, not a measurement of actual robot motion.
   *           Use {@link #getRobotRelativeSpeed()} or {@link #getFieldRelativeSpeed()} instead if
   *           you need the drive's actual measured speed.
   *
   * @return Robot-relative {@link ChassisVelocities} last commanded to the drive. Defaults to a
   *         zeroed {@link ChassisVelocities} if the drive has never been commanded.
   */
  public ChassisVelocities getDesiredChassisSpeeds() {
    return m_desiredChassisSpeeds;
  }

  /**
   * Get the last-commanded desired {@link SwerveModuleVelocity}s of the drive.
   *
   * @return {@link SwerveModuleVelocity}s last commanded to the drive. Defaults to a zeroed {@link
   *         SwerveModuleVelocity}s if the drive has never been commanded.
   */
  public SwerveModuleVelocity[] getDesiredModuleStates() {
    return m_desiredModuleStates;
  }

  /**
   * Get a module by its name.
   *
   * @param moduleName Name of the module.
   * @return {@link SwerveModule} with the given name if it exists.
   */
  public Optional<SwerveModule> getModule(String moduleName) {
    Optional<SwerveModule> module = Optional.empty();
    for (var mod : m_modules) {
      if (mod.getName().equals(moduleName)) {
        module = Optional.of(mod);
        break;
      }
    }
    return module;
  }

  /**
   * Get all {@link SwerveModule}s for this {@link SwerveDrive}.
   *
   * @return Array of {@link SwerveModule}s.
   */
  public SwerveModule[] getModules() {
    return m_modules;
  }
}
