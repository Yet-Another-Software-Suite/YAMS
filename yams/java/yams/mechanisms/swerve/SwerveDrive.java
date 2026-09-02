// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package yams.mechanisms.swerve;

import static edu.wpi.first.hal.FRCNetComm.tInstances.kRobotDriveSwerve_YAGSL;
import static edu.wpi.first.hal.FRCNetComm.tResourceType.kResourceType_RobotDrive;
import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Second;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import java.util.Arrays;
import java.util.Optional;
import java.util.function.Supplier;
import yams.mechanisms.config.SwerveDriveConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.telemetry.MechanismTelemetry;
import yams.telemetry.SwerveDriveTelemetry;
import yams.telemetry.SwerveDriveTelemetryConfig;

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
 * convert raw joystick axes into field-relative {@link edu.wpi.first.math.kinematics.ChassisSpeeds}
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
 *         // drive() accepts a robot-relative ChassisSpeeds supplier; SwerveInputStream handles
 *         // the field-to-robot conversion internally when alliance-relative control is enabled.
 *         return drive.drive(inputStream);
 *     }
 *
 *     // Alternatively, pass field-relative speeds directly.
 *     public Command driveFieldRelative(Supplier<ChassisSpeeds> speedsSupplier) {
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
  private SwerveModuleState[] m_desiredModuleStates;
  /**
   * Last-commanded desired robot-relative chassis speeds; cached and published from
   * updateTelemetry.
   */
  private ChassisSpeeds m_desiredChassisSpeeds = new ChassisSpeeds();
  /**
   * The simulated pose of the robot if the {@link SwerveModuleState} were met perfectly.
   */
  private Pose2d m_simPose = new Pose2d();

  /**
   * Create a SwerveDrive.
   *
   * @param config {@link SwerveDriveConfig} for the drive.
   */
  public SwerveDrive(SwerveDriveConfig config) {
    m_config = config;
    m_modules = config.getModules();
    m_desiredModuleStates = new SwerveModuleState[m_modules.length];
    m_simPose = config.getInitialPose();
    Arrays.fill(m_desiredModuleStates, new SwerveModuleState());
    m_kinematics = getKinematics();
    m_poseEstimator = new SwerveDrivePoseEstimator(m_kinematics, new Rotation2d(getGyroAngle()),
        getModulePositions(), m_config.getInitialPose());
    setupTelemetry();
  }

  /**
   * Setup telemetry for the drive; the {@link SwerveDriveTelemetry} config used is either the one
   * supplied via
   * {@link SwerveDriveConfig#withTelemetry(SwerveDriveTelemetryConfig)} or a default built from
   * {@link SwerveDriveConfig#getTelemetryVerbosity()} (defaulting to {@link
   * TelemetryVerbosity#HIGH}).
   */
  private void setupTelemetry() {
    var cfg = m_config.getSwerveDriveTelemetryConfig().orElseGet(
        ()
            -> new SwerveDriveTelemetryConfig().withTelemetryVerbosity(
                m_config.getTelemetryVerbosity().orElse(TelemetryVerbosity.HIGH)));
    if (cfg.getDataLogName().isPresent()) {
      m_telemetry.setupTelemetry(getName(), cfg.getDataLogName().get());
    } else {
      m_telemetry.setupTelemetry(getName());
    }

    m_swerveTelemetry = new SwerveDriveTelemetry(cfg);
    m_swerveTelemetry.setupTelemetry(this);
    m_field2d.setRobotPose(getPose());
    SmartDashboard.putData("Mechanisms/" + getName() + "/field", m_field2d);
    SmartDashboard.putData(
        "Mechanisms/" + getName() + "/tuning", Commands.startRun(() -> {
          System.out.println(
              "================= Starting SwerveDrive Tuning =================\n");
          resetTranslationPID();
          resetRotationPID();
        }, () -> m_swerveTelemetry.applyTuningValues(this)));
    // Report as YAGSL bc this will become apart of YAGSL in 2027...s
    HAL.report(kResourceType_RobotDrive, kRobotDriveSwerve_YAGSL);
  }

  /**
   * Create a {@link RunCommand} to drive the swerve drive with robot relative chassis speeds.
   *
   * @param robotRelativeChassisSpeeds {@link Supplier<ChassisSpeeds>} for the robot relative
   *     chassis speeds. Could also use {@link yams.mechanisms.swerve.utility.SwerveInputStream}
   * @return {@link RunCommand} to drive the swerve drive.
   * @implNote Not compatible with AdvantageKit
   */
  public Command drive(Supplier<ChassisSpeeds> robotRelativeChassisSpeeds) {
    return Commands
        .run(()
                 -> setRobotRelativeChassisSpeeds(robotRelativeChassisSpeeds.get()),
            m_config.getSubsystem())
        .withName("Drive");
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
      //        m_config.getMapleDriveSim().get().getOdometryEstimatedPose().getRotation().getMeasure();
      //      }
      return m_simGyroAngle;
    }
    return m_config.getGyroAngle();
  }

  /**
   * Get the simulated pose of the robot, assuming the {@link SwerveModuleState}s commanded to the
   * modules are met perfectly. Useful for feeding a simulated vision system with ground-truth
   * poses under perfect-world conditions.
   *
   * @return Simulated {@link Pose2d} of the robot. Only updated in simulation by {@link
   *     #simIterate()}; on a real robot this remains the configured starting pose.
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
    SwerveModuleState[] desiredStates = new SwerveModuleState[m_modules.length];
    for (int i = 0; i < m_modules.length; i++) {
      desiredStates[i] =
          new SwerveModuleState(0, m_modules[i].getConfig().getLocation().orElseThrow().getAngle());
    }
    setSwerveModuleStates(desiredStates);
    m_desiredChassisSpeeds = new ChassisSpeeds();
  }

  /**
   * Set the {@link SwerveModuleState}s of the swerve drive directly.
   *
   * @param states {@link SwerveModuleState}s to use, must be the same count as the swerve drive is
   *     configured order is
   *               Clockwise from FL.
   * @implNote Not compatible with AdvantageKit if MapleSim is defined.
   */
  public void setSwerveModuleStates(SwerveModuleState[] states) {
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
   * Get the {@link SwerveModuleState}s of the swerve drive given a robot relative chassis speed..
   *
   * @param robotRelativeChassisSpeeds Robot relative {@link ChassisSpeeds}.
   * @return {@link SwerveModuleState}s of the swerve drive.
   */
  public SwerveModuleState[] getStateFromRobotRelativeChassisSpeeds(
      ChassisSpeeds robotRelativeChassisSpeeds) {
    robotRelativeChassisSpeeds =
        m_config.optimizeRobotRelativeChassisSpeeds(robotRelativeChassisSpeeds);
    return m_config.getCenterOfRotation().isPresent()
        ? m_kinematics.toSwerveModuleStates(
              robotRelativeChassisSpeeds, m_config.getCenterOfRotation().get())
        : m_kinematics.toSwerveModuleStates(robotRelativeChassisSpeeds);
  }

  /**
   * Get the {@link ChassisSpeeds} based off the {@link SwerveModuleState}s.
   * @param states {@link SwerveModuleState}s to use, must be the same count as the swerve drive is.
   *
   * @return {@link ChassisSpeeds} based off the {@link SwerveModuleState}s.
   */
  public ChassisSpeeds getRobotRelativeChassisSpeedsFromState(SwerveModuleState[] states) {
    return m_kinematics.toChassisSpeeds(states);
  }

  /**
   * Set robot relative chassis speeds.
   *
   * @param robotRelativeChassisSpeeds Robot relative chassis speeds.
   */
  public void setRobotRelativeChassisSpeeds(ChassisSpeeds robotRelativeChassisSpeeds) {
    m_desiredChassisSpeeds = robotRelativeChassisSpeeds;
    setSwerveModuleStates(getStateFromRobotRelativeChassisSpeeds(robotRelativeChassisSpeeds));
  }

  /**
   * Set field relative chassis speeds.
   *
   * @param fieldRelativeChassisSpeeds Field relative chassis speeds.
   */
  public void setFieldRelativeChassisSpeeds(ChassisSpeeds fieldRelativeChassisSpeeds) {
    setRobotRelativeChassisSpeeds(ChassisSpeeds.fromFieldRelativeSpeeds(
        fieldRelativeChassisSpeeds, new Rotation2d(getGyroAngle())));
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
    return new SwerveDriveKinematics(Arrays.stream(m_modules)
            .map(module -> module.getConfig().getLocation().orElseThrow())
            .toArray(Translation2d[] ::new));
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
    m_config.withGyroOffset(getGyroAngle().plus(m_config.getGyroOffset()));
    // If in sim reset to the simulated drive.
    //    resetOdometry(
    //        RobotBase.isSimulation() ? getMapleSimPose() : new Pose2d(getPose().getTranslation(),
    //        Rotation2d.kZero));
    resetOdometry(new Pose2d(getPose().getTranslation(), Rotation2d.kZero));
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
   * Resets odometry to the given pose. Gyro angle and module positions do not need to be reset when
   * calling this method. However, if either gyro angle or module position is reset, this must be
   * called in order for odometry to keep working.
   *
   * @param pose The pose to set the odometry to. Field relative, blue-origin where 0deg is facing
   *     towards RED alliance.
   */
  public void resetOdometry(Pose2d pose) {
    //    if (RobotBase.isSimulation() && m_config.getMapleDriveSim().isPresent())
    //    {
    //      m_config.getMapleDriveSim().get().resetOdometry(pose);
    //      m_config.getMapleDriveSim().get().setSimulationWorldPose(pose);
    //    }
    m_poseEstimator.resetPosition(new Rotation2d(getGyroAngle()), getModulePositions(), pose);
    m_desiredChassisSpeeds = new ChassisSpeeds();
    m_desiredModuleStates = m_kinematics.toSwerveModuleStates(new ChassisSpeeds());
    m_simPose = pose;
  }

  /**
   * Resets the auto-align rotational PID controller.
   */
  public void resetRotationPID() {
    m_config.getRotationPID().reset();
  }

  /**
   * Resets the auto-align translation PID controller.
   */
  public void resetTranslationPID() {
    m_config.getTranslationPID().reset();
  }

  /**
   * Set the auto-align rotational PID controller.
   * @param controller {@link PIDController} to use, units given is in Radians.
   */
  public void setRotationPID(PIDController controller) {
    var currentRotationPID = m_config.getRotationPID();
    if (currentRotationPID.getP() != controller.getP()
        || currentRotationPID.getI() != controller.getI()
        || currentRotationPID.getD() != controller.getD()) {
      controller.reset();
      m_config.withRotationController(controller);
    }
  }

  /**
   * Sets the auto-align translation PID controller.
   * @param controller {@link PIDController} to reset, Units given is in Meters.
   */
  public void setTranslationPID(PIDController controller) {
    var currentTranslationPID = m_config.getTranslationPID();
    if (currentTranslationPID.getP() != controller.getP()
        || currentTranslationPID.getI() != controller.getI()
        || currentTranslationPID.getD() != controller.getD()) {
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
   * Drive the robot to the given pose.
   *
   * @param pose {@link Pose2d} to drive the robot to. Field relative, blue-origin where 0deg is
   *     facing towards RED
   * @return {@link Command} to drive the robot to the given pose.
   * @implNote Not compatible with AdvantageKit
   */
  public Command driveToPose(Pose2d pose) {
    return Commands
        .runOnce(() -> {
          resetTranslationPID();
          resetRotationPID();
        })
        .andThen(
            () -> setFieldRelativeChassisSpeeds(driveToPoseSetpoint(pose)), m_config.getSubsystem())
        .withName("Drive to Pose");
  }

  /**
   * Drive to the target pose, primarily for use in Live Tuning, could also be used for setpoint
   * commands.
   * @param targetPose Pose to drive towards.
   * @implNote Remember to call {@link #resetRotationPID()} and {@link #resetTranslationPID()}
   * before calling this method in a loop.
   * @return field-relative {@link ChassisSpeeds} to drive the robot to the given pose.
   */
  public ChassisSpeeds driveToPoseSetpoint(Pose2d targetPose) {
    var rotationPID = m_config.getRotationPID();
    var translationPID = m_config.getTranslationPID();
    var distance = getDistanceFromPose(targetPose);
    var angleDifference = getAngleDifferenceFromPose(targetPose);
    var translationScalar = translationPID.calculate(distance.in(Meters), 0);
    var currentPose = getPose();
    var translationDifference = currentPose.getTranslation().minus(targetPose.getTranslation());
    return new ChassisSpeeds(
        translationDifference.getMeasureX().per(Second).times(translationScalar),
        translationDifference.getMeasureY().per(Second).times(translationScalar),
        RadiansPerSecond.of(rotationPID.calculate(currentPose.getRotation().getRadians(),
                                                  targetPose.getRotation().getRadians()))
    );
  }

  /**
   * Add a vision measurement to the {@link SwerveDrivePoseEstimator} and update the gyro reading
   * with the given timestamp of the vision measurement.
   *
   * @param robotPose                Robot {@link Pose2d} as measured by vision.
   * @param timestamp                Timestamp the measurement was taken as time since startup,
   *     should be taken from
   *                                 {@link Timer#getFPGATimestamp()} or similar sources.
   * @param visionMeasurementStdDevs Vision measurement standard deviation that will be sent to the
   *                                 {@link SwerveDrivePoseEstimator}.The standard deviation of the
   * vision measurement, for best accuracy calculate the standard deviation at 2 or more points and
   * fit a line to it with the calculated optimal standard deviation. (Units should be meters per
   * pixel). By optimizing this you can get * vision accurate to inches instead of feet.
   */
  public void addVisionMeasurement(
      Pose2d robotPose, double timestamp, Matrix<N3, N1> visionMeasurementStdDevs) {
    m_poseEstimator.addVisionMeasurement(robotPose, timestamp, visionMeasurementStdDevs);
  }

  /**
   * Sets the pose estimator's trust of global measurements. This might be used to change trust in
   * vision measurements after the autonomous period, or to change trust as distance to a vision
   * target increases.
   *
   * @param visionMeasurementStdDevs Standard deviations of the vision measurements. Increase these
   *     numbers to trust
   *                                 global measurements from vision less. This matrix is in the
   * form [x, y, theta], with units in meters and radians.
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
   *     from {@link Timer#getFPGATimestamp()} or similar sources.
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

  /**
   * Update the telemetry and {@link SwerveDrivePoseEstimator} of the drive.
   */
  public void updateTelemetry() {
    updatePoseEstimator();
    Pose2d robotPose = getPose();
    SwerveModuleState[] currentStates = getModuleStates();

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

  /**
   * Simulate the drive, updating the gyroscope based off of module states.
   */
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
    ChassisSpeeds desired = m_kinematics.toChassisSpeeds(m_desiredModuleStates);

    var dt = m_simTimer.get();
    Twist2d twist = new Twist2d(desired.vxMetersPerSecond * dt, desired.vyMetersPerSecond * dt,
        desired.omegaRadiansPerSecond * dt);
    m_simPose = m_simPose.exp(twist);
    m_simGyroAngle = m_simGyroAngle.plus(
        Radians.of(m_kinematics.toChassisSpeeds(getModuleStates()).omegaRadiansPerSecond * dt));
    m_simTimer.reset();
  }

  /**
   * Get the robot relative speed of the drive.
   *
   * @return Robot relative speed of the drive.
   */
  public ChassisSpeeds getRobotRelativeSpeed() {
    return m_kinematics.toChassisSpeeds(getModuleStates());
  }

  /**
   * Get the field relative speed of the drive.
   *
   * @return Field relative speed of the drive.
   */
  public ChassisSpeeds getFieldRelativeSpeed() {
    return ChassisSpeeds.fromRobotRelativeSpeeds(
        getRobotRelativeSpeed(), new Rotation2d(getGyroAngle()));
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
    return Arrays.stream(m_modules)
        .map(SwerveModule::getPosition)
        .toArray(SwerveModulePosition[] ::new);
  }

  /**
   * Get the {@link SwerveModuleState} of the modules.
   *
   * @return {@link SwerveModuleState} of the modules.
   */
  public SwerveModuleState[] getModuleStates() {
    // If MapleSim is configured, return the simulated states.
    //    if (RobotBase.isSimulation() && m_config.getMapleDriveSim().isPresent())
    //    {
    //      return m_config.getMapleDriveSim().get().getMeasuredStates();
    //    }
    return Arrays.stream(m_modules).map(SwerveModule::getState).toArray(SwerveModuleState[] ::new);
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
   * can publish additional {@link edu.wpi.first.wpilibj.smartdashboard.FieldObject2d}s onto the
   * same field widget instead of creating their own.
   *
   * @return {@link Field2d} of the drive.
   */
  public Field2d getField2d() {
    return m_field2d;
  }

  /**
   * Get the last-commanded desired robot-relative {@link ChassisSpeeds} of the drive. This is the
   * value cached by {@link #setRobotRelativeChassisSpeeds(ChassisSpeeds)} (which
   * {@link #setFieldRelativeChassisSpeeds(ChassisSpeeds)} and
   * {@link #drive(java.util.function.Supplier)} funnel through) and published on every
   * {@link #updateTelemetry()} call.
   * @implNote It is a setpoint, not a measurement of actual robot motion.
   * Use {@link #getRobotRelativeSpeed()} or {@link #getFieldRelativeSpeed()} instead if you need
   * the drive's actual measured speed.
   *
   * @return Robot-relative {@link ChassisSpeeds} last commanded to the drive. Defaults to a
   *         zeroed {@link ChassisSpeeds} if the drive has never been commanded.
   */
  public ChassisSpeeds getDesiredChassisSpeeds() {
    return m_desiredChassisSpeeds;
  }

  /**
   * Get the last-commanded desired {@link SwerveModuleState}s of the drive.
   * @return {@link SwerveModuleState}s last commanded to the drive. Defaults to a zeroed {@link
   *     SwerveModuleState}s if the drive has never been commanded.
   */
  public SwerveModuleState[] getDesiredModuleStates() {
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
   * @return Array of {@link SwerveModule}s.
   */
  public SwerveModule[] getModules() {
    return m_modules;
  }
}
