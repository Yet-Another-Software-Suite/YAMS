// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package frc.robot.subsystems;


import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Second;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.SwerveDriveConfig;
import yams.mechanisms.config.SwerveModuleConfig;
import yams.mechanisms.swerve.SwerveDrive;
import yams.mechanisms.swerve.SwerveModule;
import yams.mechanisms.swerve.utility.SwerveInputStream;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.local.SparkWrapper;

/**
 * Four-module NEO swerve drive with Pigeon 2 gyro and AdvantageKit input logging.
 * Module positions, states, gyro angle, robot-relative speeds, and estimated pose
 * all cross the replay boundary via SwerveInputs. A second SwerveDrivePoseEstimator
 * runs in parallel for vision fusion -- its output is a computed value and is NOT
 * part of the replay inputs.
 */
public class SwerveSubsystem extends SubsystemBase
{
  // 720 deg/s is roughly 2 full rotations/s; comfortable for driver control
  // without making the robot feel twitchy.
  private AngularVelocity          maximumChassisSpeedsAngularVelocity = DegreesPerSecond.of(720);
  // 4 m/s is near the NEO free-speed limit for this wheel/gearing combination.
  private LinearVelocity           maximumChassisSpeedsLinearVelocity  = MetersPerSecond.of(4);
  private SwerveDrivePoseEstimator visionPoseEstimator;
  private Supplier<Angle>          gyroAngleSupplier;
  private SwerveDriveConfig        config;

  /*
   * SwerveInputs is the replay boundary for the drivetrain. @AutoLog generates
   * SwerveInputsAutoLogged so Logger.processInputs() can stamp all fields with a
   * single consistent timestamp.
   *
   * Gyro angle is logged here because it is hardware-sourced; all pose math runs
   * on the logged angle so replay is deterministic even if the Pigeon 2 firmware
   * or CAN bus timing differs on a replay machine.
   *
   * estimatedPose is included so getPose() returns the replayed pose -- critical
   * for any command that gates on robot position (driveToPose, auto paths).
   */
  @AutoLog
  public static class SwerveInputs
  {
    // Module positions (distance + heading) from each encoder; needed to tick
    // the odometry integrator during replay.
    public SwerveModulePosition[] positions           = new SwerveModulePosition[4];
    public SwerveModuleState[]    states              = new SwerveModuleState[4];
    // Raw Pigeon 2 yaw; logged before being consumed by odometry so replay sees
    // the exact hardware value, not a processed one.
    public Angle                  gyroRotation        = Degrees.of(0);
    public ChassisSpeeds          robotRelativeSpeeds = new ChassisSpeeds(0, 0, 0);
    // Pose from the SwerveDrive's internal odometry; logged here so getPose()
    // is consistent in replay without re-running the kinematics integrator.
    public Pose2d                 estimatedPose       = new Pose2d(0, 0, Rotation2d.fromDegrees(0));
  }

  private final SwerveInputsAutoLogged swerveInputs = new SwerveInputsAutoLogged();

  private final SwerveDrive drive;
  private final Field2d     field = new Field2d();

  /**
   * Builds one swerve module from a drive motor, azimuth motor, CANcoder, and
   * physical location on the robot frame.
   *
   * @param drive           Drive SparkMax
   * @param azimuth         Azimuth SparkMax
   * @param absoluteEncoder CANcoder for absolute azimuth position
   * @param moduleName      Telemetry prefix (e.g. "frontleft")
   * @param location        Module location relative to robot center
   */
  public SwerveModule createModule(SparkMax drive, SparkMax azimuth, CANcoder absoluteEncoder, String moduleName,
                                   Translation2d location)
  {
    // Drive gearing: 12:1 first stage, 2:1 second stage = 24:1 total.
    // Higher reduction gives more torque; lower gives more top speed.
    MechanismGearing driveGearing   = new MechanismGearing(GearBox.fromStages("12:1", "2:1"));
    // 21:1 azimuth reduction keeps the turning motor well within its torque band
    // while still allowing full 180 deg azimuth reversal in under 0.4 seconds.
    MechanismGearing azimuthGearing = new MechanismGearing(GearBox.fromStages("21:1"));

    SmartMotorControllerConfig driveCfg = new SmartMotorControllerConfig(this)
        .withWheelDiameter(Inches.of(4))
        // kP=50 on drive gives crisp velocity tracking; kD=4 dampens oscillation
        // at high speeds where back-EMF changes quickly.
        .withClosedLoopController(50, 0, 4)
        .withGearing(driveGearing)
        // 40 A stator limit prevents the drive NEO from derating during hard pushes.
        .withStatorCurrentLimit(Amps.of(40))
        .withTelemetry("driveMotor", SmartMotorControllerConfig.TelemetryVerbosity.HIGH);

    SmartMotorControllerConfig azimuthCfg = new SmartMotorControllerConfig(this)
        // Same kP/kD as drive; wrapping from -pi to pi means the controller never
        // takes the long way around past 180 deg.
        .withClosedLoopController(50, 0, 4)
        .withContinuousWrapping(Radians.of(-Math.PI), Radians.of(Math.PI))
        .withGearing(azimuthGearing)
        // 20 A for azimuth -- lighter load than drive and current limits help
        // prevent the motor from fighting the absolute encoder on power-up sync.
        .withStatorCurrentLimit(Amps.of(20))
        .withTelemetry("angleMotor", SmartMotorControllerConfig.TelemetryVerbosity.HIGH);

    SmartMotorController driveSMC   = new SparkWrapper(drive, DCMotor.getNEO(1), driveCfg);
    SmartMotorController azimuthSMC = new SparkWrapper(azimuth, DCMotor.getNEO(1), azimuthCfg);

    SwerveModuleConfig moduleConfig = new SwerveModuleConfig(driveSMC, azimuthSMC)
        .withAbsoluteEncoder(absoluteEncoder.getAbsolutePosition().asSupplier())
        .withTelemetry(moduleName, SmartMotorControllerConfig.TelemetryVerbosity.HIGH)
        .withLocation(location)
        .withOptimization(true); // Flip drive direction rather than rotating azimuth >90 deg.

    return new SwerveModule(moduleConfig);
  }

  /**
   * Get a {@link Supplier<ChassisSpeeds>} for the robot relative chassis speeds based on "standard" swerve drive
   * controls.
   *
   * @param translationXScalar Translation in the X direction from [-1,1]
   * @param translationYScalar Translation in the Y direction from [-1,1]
   * @param rotationScalar     Rotation speed from [-1,1]
   * @return {@link Supplier<ChassisSpeeds>} for the robot relative chassis speeds.
   */
  public SwerveInputStream getChassisSpeedsSupplier(DoubleSupplier translationXScalar,
                                                    DoubleSupplier translationYScalar,
                                                    DoubleSupplier rotationScalar)
  {
    return new SwerveInputStream(drive, translationXScalar, translationYScalar, rotationScalar)
        .withMaximumAngularVelocity(maximumChassisSpeedsAngularVelocity)
        .withMaximumLinearVelocity(maximumChassisSpeedsLinearVelocity)
        // 0.01 deadband eliminates stick drift without adding noticeable dead zone.
        .withDeadband(0.01)
        // Cubing the rotation axis gives finer control at low inputs without
        // reducing the achievable maximum.
        .withCubeRotationControllerAxis()
        .withCubeTranslationControllerAxis()
        // Alliance-relative: forward on the stick always moves toward the opposing
        // alliance wall regardless of which side the robot started on.
        .withAllianceRelativeControl();
  }

  /**
   * Get a {@link Supplier<ChassisSpeeds>} for the robot relative chassis speeds based on "standard" swerve drive
   * controls.
   *
   * @param translationXScalar Translation in the X direction from [-1,1]
   * @param translationYScalar Translation in the Y direction from [-1,1]
   * @param rotationScalar     Rotation speed from [-1,1]
   * @return {@link Supplier<ChassisSpeeds>} for the robot relative chassis speeds.
   */
  public Supplier<ChassisSpeeds> getSimpleChassisSpeeds(DoubleSupplier translationXScalar,
                                                        DoubleSupplier translationYScalar,
                                                        DoubleSupplier rotationScalar)
  {
    return () -> new ChassisSpeeds(maximumChassisSpeedsLinearVelocity.times(translationXScalar.getAsDouble())
                                                                     .in(MetersPerSecond),
                                   maximumChassisSpeedsLinearVelocity.times(translationYScalar.getAsDouble())
                                                                     .in(MetersPerSecond),
                                   maximumChassisSpeedsAngularVelocity.times(rotationScalar.getAsDouble())
                                                                      .in(RadiansPerSecond));
  }

  public SwerveSubsystem()
  {
    // Pigeon 2 on CAN ID 14; yaw supplier is read in updateInputs() each loop.
    Pigeon2 gyro = new Pigeon2(14);
    gyroAngleSupplier = gyro.getYaw().asSupplier();

    // Module locations are 24 in from center along each axis (48 in x 48 in base).
    var fl = createModule(new SparkMax(1, MotorType.kBrushless),
                          new SparkMax(2, MotorType.kBrushless),
                          new CANcoder(3),
                          "frontleft",
                          new Translation2d(Inches.of(24), Inches.of(24)));
    var fr = createModule(new SparkMax(4, MotorType.kBrushless),
                          new SparkMax(5, MotorType.kBrushless),
                          new CANcoder(6),
                          "frontright",
                          new Translation2d(Inches.of(24), Inches.of(-24)));
    var bl = createModule(new SparkMax(7, MotorType.kBrushless),
                          new SparkMax(8, MotorType.kBrushless),
                          new CANcoder(9),
                          "backleft",
                          new Translation2d(Inches.of(-24), Inches.of(24)));
    var br = createModule(new SparkMax(10, MotorType.kBrushless),
                          new SparkMax(11, MotorType.kBrushless),
                          new CANcoder(12),
                          "backright",
                          new Translation2d(Inches.of(-24), Inches.of(-24)));

    config = new SwerveDriveConfig(this, fl, fr, bl, br)
        .withGyro(() -> getGyroAngle().getMeasure())
        // Use the logged estimated pose as the starting pose so the drive's
        // internal odometry initialises from the replayed value, not from zero.
        .withStartingPose(swerveInputs.estimatedPose)
        // kP=1 translation and rotation PIDs are starters for driveToPose();
        // increase if the robot undershoots at approach speed.
        .withTranslationController(new PIDController(1, 0, 0))
        .withRotationController(new PIDController(1, 0, 0));
    drive = new SwerveDrive(config);

    // Second pose estimator for vision fusion. Its output is a computed value
    // (NOT in SwerveInputs) so it is recomputed from scratch during replay.
    visionPoseEstimator = new SwerveDrivePoseEstimator(drive.getKinematics(),
                                                       getGyroAngle(),
                                                       drive.getModulePositions(),
                                                       swerveInputs.estimatedPose);
    SmartDashboard.putData("Field", field);
  }


  private Rotation2d getGyroAngle()
  {
    // Reads from swerveInputs so replay uses the logged yaw, not live hardware.
    return new Rotation2d(swerveInputs.gyroRotation);
  }

  /**
   * Populate SwerveInputs from the SwerveDrive and gyro. Must run before
   * Logger.processInputs() to ensure all fields share the same log timestamp.
   */
  private void updateInputs()
  {
    swerveInputs.estimatedPose = drive.getPose();
    swerveInputs.states = drive.getModuleStates();
    swerveInputs.positions = drive.getModulePositions();
    swerveInputs.robotRelativeSpeeds = drive.getRobotRelativeSpeed();
    // Read gyro last so the logged angle matches the positions captured above.
    swerveInputs.gyroRotation = gyroAngleSupplier.get();
  }

  public Command setRobotRelativeChassisSpeeds(ChassisSpeeds speeds)
  {
    return run(() -> {
      // DesiredChassisSpeeds and DesiredStates are computed outputs -- they are
      // recomputed each replay loop from the same command logic, not stored.
      Logger.recordOutput("Swerve/DesiredChassisSpeeds", speeds);
      Logger.recordOutput("Swerve/DesiredOptimizedChassisSpeeds", config.optimizeRobotRelativeChassisSpeeds(speeds));
      SwerveModuleState[] states = drive.getStateFromRobotRelativeChassisSpeeds(speeds);
      Logger.recordOutput("Swerve/DesiredStates", states);
      drive.setSwerveModuleStates(states);
    }).withName("Set Robot Relative Chassis Speeds");
  }

  /**
   * PID-drive to a field-relative pose. Translation and rotation errors are
   * computed from the logged estimatedPose so the path taken during replay
   * matches the real match exactly.
   */
  public Command driveToPose(Pose2d pose)
  {
    return startRun(() -> {
      drive.resetTranslationPID();
      drive.resetAzimuthPID();
    }, () -> {
      var azimuthPID        = config.getRotationPID();
      var translationPID    = config.getTranslationPID();
      var distance          = drive.getDistanceFromPose(pose);
      var angleDifference   = drive.getAngleDifferenceFromPose(pose);
      var translationScalar = translationPID.calculate(distance.in(Meters), 0);
      var currentPose       = getPose(); // Returns replayed pose during log replay.
      var poseDifference    = currentPose.minus(pose);
      setRobotRelativeChassisSpeeds(ChassisSpeeds.fromFieldRelativeSpeeds(poseDifference.getMeasureX().per(Second)
                                                                                        .times(translationScalar),
                                                                          poseDifference.getMeasureY().per(Second)
                                                                                        .times(translationScalar),
                                                                          RadiansPerSecond.of(azimuthPID.calculate(
                                                                              currentPose.getRotation()
                                                                                         .getRadians(),
                                                                              pose.getRotation()
                                                                                  .getRadians())),
                                                                          getGyroAngle()));
    }).withName("Drive to Pose");
  }

  public Command setRobotRelativeChassisSpeeds(Supplier<ChassisSpeeds> speedsSupplier)
  {
    return run(() -> {
      Logger.recordOutput("Swerve/DesiredChassisSpeeds", speedsSupplier.get());
      Logger.recordOutput("Swerve/DesiredOptimizedChassisSpeeds",
                          config.optimizeRobotRelativeChassisSpeeds(speedsSupplier.get()));
      SwerveModuleState[] states = drive.getStateFromRobotRelativeChassisSpeeds(speedsSupplier.get());
      Logger.recordOutput("Swerve/DesiredStates", states);
      drive.setSwerveModuleStates(states);
    }).withName("Set Robot Relative Chassis Speeds Supplier");
  }

  /**
   * Point all modules to their corner angles (X-pattern) with zero drive speed.
   * Wheels resist pushing forces without braking the drive motors.
   */
  public Command lock()
  {
    return run(() -> {
      ChassisSpeeds speeds = new ChassisSpeeds();
      Logger.recordOutput("Swerve/DesiredChassisSpeeds", speeds);
      Logger.recordOutput("Swerve/DesiredOptimizedChassisSpeeds", speeds);
      SwerveModule[]      modules       = config.getModules();
      SwerveModuleState[] desiredStates = new SwerveModuleState[modules.length];
      for (int i = 0; i < modules.length; i++)
      {
        // Each module points to its own corner: getAngle() returns the vector from
        // robot center to that module, which forms an X when all four are set.
        desiredStates[i] =
            new SwerveModuleState(0, modules[i].getConfig().getLocation().orElseThrow().getAngle());
      }
      Logger.recordOutput("Swerve/DesiredStates", desiredStates);
      drive.setSwerveModuleStates(desiredStates);
    }).withName("Lock");
  }

  public void resetOdometry(Pose2d pose)
  {
    drive.resetOdometry(pose);
    visionPoseEstimator.resetPose(pose);
  }

  public Pose2d getPose()
  {
    // Reads from swerveInputs so replay returns the logged pose, making all
    // pose-gated logic deterministic without re-running odometry from scratch.
    return swerveInputs.estimatedPose;
  }

  public ChassisSpeeds getRobotRelativeSpeeds()
  {
    return swerveInputs.robotRelativeSpeeds;
  }

  @Override
  public void periodic()
  {
    // updateTelemetry() ticks the internal pose estimator -- must run before
    // updateInputs() captures the fresh estimatedPose into swerveInputs.
    drive.updateTelemetry();
    updateInputs();
    // processInputs stamps SwerveInputs and closes the replay bubble. After this
    // call getGyroAngle() and getPose() return replayed values in replay mode.
    Logger.processInputs("Swerve", swerveInputs);
    field.setRobotPose(getPose());
    // Vision estimator updates are computed outputs; they do NOT enter the replay
    // bubble. The fused pose is re-derived from the replayed module positions and
    // gyro angle, so vision accuracy can be tuned offline.
    visionPoseEstimator.update(getGyroAngle(), swerveInputs.positions);
    field.getObject("VisionPose").setPose(visionPoseEstimator.getEstimatedPosition());
    Logger.recordOutput("Swerve/VisionPose", visionPoseEstimator.getEstimatedPosition());
    // TODO: Add vision stuff here
  }

  @Override
  public void simulationPeriodic()
  {
    drive.simIterate();
  }
}
