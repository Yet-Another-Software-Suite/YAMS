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

public class SwerveSubsystem extends SubsystemBase
{

  private AngularVelocity          maximumChassisSpeedsAngularVelocity = DegreesPerSecond.of(720);
  private LinearVelocity           maximumChassisSpeedsLinearVelocity  = MetersPerSecond.of(4);
  private SwerveDrivePoseEstimator visionPoseEstimator;
  private Supplier<Angle>          gyroAngleSupplier;
  private SwerveDriveConfig        config;

  /**
   * AdvantageKit identifies inputs via the "Replay Bubble". Everything going to the SMC is an Output. Everything coming
   * from the SMC is an Input.
   */
  @AutoLog
  public static class SwerveInputs
  {

    public SwerveModulePosition[] positions           = new SwerveModulePosition[4];
    public SwerveModuleState[]    states              = new SwerveModuleState[4];
    public Angle                  gyroRotation        = Degrees.of(0);
    public ChassisSpeeds          robotRelativeSpeeds = new ChassisSpeeds(0, 0, 0);
    public Pose2d                 estimatedPose       = new Pose2d(0, 0, Rotation2d.fromDegrees(0));
  }

  private final SwerveInputsAutoLogged swerveInputs = new SwerveInputsAutoLogged();

  private final SwerveDrive drive;
  private final Field2d     field = new Field2d();

  public SwerveModule createModule(SparkMax drive, SparkMax azimuth, CANcoder absoluteEncoder, String moduleName,
                                   Translation2d location)
  {
    MechanismGearing driveGearing   = new MechanismGearing(GearBox.fromStages("12:1", "2:1"));
    MechanismGearing azimuthGearing = new MechanismGearing(GearBox.fromStages("21:1"));
    SmartMotorControllerConfig driveCfg = new SmartMotorControllerConfig(this)
        .withWheelDiameter(Inches.of(4))
        .withClosedLoopController(50, 0, 4)
        .withGearing(driveGearing)
        .withStatorCurrentLimit(Amps.of(40))
        .withTelemetry("driveMotor", SmartMotorControllerConfig.TelemetryVerbosity.HIGH);
    SmartMotorControllerConfig azimuthCfg = new SmartMotorControllerConfig(this)
        .withClosedLoopController(50, 0, 4)
        .withContinuousWrapping(Radians.of(-Math.PI), Radians.of(Math.PI))
        .withGearing(azimuthGearing)
        .withStatorCurrentLimit(Amps.of(20))
        .withTelemetry("angleMotor", SmartMotorControllerConfig.TelemetryVerbosity.HIGH);
    SmartMotorController driveSMC   = new SparkWrapper(drive, DCMotor.getNEO(1), driveCfg);
    SmartMotorController azimuthSMC = new SparkWrapper(azimuth, DCMotor.getNEO(1), azimuthCfg);
    SwerveModuleConfig moduleConfig = new SwerveModuleConfig(driveSMC, azimuthSMC)
        .withAbsoluteEncoder(absoluteEncoder.getAbsolutePosition().asSupplier())
        .withTelemetry(moduleName, SmartMotorControllerConfig.TelemetryVerbosity.HIGH)
        .withLocation(location)
        .withOptimization(true);
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
        .withDeadband(0.01)
        .withCubeRotationControllerAxis()
        .withCubeTranslationControllerAxis()
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
    Pigeon2 gyro = new Pigeon2(14);
    gyroAngleSupplier = gyro.getYaw().asSupplier();

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
        .withStartingPose(swerveInputs.estimatedPose)
        .withTranslationController(new PIDController(1, 0, 0))
        .withRotationController(new PIDController(1, 0, 0));
    drive = new SwerveDrive(config);

    visionPoseEstimator = new SwerveDrivePoseEstimator(drive.getKinematics(),
                                                       getGyroAngle(),
                                                       drive.getModulePositions(),
                                                       swerveInputs.estimatedPose);
    SmartDashboard.putData("Field", field);
  }


  private Rotation2d getGyroAngle()
  {
    return new Rotation2d(swerveInputs.gyroRotation);
  }

  private void updateInputs()
  {
    swerveInputs.estimatedPose = drive.getPose();
    swerveInputs.states = drive.getModuleStates();
    swerveInputs.positions = drive.getModulePositions();
    swerveInputs.robotRelativeSpeeds = drive.getRobotRelativeSpeed();
    swerveInputs.gyroRotation = gyroAngleSupplier.get();
  }

  public Command setRobotRelativeChassisSpeeds(ChassisSpeeds speeds)
  {
    return run(() -> {
      Logger.recordOutput("Swerve/DesiredChassisSpeeds", speeds);
      Logger.recordOutput("Swerve/DesiredOptimizedChassisSpeeds", config.optimizeRobotRelativeChassisSpeeds(speeds));
      SwerveModuleState[] states = drive.getStateFromRobotRelativeChassisSpeeds(speeds);
      Logger.recordOutput("Swerve/DesiredStates", states);
      drive.setSwerveModuleStates(states);
    }).withName("Set Robot Relative Chassis Speeds");
  }

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
      var currentPose       = getPose();
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

  public Command lock()
  {
    return run(() -> {
      ChassisSpeeds speeds = new ChassisSpeeds();
      Logger.recordOutput("Swerve/DesiredChassisSpeeds", speeds);
      Logger.recordOutput("Swerve/DesiredOptimizedChassisSpeeds", speeds);
      // Sets states
      SwerveModule[]      modules       = config.getModules();
      SwerveModuleState[] desiredStates = new SwerveModuleState[modules.length];
      for (int i = 0; i < modules.length; i++)
      {
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
    return swerveInputs.estimatedPose;
  }

  public ChassisSpeeds getRobotRelativeSpeeds()
  {
    return swerveInputs.robotRelativeSpeeds;
  }

  @Override
  public void periodic()
  {
    drive.updateTelemetry(); // Updates the pose estimator, must be called before processInputs
    updateInputs();
    Logger.processInputs("Swerve", swerveInputs);
    field.setRobotPose(getPose());
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

