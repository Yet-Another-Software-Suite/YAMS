// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package first.robot.subsystems;

import static first.robot.Constants.SkidConstants.*;
import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.DegreesPerSecond;
import static org.wpilib.units.Units.Inches;
import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.MetersPerSecond;
import static org.wpilib.units.Units.RadiansPerSecond;
import static org.wpilib.units.Units.Rotations;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import first.robot.SkidDetector;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.wpilib.command2.Command;
import org.wpilib.command2.SubsystemBase;
import org.wpilib.framework.RobotBase;
import org.wpilib.hardware.bus.CANPort;
import org.wpilib.math.controller.PIDController;
import org.wpilib.math.controller.SimpleMotorFeedforward;
import org.wpilib.math.estimator.SwerveDrivePoseEstimator;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.math.kinematics.SwerveDriveKinematics;
import org.wpilib.math.kinematics.SwerveModulePosition;
import org.wpilib.math.system.DCMotor;
import org.wpilib.networktables.BooleanPublisher;
import org.wpilib.networktables.DoublePublisher;
import org.wpilib.networktables.IntegerPublisher;
import org.wpilib.networktables.NetworkTable;
import org.wpilib.networktables.NetworkTableInstance;
import org.wpilib.smartdashboard.Field2d;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.AngularVelocity;
import yams.commands2.config.SmartMotorControllerConfig;
import yams.commands2.config.SwerveDriveConfig;
import yams.commands2.swerve.SwerveDrive;
import yams.commands2.swerve.SwerveInputStream;
import yams.core.gearing.MechanismGearing;
import yams.core.mechanisms.config.SwerveModuleConfig;
import yams.core.mechanisms.swerve.SwerveModule;
import yams.core.motorcontrollers.SmartMotorController;
import yams.core.motorcontrollers.local.SparkWrapper;
import yams.core.telemetry.enums.TelemetryVerbosity;

/** YAMS SwerveDrive paired with a standalone gyro-referenced SkidDetector and filtered PoseEstimator. */
public class SwerveSubsystem extends SubsystemBase {
  private static final Pose2d STARTING_POSE = new Pose2d(Meters.of(2), Meters.of(2), Rotation2d.ZERO);

  private final SwerveDrive drive;
  private final Pigeon2 gyro = new Pigeon2(14, new CANBus(CANPort.CAN_S0));
  private final SkidDetector skidDetector =
      new SkidDetector(MODULE_LOCATIONS, SKID_RATIO_THRESHOLD, MIN_MODULE_TRANSLATIONAL_SPEED);
  private final SwerveDrivePoseEstimator filteredPoseEstimator;
  private SwerveModulePosition[] lastRawPositions;
  private SwerveModulePosition[] filteredPositions;
  private SkidDetector.Result lastSkidResult = SkidDetector.Result.noSkid(MODULE_LOCATIONS.length);

  private final DoublePublisher ratioPub;
  private final BooleanPublisher isSkiddingPub;
  private final IntegerPublisher slippingCountPub;

  private SmartMotorControllerConfig buildDriveCfg() {
    return new SmartMotorControllerConfig(this)
        .withWheelDiameter(Inches.of(4))
        .withClosedLoopController(0.3, 0, 0)
        .withGearing(new MechanismGearing(6.75))
        .withFeedforward(new SimpleMotorFeedforward(0, 0.85))
        .withMotorInverted(false)
        .withStatorCurrentLimit(Amps.of(40))
        .withTelemetry("driveMotor", TelemetryVerbosity.HIGH);
  }

  private SmartMotorControllerConfig buildAzimuthCfg() {
    return new SmartMotorControllerConfig(this)
        .withClosedLoopController(4, 0, 0)
        .withContinuousWrapping(Rotations.of(-0.5), Rotations.of(0.5))
        .withGearing(new MechanismGearing(12.8))
        .withMotorInverted(false)
        .withStatorCurrentLimit(Amps.of(20))
        .withTelemetry("angleMotor", TelemetryVerbosity.HIGH);
  }

  private SwerveModule createModule(
      SparkMax driveMotorHw,
      SparkMax azimuthMotorHw,
      CANcoder absoluteEncoder,
      Angle encoderOffset,
      String moduleName,
      Translation2d location) {
    SmartMotorController driveMotor =
        new SparkWrapper(driveMotorHw, DCMotor.getNEO(1), buildDriveCfg());
    SmartMotorController azimuthMotor =
        new SparkWrapper(azimuthMotorHw, DCMotor.getNEO(1), buildAzimuthCfg());

    return new SwerveModule(
        new SwerveModuleConfig(driveMotor, azimuthMotor)
            .withAbsoluteEncoder(absoluteEncoder.getAbsolutePosition().asSupplier())
            .withAbsoluteEncoderOffset(encoderOffset)
            .withTelemetry(moduleName, TelemetryVerbosity.HIGH)
            .withLocation(location)
            .withOptimization(true));
  }

  public SwerveSubsystem() {
    var frontLeft = createModule(
        new SparkMax(CANPort.CAN_S0, 1, MotorType.kBrushless),
        new SparkMax(CANPort.CAN_S0, 2, MotorType.kBrushless),
        new CANcoder(3, new CANBus(CANPort.CAN_S0)),
        Degrees.of(0), "frontleft", MODULE_LOCATIONS[0]);
    var frontRight = createModule(
        new SparkMax(CANPort.CAN_S0, 4, MotorType.kBrushless),
        new SparkMax(CANPort.CAN_S0, 5, MotorType.kBrushless),
        new CANcoder(6, new CANBus(CANPort.CAN_S0)),
        Degrees.of(0), "frontright", MODULE_LOCATIONS[1]);
    var backLeft = createModule(
        new SparkMax(CANPort.CAN_S0, 7, MotorType.kBrushless),
        new SparkMax(CANPort.CAN_S0, 8, MotorType.kBrushless),
        new CANcoder(9, new CANBus(CANPort.CAN_S0)),
        Degrees.of(0), "backleft", MODULE_LOCATIONS[2]);
    var backRight = createModule(
        new SparkMax(CANPort.CAN_S0, 10, MotorType.kBrushless),
        new SparkMax(CANPort.CAN_S0, 11, MotorType.kBrushless),
        new CANcoder(12, new CANBus(CANPort.CAN_S0)),
        Degrees.of(0), "backright", MODULE_LOCATIONS[3]);

    SwerveDriveConfig config =
        new SwerveDriveConfig(this, frontLeft, frontRight, backLeft, backRight)
            .withGyro(gyro::getRotation3d)
            .withStartingPose(STARTING_POSE)
            .withTranslationController(new PIDController(1, 0, 0))
            .withRotationController(new PIDController(5, 0, 0))
            .withTelemetry("swerve", TelemetryVerbosity.HIGH);

    drive = new SwerveDrive(config);
    SwerveModulePosition[] initialPositions = drive.getModulePositions();
    lastRawPositions = copyPositions(initialPositions);
    filteredPositions = copyPositions(initialPositions);
    filteredPoseEstimator =
        new SwerveDrivePoseEstimator(
            new SwerveDriveKinematics(MODULE_LOCATIONS),
            drive.getRotation(),
            filteredPositions,
            STARTING_POSE);

    NetworkTable table = NetworkTableInstance.getDefault().getTable("SkidDetection");
    ratioPub = table.getDoubleTopic("Ratio").publish();
    isSkiddingPub = table.getBooleanTopic("IsSkidding").publish();
    slippingCountPub = table.getIntegerTopic("SlippingModuleCount").publish();
  }

  public Command driveFieldRelative(Supplier<ChassisVelocities> speedsSupplier) {
    return run(() -> drive.setFieldRelativeChassisSpeeds(speedsSupplier.get()))
        .withName("Field Oriented Drive");
  }

  public Command zeroGyro() {
    return runOnce(drive::zeroGyro).withName("Zero Gyro");
  }

  public SwerveInputStream getChassisSpeedsSupplier(
      DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier rotation) {
    return SwerveInputStream.of(drive, translationX, translationY)
        .withControllerRotationAxis(rotation)
        .withMaximumLinearVelocity(MetersPerSecond.of(2))
        .withMaximumAngularVelocity(DegreesPerSecond.of(360))
        .withDeadband(0.05)
        .withCubeTranslationControllerAxis()
        .withCubeRotationControllerAxis()
        .withTelemetry("Driver", TelemetryVerbosity.HIGH);
  }

  public Pose2d getFilteredPose() {
    return filteredPoseEstimator.getEstimatedPosition();
  }

  public SkidDetector.Result getLastSkidResult() {
    return lastSkidResult;
  }

  public Field2d getField2d() {
    return drive.getField2d();
  }

  private AngularVelocity getGyroYawRate() {
    if (RobotBase.isSimulation()) {
      return RadiansPerSecond.of(drive.getFieldRelativeSpeed().omega);
    }
    return gyro.getAngularVelocityZWorld().getValue();
  }

  private void updateFilteredOdometry() {
    lastSkidResult = skidDetector.detect(drive.getModuleStates(), getGyroYawRate());
    SwerveModulePosition[] rawPositions = drive.getModulePositions();
    for (int i = 0; i < rawPositions.length; i++) {
      double deltaMeters = rawPositions[i].distance - lastRawPositions[i].distance;
      double nextDistance =
          lastSkidResult.isModuleSlipping(i)
              ? filteredPositions[i].distance
              : filteredPositions[i].distance + deltaMeters;
      filteredPositions[i] = new SwerveModulePosition(nextDistance, rawPositions[i].angle);
      lastRawPositions[i] = new SwerveModulePosition(rawPositions[i].distance, rawPositions[i].angle);
    }
    filteredPoseEstimator.update(drive.getRotation(), filteredPositions);

    ratioPub.set(lastSkidResult.ratio());
    isSkiddingPub.set(lastSkidResult.isSkidding());
    slippingCountPub.set(lastSkidResult.slippingModuleCount());
    drive.getField2d().getObject("Filtered Pose").setPose(getFilteredPose());
  }

  private static SwerveModulePosition[] copyPositions(SwerveModulePosition[] src) {
    SwerveModulePosition[] out = new SwerveModulePosition[src.length];
    for (int i = 0; i < src.length; i++) {
      out[i] = new SwerveModulePosition(src[i].distance, src[i].angle);
    }
    return out;
  }

  @Override
  public void periodic() {
    drive.updateTelemetry();
    updateFilteredOdometry();
  }

  @Override
  public void simulationPeriodic() {
    drive.simIterate();
    gyro.getSimState().setRawYaw(drive.getSimPose().getRotation().getMeasure());
  }
}
