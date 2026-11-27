// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package first.robot.mechanisms;

import static first.robot.Constants.TiltConstants.*;
import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.DegreesPerSecond;
import static org.wpilib.units.Units.Inches;
import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.MetersPerSecond;
import static org.wpilib.units.Units.Rotations;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import first.robot.TiltOdometryScaler;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;
import org.wpilib.hardware.bus.CANPort;
import org.wpilib.math.controller.PIDController;
import org.wpilib.math.controller.SimpleMotorFeedforward;
import org.wpilib.math.estimator.SwerveDrivePoseEstimator;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.math.kinematics.SwerveDriveKinematics;
import org.wpilib.math.kinematics.SwerveModulePosition;
import org.wpilib.math.system.DCMotor;
import org.wpilib.networktables.BooleanPublisher;
import org.wpilib.networktables.DoublePublisher;
import org.wpilib.networktables.NetworkTable;
import org.wpilib.networktables.NetworkTableInstance;
import org.wpilib.smartdashboard.Field2d;
import org.wpilib.units.measure.Angle;
import yams.commands3.config.SmartMotorControllerConfig;
import yams.commands3.config.SwerveDriveConfig;
import yams.commands3.swerve.SwerveDrive;
import yams.commands3.swerve.SwerveInputStream;
import yams.core.gearing.MechanismGearing;
import yams.core.mechanisms.config.SwerveModuleConfig;
import yams.core.mechanisms.swerve.SwerveModule;
import yams.core.motorcontrollers.SmartMotorController;
import yams.core.motorcontrollers.local.SparkWrapper;
import yams.core.telemetry.enums.TelemetryVerbosity;

/** YAMS SwerveDrive paired with a standalone TiltOdometryScaler and vision tilt gate. */
public class SwerveMechanism implements Mechanism {
  private static final Pose2d STARTING_POSE = new Pose2d(Meters.of(2), Meters.of(2), Rotation2d.ZERO);

  private final SwerveDrive drive;
  private final Pigeon2 gyro = new Pigeon2(14, new CANBus(CANPort.CAN_S0));
  private final TiltOdometryScaler tiltScaler =
      new TiltOdometryScaler(SCALING_MODE, FULL_TRUST_TILT, ZERO_TRUST_TILT, MAX_VISION_TILT);
  private final SwerveDrivePoseEstimator tiltPoseEstimator;

  private final DoublePublisher tiltDegPub;
  private final DoublePublisher scalePub;
  private final BooleanPublisher visionAllowedPub;

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

  public SwerveMechanism() {
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
    SwerveModulePosition[] initPos = drive.getModulePositions();
    tiltScaler.reset(initPos);
    tiltPoseEstimator =
        new SwerveDrivePoseEstimator(
            new SwerveDriveKinematics(MODULE_LOCATIONS),
            drive.getRotation(),
            initPos,
            STARTING_POSE);

    NetworkTable table = NetworkTableInstance.getDefault().getTable("TiltOdometry");
    tiltDegPub = table.getDoubleTopic("TiltDeg").publish();
    scalePub = table.getDoubleTopic("Scale").publish();
    visionAllowedPub = table.getBooleanTopic("VisionAllowed").publish();
  }

  public Command driveFieldRelative(Supplier<ChassisVelocities> speedsSupplier) {
    return run(coroutine -> {
      while (true) {
        drive.setFieldRelativeChassisSpeeds(speedsSupplier.get());
        coroutine.yield();
      }
    }).named("Field Oriented Drive");
  }

  public Command zeroGyro() {
    return run(coroutine -> drive.zeroGyro()).named("Zero Gyro");
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

  /** Resets both YAMS odometry and the standalone tilt-scaled pose estimator together. */
  public void resetOdometry(Pose2d pose) {
    drive.resetOdometry(pose);
    SwerveModulePosition[] positions = drive.getModulePositions();
    tiltScaler.reset(positions);
    tiltPoseEstimator.resetPosition(drive.getRotation(), positions, pose);
  }

  /** Fuses a 2D vision measurement only when current chassis 3D tilt is within {@code MAX_VISION_TILT}. */
  public boolean addVisionMeasurement(Pose2d visionPose, double timestampSeconds) {
    if (!tiltScaler.shouldAcceptVision(drive.getGyroRotation3d())) {
      return false;
    }
    tiltPoseEstimator.addVisionMeasurement(visionPose, timestampSeconds);
    return true;
  }

  public Pose2d getTiltScaledPose() {
    return tiltPoseEstimator.getEstimatedPosition();
  }

  public Field2d getField2d() {
    return drive.getField2d();
  }

  private void updateTiltOdometry() {
    Rotation3d attitude = drive.getGyroRotation3d();
    SwerveModulePosition[] scaled = tiltScaler.scalePositions(attitude, drive.getModulePositions());
    tiltPoseEstimator.update(drive.getRotation(), scaled);

    tiltDegPub.set(TiltOdometryScaler.computeTiltAngle(attitude).in(Degrees));
    scalePub.set(tiltScaler.getLastScale());
    visionAllowedPub.set(tiltScaler.shouldAcceptVision(attitude));
    drive.getField2d().getObject("Tilt-Scaled Pose").setPose(getTiltScaledPose());
  }

  public void periodic() {
    drive.updateTelemetry();
    updateTiltOdometry();
  }

  public void simulationPeriodic() {
    drive.simIterate();
    gyro.getSimState().setRawYaw(drive.getSimPose().getRotation().getMeasure());
  }
}
