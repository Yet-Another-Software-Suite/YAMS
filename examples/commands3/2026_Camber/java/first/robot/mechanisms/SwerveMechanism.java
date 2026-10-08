// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later
// Ported from Team 9658's 2026-KitBot (https://github.com/9658-Camber-Robotics/2026-KitBot).

package first.robot.mechanisms;

import static first.robot.Constants.SwerveDrive.Modules.*;
import static org.wpilib.units.Units.Celsius;
import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Fahrenheit;
import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.Radians;

import com.ctre.phoenix6.hardware.CANcoder;
import com.limelightvision.FiducialTarget;
import com.limelightvision.IMUMode;
import com.limelightvision.Limelight;
import com.limelightvision.PoseEstimate;
import com.limelightvision.PoseEstimateType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import first.robot.Constants;
import first.robot.Constants.CANIDS;
import first.robot.Constants.SwerveDrive.Modules.Module;
import first.robot.utils.AllianceFlipUtil;
import first.robot.utils.FieldConstants.Hub;
import java.util.Arrays;
import java.util.List;
import java.util.function.Supplier;
import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;
import org.wpilib.driverstation.Alliance;
import org.wpilib.driverstation.MatchState;
import org.wpilib.driverstation.RobotState;
import org.wpilib.hardware.imu.OnboardIMU;
import org.wpilib.hardware.imu.OnboardIMU.MountOrientation;
import org.wpilib.math.controller.PIDController;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.math.util.Units;
import org.wpilib.smartdashboard.Field2d;
import org.wpilib.telemetry.Telemetry;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.Distance;
import yams.commands3.config.SmartMotorControllerConfig;
import yams.commands3.config.SwerveDriveConfig;
import yams.commands3.swerve.SwerveDrive;
import yams.commands3.swerve.SwerveInputStream;
import yams.commands3.telemetry.SwerveInputStreamTelemetry;
import yams.core.gearing.MechanismGearing;
import yams.core.mechanisms.config.SwerveModuleConfig;
import yams.core.mechanisms.swerve.SwerveModule;
import yams.core.motorcontrollers.SmartMotorController;
import yams.core.motorcontrollers.enums.ControlMode;
import yams.core.motorcontrollers.enums.MotorMode;
import yams.core.motorcontrollers.local.SparkWrapper;
import yams.core.telemetry.enums.TelemetryVerbosity;

/**
 * Swerve drivetrain: four NEO + NEO SPARK MAX modules with CANcoders. YAGSL built it from the JSON
 * files in {@code deploy/swerve}; it is now a YAMS {@link SwerveDrive} built from
 * {@link Constants.SwerveDrive.Modules}. It also fuses Limelight poses.
 */
public class SwerveMechanism implements Mechanism
{

  /** Tolerances at a waypoint the robot drives through, into the next one. */
  private static final Distance kWaypointTranslationTolerance = Meters.of(0.3);
  private static final Angle    kWaypointRotationTolerance    = Degrees.of(15);
  /** Tolerances at a pose the robot stops at. */
  private static final Distance kStopTranslationTolerance     = Meters.of(0.05);
  private static final Angle    kStopRotationTolerance        = Degrees.of(3);

  SwerveDrive swerveDrive;
  /** Active driver input, set by the teleop opmode. */
  private SwerveInputStream inputStream;
  // Systemcore's built-in IMU replaces the navX on the roboRIO SPI port.
  private final OnboardIMU imu = new OnboardIMU(MountOrientation.FLAT);
  // limelight stuff
  private Limelight limelight_swerve;
  private double    lastLLTimestamp_swerve = 0;
  private boolean   isLLEnabled_swerve     = false;

  public SwerveMechanism()
  {
    SwerveModule[] modules = Arrays.stream(new Module[]{frontLeft, frontRight, backLeft, backRight})
                                   .map(this::createModule)
                                   .toArray(SwerveModule[]::new);
    SwerveDriveConfig config = new SwerveDriveConfig(this, modules)
        .withGyro(imu::getRotation3d)
        .withStartingPose(Constants.SwerveDrive.startPose)
        .withMaximumChassisSpeed(maxLinearVelocity, maxAngularVelocity)
        .withMaximumModuleSpeed(maxLinearVelocity)
        .withModuleStateOptimization(true)
        // Translation PID for the autos' drive to pose, and heading PID from controllerproperties.json,
        // used by drive to pose, SwerveInputStream heading control and aiming.
        .withTranslationController(new PIDController(translationKP, 0, 0))
        .withRotationController(new PIDController(headingKP, 0, 0))
        .withTelemetry("Swerve", TelemetryVerbosity.HIGH);
    swerveDrive = new SwerveDrive(config);
    setupLimeLight();
  }

  private SwerveModule createModule(Module module)
  {
    SmartMotorControllerConfig driveConfig = new SmartMotorControllerConfig(this)
        .withControlMode(ControlMode.CLOSED_LOOP)
        .withGearing(new MechanismGearing(driveGearRatio))
        .withWheelDiameter(wheelDiameter)
        .withClosedLoopController(driveKP, 0, 0)
        .withFeedforward(driveFeedforward)
        .withZeroPower(MotorMode.BRAKE)
        .withStatorCurrentLimit(driveCurrentLimit)
        .withOpenLoopRampRate(rampRate)
        .withClosedLoopRampRate(rampRate)
        .withMotorInverted(driveInverted)
        .withTelemetry("driveMotor", TelemetryVerbosity.HIGH);
    SmartMotorControllerConfig angleConfig = new SmartMotorControllerConfig(this)
        .withControlMode(ControlMode.CLOSED_LOOP)
        .withGearing(new MechanismGearing(angleGearRatio))
        .withClosedLoopController(angleKP, 0, 0)
        .withSimClosedLoopController(angleSimKP, 0, 0)
        .withContinuousWrapping(Radians.of(-Math.PI), Radians.of(Math.PI))
        .withZeroPower(MotorMode.BRAKE)
        .withStatorCurrentLimit(angleCurrentLimit)
        .withOpenLoopRampRate(rampRate)
        .withClosedLoopRampRate(rampRate)
        .withMotorInverted(angleInverted)
        .withTelemetry("angleMotor", TelemetryVerbosity.HIGH);
    SmartMotorController driveMotor = new SparkWrapper(new SparkMax(CANIDS.canPort, module.driveId(),
                                                                    MotorType.kBrushless),
                                                       Constants.SwerveDrive.Modules.driveMotor,
                                                       driveConfig);
    SmartMotorController angleMotor = new SparkWrapper(new SparkMax(CANIDS.canPort, module.angleId(),
                                                                    MotorType.kBrushless),
                                                       Constants.SwerveDrive.Modules.angleMotor,
                                                       angleConfig);
    CANcoder encoder = new CANcoder(module.encoderId(), CANIDS.canBus);
    return new SwerveModule(new SwerveModuleConfig(driveMotor, angleMotor)
                                // The CANcoder seeds the angle motor's encoder, which the SPARK
                                // closes the loop on, as YAGSL did.
                                .withAbsoluteEncoder(encoder.getAbsolutePosition().asSupplier())
                                .withAbsoluteEncoderOffset(module.encoderOffset())
                                .withLocation(module.front(), module.left())
                                .withOptimization(true)
                                .withTelemetry(module.name(), TelemetryVerbosity.HIGH));
  }

  public void setupLimeLight()
  {
    // YAMS updates odometry in periodic(); there is no odometry thread to stop.
    limelight_swerve = new Limelight("limelight");
    limelight_swerve.setPipelineIndex(0);
    limelight_swerve.setIMUMode(IMUMode.EXTERNAL);
    limelight_swerve.setThrottle(200);
    limelight_swerve.setCameraPose_RobotSpaceOverride(
        new Pose3d(
            Units.inchesToMeters(-9),
            Units.inchesToMeters(-11),
            Units.inchesToMeters(15),
            new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(25), Units.degreesToRadians(0))),
        false);
    limelight_swerve.setFiducialIDFiltersOverride(new int[]{17, 18, 19, 20, 21, 22, 25, 26, 27, 18, 19, 20, 21,
                                                            24, 6, 7, 8, 9, 10, 11});
    Limelight.flushNT();
  }

  public double updateLimelight(Limelight ll, double llTImestamp, Rotation2d cameraYaw, String llname)
  {
    // Yaw rate of 180 deg/s kept from the original.
    ll.setRobotOrientation(getHeading().rotateBy(cameraYaw).getDegrees(), 180, 0, 0, 0, 0, true);
    PoseEstimate poseEstimate = ll.getPoseEstimate(PoseEstimateType.MT1_WPIBLUE);
    if (poseEstimate != null && poseEstimate.isValid())
    {
      Pose2d estimatorPose = poseEstimate.pose;
      swerveDrive.getField2d().getObject("Vision").setPose(estimatorPose);

      double ambiguity = averageTagAmbiguity(poseEstimate);
      Telemetry.log("LimeLightTuning/" + llname + "/ambiguity", ambiguity);
      if (ambiguity < 0.3 &&
          poseEstimate.reportedTagCount > 1)
      {
        if (llTImestamp != poseEstimate.timestampSeconds)
        {
          swerveDrive.addVisionMeasurement(estimatorPose,
                                           poseEstimate.timestampSeconds);
          return poseEstimate.timestampSeconds;
        }
      }
    }
    return llTImestamp;
  }

  /** Average ambiguity of the tags in a pose estimate, as YALL's getAvgTagAmbiguity() reported. */
  private static double averageTagAmbiguity(PoseEstimate poseEstimate)
  {
    FiducialTarget[] fiducials = poseEstimate.rawFiducials;
    if (fiducials == null || fiducials.length == 0)
    {
      return 0;
    }
    return Arrays.stream(fiducials).mapToDouble(fiducial -> fiducial.ambiguity).average().orElse(0);
  }

  /** Called from {@code Robot.robotPeriodic()}, replacing the v2 subsystem periodic. */
  public void periodic()
  {
    if (limelight_swerve.isConnected())
    {
      var temp = Celsius.of(limelight_swerve.getHardwareData().cpuTempCelsius);
      Telemetry.log("LimeLightTuning/swerve/tempF", temp.in(Fahrenheit));
    }
    if (!isLLEnabled_swerve && RobotState.isEnabled())
    {
      isLLEnabled_swerve = true;
      limelight_swerve.setThrottle(0);
    }
    // Updates odometry along with the YAMS swerve telemetry.
    swerveDrive.updateTelemetry();
    lastLLTimestamp_swerve = updateLimelight(limelight_swerve,
                                             lastLLTimestamp_swerve,
                                             Rotation2d.ZERO,
                                             "swerve");
    Telemetry.log("HubDistance(Meters)", distanceFromHubMeters().in(Meters));
  }

  /** Called from {@code Robot.simulationPeriodic()}. */
  public void simulationPeriodic()
  {
    swerveDrive.simIterate();
  }

  /**
   * Underlying YAMS {@link SwerveDrive}, for building a {@link SwerveInputStream}.
   *
   * @return {@link SwerveDrive} driven by this mechanism.
   */
  public SwerveDrive getSwerveDrive()
  {
    return swerveDrive;
  }

  /**
   * Drive with field relative {@link ChassisVelocities}. Runs until canceled.
   *
   * @param velocities Field relative {@link ChassisVelocities}, read every loop.
   * @return {@link Command} that drives the robot.
   */
  public Command driveFieldRelative(Supplier<ChassisVelocities> velocities)
  {
    return run(coroutine -> {
      while (true)
      {
        swerveDrive.setFieldRelativeChassisSpeeds(velocities.get());
        coroutine.yield();
      }
    }).named("Swerve Drive Field Relative");
  }

  /**
   * Drive with the active {@link SwerveInputStream}. A stream set while this runs takes effect on the next loop. Runs
   * until canceled.
   *
   * @return {@link Command} that drives the robot.
   */
  public Command driveInputStream()
  {
    return run(coroutine -> {
      while (true)
      {
        swerveDrive.setFieldRelativeChassisSpeeds(inputStream.get());
        coroutine.yield();
      }
    }).named("Swerve Drive Input Stream");
  }

  /**
   * Translate with the active {@link SwerveInputStream} while the robot turns to face a target, which is shown as
   * {@code AimTarget} on the field. Runs until canceled.
   *
   * @param aimTarget Pose to face, read every loop.
   * @return {@link Command} that aims while driving.
   */
  public Command driveAimedAt(Supplier<Pose2d> aimTarget)
  {
    return run(coroutine -> {
      SwerveInputStream aimInput = inputStream.clone().withAim(aimTarget, () -> true);
      while (true)
      {
        getField().getObject("AimTarget").setPose(aimTarget.get());
        swerveDrive.setFieldRelativeChassisSpeeds(aimInput.get());
        coroutine.yield();
      }
    }).whenCanceled(() -> getField().getObject("AimTarget").setPoses(List.of())).named("Auto Aim");
  }

  /**
   * Set the active driver input.
   *
   * @param inputStream Field relative {@link SwerveInputStream} driven by {@link #driveInputStream()}.
   */
  public void setInputStream(SwerveInputStream inputStream)
  {
    // Stop publishing the replaced stream's telemetry, e.g. when another teleop opmode is selected.
    if (this.inputStream != null)
    {
      this.inputStream.getTelemetry().ifPresent(SwerveInputStreamTelemetry::close);
    }
    this.inputStream = inputStream;
  }

  /**
   * Get the active driver input, so commands can read or adjust it.
   *
   * @return Active {@link SwerveInputStream}.
   */
  public SwerveInputStream getInputStream()
  {
    return inputStream;
  }

  /**
   * Drive straight to a pose, ending once the robot is within the given tolerances. The drive stops when the command
   * ends or is canceled.
   *
   * @param pose                 Field relative, blue-origin {@link Pose2d} to drive to.
   * @param translationTolerance Maximum distance from the pose to be considered at the pose.
   * @param rotationTolerance    Maximum heading error from the pose to be considered at the pose.
   * @return {@link Command} that drives to the pose.
   */
  public Command driveToPose(Pose2d pose, Distance translationTolerance, Angle rotationTolerance)
  {
    return swerveDrive.driveToPose(pose, translationTolerance, rotationTolerance);
  }

  /**
   * Drive through a waypoint: drive toward a blue-origin pose, flipped for the red alliance, ending once the robot is
   * near it so it carries on into the next waypoint without stopping.
   *
   * @param bluePose Field relative, blue-origin {@link Pose2d}.
   * @return {@link Command} that ends near the pose.
   */
  public Command driveThroughPose(Pose2d bluePose)
  {
    return driveToPose(AllianceFlipUtil.apply(bluePose), kWaypointTranslationTolerance, kWaypointRotationTolerance);
  }

  /**
   * Drive to a blue-origin pose, flipped for the red alliance, and stop there.
   *
   * @param bluePose Field relative, blue-origin {@link Pose2d}.
   * @return {@link Command} that ends at the pose.
   */
  public Command driveToPose(Pose2d bluePose)
  {
    return driveToPose(AllianceFlipUtil.apply(bluePose), kStopTranslationTolerance, kStopRotationTolerance);
  }

  /**
   * Reset odometry to a pose, e.g. an auto's starting pose.
   *
   * @param pose Field relative, blue-origin {@link Pose2d} the robot is at.
   */
  public void resetPose(Pose2d pose)
  {
    swerveDrive.resetOdometry(pose);
  }

  public Rotation2d getHeading()
  {
    return getPose().getRotation();
  }

  /**
   * Gets the current pose (position and rotation) of the robot, as reported by odometry.
   *
   * @return The robot's pose
   */
  public Pose2d getPose()
  {
    return swerveDrive.getPose();
  }


  public Command zeroGyroWithAllianceCommand()
  {
    return run(coroutine -> {
      MatchState.getAlliance().ifPresent(alliance -> {
        swerveDrive.zeroGyro();
        if (alliance == Alliance.RED)
        {
          swerveDrive.resetOdometry(new Pose2d(getPose().getTranslation(), Rotation2d.k180deg));
        }
      });
    }).named("Zero Gyro With Alliance");
  }

  public Command resetOdometryCommand(Pose2d odom)
  {
    return run(coroutine -> swerveDrive.resetOdometry(AllianceFlipUtil.apply(odom))).named("Reset Odometry");
  }

  public Command lock()
  {
    return run(coroutine -> {
      while (true)
      {
        swerveDrive.lockPose();
        coroutine.yield();
      }
    }).named("Lock");
  }

  public Distance distanceFromHubMeters()
  {
    return Meters.of(getPose().getTranslation()
                              .getDistance(AllianceFlipUtil.apply(Hub.topCenterPoint.toTranslation2d())));
  }

  public Field2d getField()
  {
    return swerveDrive.getField2d();
  }
}
