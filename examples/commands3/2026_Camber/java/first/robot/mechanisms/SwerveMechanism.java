// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later
// Ported from Team 9658's 2026-KitBot (https://github.com/9658-Camber-Robotics/2026-KitBot).

package first.robot.mechanisms;

import static first.robot.Constants.SwerveDrive.Modules.*;
import static org.wpilib.units.Units.Celsius;
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
import first.robot.pathplanner.PathPlannerPath;
import first.robot.pathplanner.AutoBuilder;
import first.robot.utils.AllianceFlipUtil;
import first.robot.utils.FieldConstants.Hub;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
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
import org.wpilib.math.trajectory.HolonomicSample;
import org.wpilib.math.trajectory.HolonomicTrajectory;
import org.wpilib.math.util.Units;
import org.wpilib.smartdashboard.Field2d;
import org.wpilib.system.Timer;
import org.wpilib.telemetry.Telemetry;
import org.wpilib.units.measure.Distance;
import yams.commands3.config.SmartMotorControllerConfig;
import yams.commands3.config.SwerveDriveConfig;
import yams.commands3.swerve.SwerveDrive;
import yams.commands3.swerve.SwerveInputStream;
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
 * {@link Constants.SwerveDrive.Modules}. It also fuses Limelight poses and follows PathPlanner
 * paths.
 */
public class SwerveMechanism implements Mechanism
{

  SwerveDrive swerveDrive;
  // Systemcore's built-in IMU replaces the navX on the roboRIO SPI port.
  private final OnboardIMU imu = new OnboardIMU(MountOrientation.FLAT);
  // limelight stuff
  private Limelight limelight_swerve;
  private double    lastLLTimestamp_swerve = 0;
  private boolean   isLLEnabled_swerve     = false;

  // PathPlanner path following PID, as the original PPHolonomicDriveController.
  private final PIDController pathXController     = new PIDController(5.0, 0.0, 0.0);
  private final PIDController pathYController     = new PIDController(5.0, 0.0, 0.0);
  private final PIDController pathRotationController = new PIDController(5.0, 0.0, 0.0);

  public SwerveMechanism()
  {
    SwerveModule[] modules = Arrays.stream(new Module[]{frontLeft, frontRight, backLeft, backRight})
                                   .map(this::createModule)
                                   .toArray(SwerveModule[]::new);
    SwerveDriveConfig config = new SwerveDriveConfig(this, modules)
        .withGyro(() -> Radians.of(imu.getYawRadians()))
        .withStartingPose(Constants.SwerveDrive.startPose)
        .withMaximumChassisSpeed(maxLinearVelocity, maxAngularVelocity)
        .withMaximumModuleSpeed(maxLinearVelocity)
        .withModuleStateOptimization(true)
        // Heading PID from controllerproperties.json, used by SwerveInputStream heading control and
        // aiming.
        .withRotationController(new PIDController(headingKP, 0, 0))
        .withTelemetry("Swerve", TelemetryVerbosity.HIGH);
    swerveDrive = new SwerveDrive(config);
    pathRotationController.enableContinuousInput(-Math.PI, Math.PI);
    setupPathPlanner();
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
        .withIdleMode(MotorMode.BRAKE)
        .withStatorCurrentLimit(driveCurrentLimit)
        .withOpenLoopRampRate(rampRate)
        .withClosedLoopRampRate(rampRate)
        .withMotorInverted(driveInverted)
        .withTelemetry("driveMotor", TelemetryVerbosity.HIGH);
    SmartMotorControllerConfig angleConfig = new SmartMotorControllerConfig(this)
        .withControlMode(ControlMode.CLOSED_LOOP)
        .withGearing(new MechanismGearing(angleGearRatio))
        .withClosedLoopController(angleKP, 0, 0)
        .withContinuousWrapping(Radians.of(-Math.PI), Radians.of(Math.PI))
        .withIdleMode(MotorMode.BRAKE)
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

  public SwerveDrive getSwerveDrive()
  {
    return swerveDrive;
  }

  /**
   * Create a driver input stream for this drivetrain. Each drive command owns one and sets its
   * sticks and modes every loop.
   *
   * @return Input stream with the original deadband, translation scale and alliance relative control.
   */
  public SwerveInputStream createInputStream()
  {
    return new SwerveInputStream(swerveDrive)
        .withDeadband(0.1)
        .withScaleTranslation(.8)
        .withAllianceRelativeControl(true);
  }

  public void driveFieldOriented(ChassisVelocities velocity)
  {
    swerveDrive.setFieldRelativeChassisSpeeds(velocity);
  }

  /** Point the PathPlanner reader at this drivetrain. */
  public void setupPathPlanner()
  {
    AutoBuilder.configure(this::followPath,
                          swerveDrive::resetOdometry,
                          () -> {
                            // Boolean supplier that controls when the path will be mirrored for the red alliance
                            // This will flip the path being followed to the red side of the field.
                            // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
                            var alliance = MatchState.getAlliance();
                            if (alliance.isPresent())
                            {
                              return alliance.get() == Alliance.RED;
                            }
                            return false;
                          });
  }

  /**
   * Follow a PathPlanner path, mirrored for the red alliance, running its event markers along the way.
   * Each marker's command is forked when the robot reaches it. A zoned marker's command is canceled
   * when the robot leaves the zone, and every marker command still running is canceled when the path
   * ends, as PathPlannerLib does.
   *
   * @param path Path to follow.
   * @return Command that follows the path.
   */
  public Command followPath(PathPlannerPath path)
  {
    List<Command> markerCommands = new ArrayList<>();
    for (PathPlannerPath.EventMarker marker : path.getEventMarkers())
    {
      markerCommands.add(AutoBuilder.buildCommand(marker.command()));
    }
    return run(coroutine -> {
      HolonomicTrajectory trajectory = path.getTrajectory(AutoBuilder.shouldFlip());
      List<PathPlannerPath.EventMarker> markers = path.getEventMarkers();
      boolean[] started = new boolean[markers.size()];
      pathXController.reset();
      pathYController.reset();
      pathRotationController.reset();
      Timer timer = new Timer();
      timer.start();
      while (!timer.hasElapsed(trajectory.duration))
      {
        double time = timer.get();
        followSample(trajectory.sampleAt(time));
        for (int i = 0; i < markers.size(); i++)
        {
          PathPlannerPath.EventMarker marker = markers.get(i);
          if (!started[i] && time >= marker.startTime())
          {
            started[i] = true;
            coroutine.fork(markerCommands.get(i));
          }
          if (started[i] && marker.endTime().isPresent() && time >= marker.endTime().get())
          {
            // Left the marker's zone.
            coroutine.scheduler().cancel(markerCommands.get(i));
          }
        }
        coroutine.yield();
      }
      // Every path in this robot ends at rest, so stop instead of holding the last speed. Marker
      // commands are canceled as this command ends.
      swerveDrive.setRobotRelativeChassisSpeeds(new ChassisVelocities());
    }).named("Follow " + path.getName());
  }

  /** Drive toward a field relative trajectory sample: its velocity plus a PID correction. */
  private void followSample(HolonomicSample sample)
  {
    Pose2d            pose   = getPose();
    ChassisVelocities speeds = new ChassisVelocities(
        sample.velocity.vx + pathXController.calculate(pose.getX(), sample.pose.getX()),
        sample.velocity.vy + pathYController.calculate(pose.getY(), sample.pose.getY()),
        sample.velocity.omega + pathRotationController.calculate(pose.getRotation().getRadians(),
                                                                 sample.pose.getRotation().getRadians()));
    swerveDrive.setRobotRelativeChassisSpeeds(speeds.toRobotRelative(pose.getRotation()));
  }

  /**
   * Run a PathPlanner auto.
   *
   * @param autoName PathPlanner auto name.
   * @return Command that runs the auto.
   */
  public Command getAutonomousCommand(String autoName)
  {
    return AutoBuilder.buildAuto(autoName);
  }

  public Command driveToPose(Pose2d pose)
  {
    // PathPlanner pathfinding is not available, so drive straight to the pose with the YAMS PID.
    return swerveDrive.driveToPose(pose);
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

  public void driveFieldOrientedSetpoint(ChassisVelocities speeds)
  {
    swerveDrive.setFieldRelativeChassisSpeeds(speeds);
  }

  public Field2d getField()
  {
    return swerveDrive.getField2d();
  }
}
