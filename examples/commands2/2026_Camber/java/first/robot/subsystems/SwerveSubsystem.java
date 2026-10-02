// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later
// Ported from Team 9658's 2026-KitBot (https://github.com/9658-Camber-Robotics/2026-KitBot).

package first.robot.subsystems;

import static first.robot.Constants.SwerveDrive.Modules.*;
import static org.wpilib.units.Units.Fahrenheit;
import static org.wpilib.units.Units.Celsius;
import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.Radians;

import com.ctre.phoenix6.hardware.CANcoder;
import com.limelightvision.FiducialTarget;
import com.limelightvision.IMUMode;
import com.limelightvision.Limelight;
import com.limelightvision.PoseEstimate;
import com.limelightvision.PoseEstimateType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.swerve.SwerveSetpoint;
import com.pathplanner.lib.util.swerve.SwerveSetpointGenerator;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import first.robot.Constants;
import first.robot.Constants.CANIDS;
import first.robot.Constants.SwerveDrive.Modules.Module;
import first.robot.utils.AllianceFlipUtil;
import first.robot.utils.FieldConstants.Hub;
import java.util.Arrays;
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Supplier;
import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import org.wpilib.command2.SubsystemBase;
import org.wpilib.driverstation.Alliance;
import org.wpilib.driverstation.DriverStationErrors;
import org.wpilib.driverstation.MatchState;
import org.wpilib.driverstation.RobotState;
import org.wpilib.hardware.imu.OnboardIMU;
import org.wpilib.hardware.imu.OnboardIMU.MountOrientation;
import org.wpilib.math.controller.PIDController;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.math.util.Units;
import org.wpilib.smartdashboard.Field2d;
import org.wpilib.system.Timer;
import org.wpilib.telemetry.Telemetry;
import org.wpilib.units.measure.Distance;
import yams.commands2.config.SmartMotorControllerConfig;
import yams.commands2.config.SwerveDriveConfig;
import yams.commands2.swerve.SwerveDrive;
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
 * {@link Constants.SwerveDrive.Modules}. PathPlanner and the Limelight are set up as before.
 */
public class SwerveSubsystem extends SubsystemBase
{

  SwerveDrive swerveDrive;
  // Systemcore's built-in IMU replaces the navX on the roboRIO SPI port.
  private final OnboardIMU imu = new OnboardIMU(MountOrientation.FLAT);
  // limelight stuff
  private Limelight limelight_swerve;
  private double    lastLLTimestamp_swerve = 0;
  private boolean   isLLEnabled_swerve     = false;

  public SwerveSubsystem()
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
        // Heading PID from controllerproperties.json, used by SwerveInputStream heading control and
        // aiming.
        .withRotationController(new PIDController(headingKP, 0, 0))
        .withTelemetry("Swerve", TelemetryVerbosity.HIGH);
    swerveDrive = new SwerveDrive(config);
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
        .withSimClosedLoopController(angleSimKP, 0, 0)
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

  @Override
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
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic()
  {
    // This method will be called once per scheduler run during simulation
    swerveDrive.simIterate();
  }

  public SwerveDrive getSwerveDrive()
  {
    return swerveDrive;
  }

  public void driveFieldOriented(ChassisVelocities velocity)
  {
    swerveDrive.setFieldRelativeChassisSpeeds(velocity);
  }

  public Command driveFieldOriented(Supplier<ChassisVelocities> velocity)
  {
    return run(() -> {
      swerveDrive.setFieldRelativeChassisSpeeds(velocity.get());
    });
  }

  /**
   * Setup AutoBuilder for PathPlanner.
   */
  public void setupPathPlanner()
  {
    // Load the RobotConfig from the GUI settings. You should probably
    // store this in your Constants file
    RobotConfig config;
    try
    {
      config = RobotConfig.fromGUISettings();

      final boolean enableFeedforward = true;
      // Configure AutoBuilder last
      AutoBuilder.configure(
          swerveDrive::getPose,
          // Robot pose supplier
          swerveDrive::resetOdometry,
          // Method to reset odometry (will be called if your auto has a starting pose)
          swerveDrive::getRobotRelativeSpeed,
          // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
          (speedsRobotRelative, moduleFeedForwards) -> {
            if (enableFeedforward)
            {
              swerveDrive.setRobotRelativeChassisSpeeds(
                  speedsRobotRelative,
                  moduleFeedForwards.linearForces()
                                                       );
            } else
            {
              swerveDrive.setRobotRelativeChassisSpeeds(speedsRobotRelative);
            }
          },
          // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also, optionally outputs individual module feedforwards
          new PPHolonomicDriveController(
              // PPHolonomicController is the built-in path following controller for holonomic drive trains
              new PIDConstants(5.0, 0.0, 0.0),
              // Translation PID constants
              new PIDConstants(5.0, 0.0, 0.0)
              // Rotation PID constants
          ),
          config,
          // The robot configuration
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
          },
          this
          // Reference to this subsystem to set requirements
                           );

    } catch (Exception | LinkageError e)
    {
      // PathPlanner failing to load only loses the autos; the drivetrain still works in teleop.
      DriverStationErrors.reportError("Could not configure PathPlanner: " + e, e.getStackTrace());
    }
  }

  /**
   * Get the path follower with events.
   *
   * @param pathName PathPlanner path name.
   * @return {@link AutoBuilder#followPath(PathPlannerPath)} path command.
   */
  public Command getAutonomousCommand(String pathName)
  {
    // Create a path following command using AutoBuilder. This will also trigger event markers.
    try
    {
      return new PathPlannerAuto(pathName);
    } catch (Exception | LinkageError e)
    {
      // Do nothing rather than crash the robot code, which would also lose teleop.
      DriverStationErrors.reportError("Could not load PathPlanner auto " + pathName + ": " + e, e.getStackTrace());
      return Commands.none();
    }
  }

  public Command driveToPose(Pose2d pose)
  {
// Create the constraints to use while pathfinding
    PathConstraints constraints = new PathConstraints(
        maxLinearVelocity.in(org.wpilib.units.Units.MetersPerSecond), 4.0,
        maxAngularVelocity.in(org.wpilib.units.Units.RadiansPerSecond), Units.degreesToRadians(720));

// Since AutoBuilder is configured, we can use it to build pathfinding commands
    return AutoBuilder.pathfindToPose(
        pose,
        constraints,
        org.wpilib.units.Units.MetersPerSecond.of(0) // Goal end velocity in meters/sec
                                     );
  }

  /**
   * Drive with {@link SwerveSetpointGenerator} from 254, implemented by PathPlanner.
   *
   * @param robotRelativeChassisSpeed Robot relative {@link ChassisVelocities} to achieve.
   * @return {@link Command} to run.
   * @throws Exception If the PathPlanner GUI settings are invalid or nonexistent.
   */
  private Command driveWithSetpointGenerator(Supplier<ChassisVelocities> robotRelativeChassisSpeed)
  throws Exception
  {
    SwerveSetpointGenerator setpointGenerator = new SwerveSetpointGenerator(RobotConfig.fromGUISettings(),
                                                                            maxAngularVelocity);
    AtomicReference<SwerveSetpoint> prevSetpoint
        = new AtomicReference<>(new SwerveSetpoint(swerveDrive.getRobotRelativeSpeed(),
                                                   swerveDrive.getModuleStates(),
                                                   DriveFeedforwards.zeros(swerveDrive.getModules().length)));
    AtomicReference<Double> previousTime = new AtomicReference<>();

    return startRun(() -> previousTime.set(Timer.getTimestamp()),
                    () -> {
                      double newTime = Timer.getTimestamp();
                      SwerveSetpoint newSetpoint = setpointGenerator.generateSetpoint(prevSetpoint.get(),
                                                                                      robotRelativeChassisSpeed.get(),
                                                                                      newTime - previousTime.get());
                      swerveDrive.setSwerveModuleStates(newSetpoint.moduleStates(),
                                                        newSetpoint.feedforwards().linearForces());
                      prevSetpoint.set(newSetpoint);
                      previousTime.set(newTime);

                    });
  }

  public Command driveWithSetpointGeneratorFieldRelative(Supplier<ChassisVelocities> fieldRelativeSpeeds)
  {
    try
    {
      return driveWithSetpointGenerator(() -> {
        return fieldRelativeSpeeds.get().toRobotRelative(getHeading());

      });
    } catch (Exception e)
    {
      DriverStationErrors.reportError(e.toString(), true);
    }
    return Commands.none();

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
    return runOnce(() -> {
      MatchState.getAlliance().ifPresent(alliance -> {
        swerveDrive.zeroGyro();
        if (alliance == Alliance.RED)
        {
          swerveDrive.resetOdometry(new Pose2d(getPose().getTranslation(), Rotation2d.k180deg));
        }
      });
    });
  }

  public Command resetOdometryCommand(Pose2d odom)
  {
    return runOnce(() -> swerveDrive.resetOdometry(AllianceFlipUtil.apply(odom)));
  }

  public Command lock()
  {
    return run(swerveDrive::lockPose);
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
