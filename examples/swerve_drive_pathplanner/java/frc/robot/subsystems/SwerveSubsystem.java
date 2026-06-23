// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package frc.robot.subsystems;

/**
 * SwerveSubsystem -- extends the basic swerve_drive example with PathPlanner integration.
 *
 * The key addition here is setupPathPlanner(), which calls AutoBuilder.configure() once
 * at construction time to register this subsystem as the drive base for all PathPlanner
 * auto routines. After that single call, PathPlannerAuto commands and GUI-defined paths
 * just work -- no extra wiring per auto.
 *
 * Robot config (mass, MOI, module positions) is loaded from the GUI-exported JSON via
 * RobotConfig.fromGUISettings() rather than hardcoded here. Edit the config in the
 * PathPlanner GUI and export; the robot picks it up on next deploy.
 *
 * CAN ID layout (adjust to match your robot):
 *   FL drive=1  azimuth=2  encoder=3
 *   FR drive=4  azimuth=5  encoder=6
 *   BL drive=7  azimuth=8  encoder=9
 *   BR drive=10 azimuth=11 encoder=12
 *   Pigeon2=14
 */

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.io.IOException;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.json.simple.parser.ParseException;
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
  private final SwerveDrive drive;
  private final Field2d     field = new Field2d();

  // 720 deg/s == two full rotations per second; aggressive but reachable with NEOs at 12 V.
  private AngularVelocity maximumChassisSpeedsAngularVelocity = DegreesPerSecond.of(720);
  // 4 m/s is the free-speed ceiling for most competition swerve modules with 4-inch wheels.
  private LinearVelocity  maximumChassisSpeedsLinearVelocity  = MetersPerSecond.of(4);

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
        .withDeadband(0.01) // Filters joystick drift below 1%; raise to 0.05 if sticks are sloppy.
        .withCubeRotationControllerAxis()     // Cube scaling gives finer control near center without reducing max speed.
        .withCubeTranslationControllerAxis()  // Same reasoning for translation.
        .withAllianceRelativeControl();       // Translates driver inputs to field coords based on FMS alliance color.
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
    // Multiplies the [-1,1] scalar directly by the max velocity -- no deadband or shaping.
    // Useful for debugging or when SwerveInputStream behavior is not wanted.
    return () -> new ChassisSpeeds(maximumChassisSpeedsLinearVelocity.times(translationXScalar.getAsDouble())
                                                                     .in(MetersPerSecond),
                                   maximumChassisSpeedsLinearVelocity.times(translationYScalar.getAsDouble())
                                                                     .in(MetersPerSecond),
                                   maximumChassisSpeedsAngularVelocity.times(rotationScalar.getAsDouble())
                                                                      .in(RadiansPerSecond));
  }

  public SwerveModule createModule(SparkMax drive, SparkMax azimuth, CANcoder absoluteEncoder, String moduleName,
                                   Translation2d location)
  {
    // 12:1 stage then 2:1 bevel == 24:1 total drive reduction; tune to match your actual gearbox.
    MechanismGearing driveGearing         = new MechanismGearing(GearBox.fromStages("12:1", "2:1"));
    // 21:1 single-stage azimuth; typical for MK4i-style modules.
    MechanismGearing azimuthGearing       = new MechanismGearing(GearBox.fromStages("21:1"));
    PIDController    azimuthPIDController = new PIDController(1, 0, 0);
    SmartMotorControllerConfig driveCfg = new SmartMotorControllerConfig(this)
        .withWheelDiameter(Inches.of(4))           // Measure the actual wheel; worn wheels are smaller than nominal.
        .withClosedLoopController(50, 0, 4)        // kP=50 on velocity is a starting point; expect 20-80 in practice.
        .withGearing(driveGearing)
        .withStatorCurrentLimit(Amps.of(40))       // 40 A stall limit keeps NEOs below thermal rollback at full push.
        .withTelemetry("driveMotor", SmartMotorControllerConfig.TelemetryVerbosity.HIGH);
    SmartMotorControllerConfig azimuthCfg = new SmartMotorControllerConfig(this)
        .withClosedLoopController(50, 0, 4)
        // Continuous wrapping lets the controller take the shortest arc through +/-pi without unwinding.
        .withContinuousWrapping(Radians.of(-Math.PI), Radians.of(Math.PI))
        .withGearing(azimuthGearing)
        .withStatorCurrentLimit(Amps.of(20))       // Azimuth has no traction load; 20 A is generous headroom.
        .withTelemetry("angleMotor", SmartMotorControllerConfig.TelemetryVerbosity.HIGH);
    SmartMotorController driveSMC   = new SparkWrapper(drive, DCMotor.getNEO(1), driveCfg);
    SmartMotorController azimuthSMC = new SparkWrapper(azimuth, DCMotor.getNEO(1), azimuthCfg);
    SwerveModuleConfig moduleConfig = new SwerveModuleConfig(driveSMC, azimuthSMC)
        .withAbsoluteEncoder(absoluteEncoder.getAbsolutePosition().asSupplier())
        .withTelemetry(moduleName, SmartMotorControllerConfig.TelemetryVerbosity.HIGH)
        .withLocation(location)
        .withOptimization(true); // Wheel flip optimization: never rotate more than 90 deg, reverse drive instead.
    return new SwerveModule(moduleConfig);
  }

  public SwerveSubsystem()
  {
    Pigeon2 gyro = new Pigeon2(14); // Pigeon2 on CAN ID 14; change to match your bus.
    /*
     * Module locations are measured from the robot center to the wheel contact patch.
     * +X = forward, +Y = left (standard WPILib convention).
     * These are 24"x24" corner positions for a typical 26-inch square frame.
     */
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
    SwerveDriveConfig config = new SwerveDriveConfig(this, fl, fr, bl, br)
        .withGyro(gyro.getYaw().asSupplier())
        // Starting at (0,0) facing 0 deg places the robot at the blue-alliance wall origin.
        // Override in auto init if your starting position differs.
        .withStartingPose(new Pose2d(0, 0, Rotation2d.fromDegrees(0)))
        .withTranslationController(new PIDController(1, 0, 0)) // Used by driveToPose(); tune alongside PathPlanner PIDs.
        .withRotationController(new PIDController(1, 0, 0));
    drive = new SwerveDrive(config);

    SmartDashboard.putData("Field", field); // Puts a Field2d widget on the dashboard for pose visualization.

    // Register the drive with PathPlanner. Must be called once after SwerveDrive is constructed.
    try
    {
      setupPathPlanner();
    } catch (IOException | ParseException e)
    {
      throw new RuntimeException("PathPlanner setup failed -- check deploy/pathplanner/settings.json exists", e);
    }
  }

  /*
   * Registers this subsystem with PathPlanner's AutoBuilder so that PathPlannerAuto commands
   * can locate the drive and issue ChassisSpeeds without any further wiring.
   *
   * AutoBuilder.configure() is the holonomic variant; it expects robot-relative speeds in
   * and out. WPILib's ramsete/LTV controllers are NOT used here -- PathPlanner's own
   * PPHolonomicDriveController handles the path following math.
   */
  private void setupPathPlanner() throws IOException, ParseException
  {
    AutoBuilder.configure(
        drive::getPose,
        // Robot pose supplier
        drive::resetOdometry,
        // Method to reset odometry (will be called if your auto has a starting pose)
        drive::getRobotRelativeSpeed,
        // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        (speedsRobotRelative, moduleFeedForwards) -> {
          drive.setRobotRelativeChassisSpeeds(speedsRobotRelative);
        },
        // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
        new PPHolonomicDriveController(
            // PPHolonomicController is the built in path following controller for holonomic drive trains
            new PIDConstants(5.0, 0.0, 0.0),
            // Translation PID constants; kP=5 works for most robots -- increase if position error is large at path end.
            new PIDConstants(5.0, 0.0, 0.0)
            // Rotation PID constants; same tuning philosophy as translation.
        ),
        // Reads robot mass, MOI, and module positions from the JSON exported by the PathPlanner GUI.
        // File lives at deploy/pathplanner/settings.json after export. Do not hardcode these values here.
        RobotConfig.fromGUISettings(),
        () -> {
          // Flip the path to the red side when we are on Red alliance.
          // The field origin is always on the Blue alliance wall regardless of which side we drive from.
          var alliance = DriverStation.getAlliance();
          return alliance.filter(value -> value == DriverStation.Alliance.Red).isPresent();
        },
        this
        // Reference to this subsystem to set requirements
                         );
  }

  /**
   * Drive the {@link SwerveDrive} object with robot relative chassis speeds.
   *
   * @param speedsSupplier Robot relative {@link ChassisSpeeds}.
   * @return {@link Command} to run the drive.
   */
  public Command drive(Supplier<ChassisSpeeds> speedsSupplier)
  {
    return drive.drive(speedsSupplier);
  }

  public Command setRobotRelativeChassisSpeeds(ChassisSpeeds speeds)
  {
    return run(() -> drive.setRobotRelativeChassisSpeeds(speeds));
  }

  public Command driveToPose(Pose2d pose)
  {
    return drive.driveToPose(pose);
  }

  public Command driveRobotRelative(Supplier<ChassisSpeeds> speedsSupplier)
  {
    return drive.drive(speedsSupplier);
  }

  public Command lock()
  {
    return run(drive::lockPose);
  }

  @Override
  public void periodic()
  {
    drive.updateTelemetry();
    field.setRobotPose(drive.getPose()); // Keep the dashboard Field2d widget in sync every loop.
  }

  @Override
  public void simulationPeriodic()
  {
    drive.simIterate();
  }

}
