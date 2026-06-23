// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package frc.robot.subsystems;


import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
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
 * Swerve drive subsystem built with YAMS. This example shows how to wire up four NEO-driven modules
 * (each with a drive motor, a steer motor, and a CANcoder absolute encoder) into a field-relative
 * SwerveDrive. Key points:
 * <ul>
 *   <li>Module positions are declared as Translation2d offsets from robot center; kinematics is
 *       derived automatically from those positions by SwerveDriveConfig.</li>
 *   <li>Drive PID uses velocity control backed by a SimpleMotorFeedforward whose kV is derived
 *       from the theoretical top speed, so the feedforward does most of the work and kP only
 *       trims the residual error.</li>
 *   <li>Steer PID uses position control; kP=1 is usually enough for a 6.75:1 ratio because the
 *       gear reduction makes the output stiff.</li>
 *   <li>azimuthSysId() and driveSysId() are intentionally stubbed -- replace their bodies with
 *       real SysId routines once you have hardware to characterize.</li>
 * </ul>
 */
public class SwerveSubsystem extends SubsystemBase
{
  private final SwerveDrive drive;
  private final Field2d     field = new Field2d();

  // 360 deg/s gives comfortable spin speed without overshooting in teleop.
  private AngularVelocity maximumChassisSpeedsAngularVelocity = DegreesPerSecond.of(360);
  // 1 m/s is a conservative starting cap; raise it once PID and feedforward are tuned.
  private LinearVelocity  maximumChassisSpeedsLinearVelocity  = MetersPerSecond.of(1);

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

  /**
   * Build a single swerve module from its two motors and absolute encoder.
   *
   * <p>All four modules on this robot share identical gearing, wheel size, and PID gains. If your
   * robot has asymmetric modules, create separate configs per module instead of reusing this
   * factory.
   *
   * @param drive           NEO driving the wheel.
   * @param azimuth         NEO steering the module.
   * @param absoluteEncoder CANcoder that gives the true wheel angle at power-on.
   * @param moduleName      Telemetry prefix shown in SmartDashboard.
   * @param location        Module center relative to robot center (WPILib convention: +X forward,
   *                        +Y left).
   * @return Configured {@link SwerveModule}.
   */
  public SwerveModule createModule(SparkMax drive, SparkMax azimuth, CANcoder absoluteEncoder, String moduleName,
                                   Translation2d location)
  {
    // MK4i L1 drive ratio; adjust to match your actual module.
    MechanismGearing driveGearing   = new MechanismGearing(12.75);
    // MK4i steer ratio; most COTS modules land between 6:1 and 15:1.
    MechanismGearing azimuthGearing = new MechanismGearing(6.75);
    // 4-inch wheels are standard for MK4i; diameter feeds both kinematics and feedforward.
    Distance wheelDiameter = Inches.of(4);
    SmartMotorControllerConfig driveCfg = new SmartMotorControllerConfig(this)
        .withWheelDiameter(wheelDiameter)
        // kP=0.3 trims velocity error left over after the feedforward; raise it if the module
        // lags under load, lower it if it oscillates.
        .withClosedLoopController(0.3, 0, 0)
        .withGearing(driveGearing)
        // kV = 12 V / (max wheel speed in rad/s): at full speed the feedforward alone commands 12 V,
        // leaving the PID with nothing to do except correct disturbances.
        .withFeedforward(new SimpleMotorFeedforward(0,
                                                    12.0 / (maximumChassisSpeedsLinearVelocity.in(MetersPerSecond) /
                                                            wheelDiameter.in(Meters)),
                                                    0.01))
        // 40 A prevents belt slip under hard acceleration; typical NEO limit for drive applications.
        .withStatorCurrentLimit(Amps.of(40))
        .withTelemetry("driveMotor", SmartMotorControllerConfig.TelemetryVerbosity.HIGH);
    SmartMotorControllerConfig azimuthCfg = new SmartMotorControllerConfig(this)
        // kP=1 is enough at 6.75:1 because the reduction makes the output stiff; the module
        // settles within one or two robot loops without needing D or I.
        .withClosedLoopController(1, 0, 0)
        // kV=1 is a placeholder; a real characterization pass with driveSysId() will replace it.
        .withFeedforward(new SimpleMotorFeedforward(0, 1))
        .withGearing(azimuthGearing)
        // 20 A is sufficient for steering; the steer motor almost never stalls under normal driving.
        .withStatorCurrentLimit(Amps.of(20))
        .withTelemetry("angleMotor", SmartMotorControllerConfig.TelemetryVerbosity.HIGH);
    SmartMotorController driveSMC   = new SparkWrapper(drive, DCMotor.getNEO(1), driveCfg);
    SmartMotorController azimuthSMC = new SparkWrapper(azimuth, DCMotor.getNEO(1), azimuthCfg);
    SwerveModuleConfig moduleConfig = new SwerveModuleConfig(driveSMC, azimuthSMC)
        // The CANcoder absolute position eliminates the need to home the steer motor at startup.
        .withAbsoluteEncoder(absoluteEncoder.getAbsolutePosition().asSupplier())
        .withTelemetry(moduleName, SmartMotorControllerConfig.TelemetryVerbosity.HIGH)
        .withLocation(location)
        // Optimization rotates the module at most 90 deg instead of 180 deg + reversing drive direction.
        .withOptimization(true);
    return new SwerveModule(moduleConfig);
  }

  public SwerveSubsystem()
  {
    // CAN ID 14 is arbitrary; assign it to match your robot's Pigeon2 CAN ID.
    Pigeon2 gyro = new Pigeon2(14);
    /*
     * Module locations use the WPILib coordinate frame: +X toward the front of the robot,
     * +Y toward the left side. The 24-inch offsets assume the module centers sit 24 inches
     * from the robot center in both axes; update these to match your chassis measurements.
     * CAN IDs are grouped in pairs (drive, steer) followed by the CANcoder, incrementing by
     * module to make wiring audits straightforward.
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
        // gyro.getYaw() gives the heading used to rotate the velocity vector for field-relative
        // driving; without this, forward on the stick always means "robot nose direction".
        .withGyro(gyro.getYaw().asSupplier())
        .withStartingPose(new Pose2d(0, 0, Rotation2d.fromDegrees(0)))
        // Translation and rotation PIDs are used by driveToPose(); kP=1 is a conservative start.
        .withTranslationController(new PIDController(1, 0, 0))
        .withRotationController(new PIDController(1, 0, 0));
    drive = new SwerveDrive(config);

    SmartDashboard.putData("Field", field);
  }

  /**
   * Drive the {@link SwerveDrive} object with field relative chassis speeds.
   *
   * @param speedsSupplier Field relative {@link ChassisSpeeds}.
   * @return {@link Command} to run the drive.
   */
  public Command drive(Supplier<ChassisSpeeds> speedsSupplier)
  {
    return run(() -> drive.setFieldRelativeChassisSpeeds(speedsSupplier.get())).withName("Field Oriented Drive");
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

  public Command azimuthSysId()
  {
    return runOnce(() -> {});
  }

  public Command driveSysId()
  {
    return runOnce(() -> {});
  }

  @Override
  public void periodic()
  {
    drive.updateTelemetry();
    field.setRobotPose(drive.getPose());
  }

  @Override
  public void simulationPeriodic()
  {
    drive.simIterate();
  }
}
