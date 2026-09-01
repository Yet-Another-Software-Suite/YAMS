// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package frc.robot.subsystems;


import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.Inches;
import static org.wpilib.units.Units.Radians;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import org.wpilib.math.controller.PIDController;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.kinematics.ChassisSpeeds;
import org.wpilib.math.system.plant.DCMotor;
import org.wpilib.units.measure.Angle;
import org.wpilib.smartdashboard.Field2d;
import org.wpilib.smartdashboard.SmartDashboard;
import org.wpilib.command2.Command;
import org.wpilib.command2.SubsystemBase;
import java.util.function.Supplier;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.SwerveDriveConfig;
import yams.mechanisms.config.SwerveModuleConfig;
import yams.mechanisms.swerve.SwerveDrive;
import yams.mechanisms.swerve.SwerveModule;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.local.SparkWrapper;

public class SwerveSubsystem extends SubsystemBase
{
  private final SwerveDrive drive;
  private final Field2d     field = new Field2d();

  public SwerveModule createModule(SparkMax drive, SparkMax azimuth, CANcoder absoluteEncoder, String moduleName,
                                   Translation2d location)
  {
    MechanismGearing driveGearing         = new MechanismGearing(GearBox.fromStages("12:1", "2:1"));
    MechanismGearing azimuthGearing       = new MechanismGearing(GearBox.fromStages("21:1"));
    PIDController    azimuthPIDController = new PIDController(1, 0, 0);
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

  public SwerveSubsystem()
  {
    Pigeon2 gyro = new Pigeon2(14);
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
        .withStartingPose(new Pose2d(0, 0, Rotation2d.fromDegrees(0)))
        .withTranslationController(new PIDController(1, 0, 0))
        .withRotationController(new PIDController(1, 0, 0));
    drive = new SwerveDrive(config);

    SmartDashboard.putData("Field", field);
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
    field.setRobotPose(drive.getPose());
  }

  @Override
  public void simulationPeriodic()
  {
    drive.simIterate();
  }

  public Pose2d getPose()
  {
    return drive.getPose();
  }

  public ChassisSpeeds getFieldOrientedChassisSpeed()
  {
    return drive.getFieldRelativeSpeed();
  }

  public Angle getGyroAngle()
  {
    return drive.getGyroAngle();
  }
}
