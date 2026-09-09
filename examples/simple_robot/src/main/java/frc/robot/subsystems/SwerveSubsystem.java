// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RotationsPerSecond;

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
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import java.util.function.Supplier;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.math.DerivativeTimeFilter;
import yams.mechanisms.config.SwerveDriveConfig;
import yams.mechanisms.config.SwerveModuleConfig;
import yams.mechanisms.swerve.SwerveDrive;
import yams.mechanisms.swerve.SwerveModule;
import yams.mechanisms.swerve.utility.SwerveInputStream;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.local.SparkWrapper;

public class SwerveSubsystem extends SubsystemBase {
  private final SwerveDrive drive;
  private final Field2d field = new Field2d();

  public SwerveModule createModule(SparkMax drive, SparkMax azimuth, CANcoder absoluteEncoder,
      String moduleName, Translation2d location) {
    MechanismGearing driveGearing = new MechanismGearing(GearBox.fromReductionStages(6.75));
    MechanismGearing azimuthGearing = new MechanismGearing(GearBox.fromReductionStages(12.8));
    SmartMotorControllerConfig driveCfg =
        new SmartMotorControllerConfig(this)
            .withWheelDiameter(Inches.of(4))
            .withClosedLoopController(0.4, 0, 0)
            //            .withFeedforward(new SimpleMotorFeedforward(0, 0.7, 0.1))
            .withGearing(driveGearing)
            .withStatorCurrentLimit(Amps.of(40))
            .withTelemetry("driveMotor", SmartMotorControllerConfig.TelemetryVerbosity.HIGH);
    SmartMotorControllerConfig azimuthCfg =
        new SmartMotorControllerConfig(this)
            .withClosedLoopController(3.8476, 0, 0)
            .withContinuousWrapping(Radians.of(-Math.PI), Radians.of(Math.PI))
            .withGearing(azimuthGearing)
            .withStatorCurrentLimit(Amps.of(40))
            .withTelemetry("angleMotor", SmartMotorControllerConfig.TelemetryVerbosity.HIGH);
    SmartMotorController driveSMC = new SparkWrapper(drive, DCMotor.getNEO(1), driveCfg);
    SmartMotorController azimuthSMC = new SparkWrapper(azimuth, DCMotor.getNEO(1), azimuthCfg);
    SwerveModuleConfig moduleConfig =
        new SwerveModuleConfig(driveSMC, azimuthSMC)
            .withAbsoluteEncoder(absoluteEncoder.getAbsolutePosition().asSupplier())
            .withTelemetry(moduleName, SmartMotorControllerConfig.TelemetryVerbosity.HIGH)
            .withLocation(location)
            .withOptimization(true);
    return new SwerveModule(moduleConfig);
  }

  private final Pigeon2 gyro = new Pigeon2(14);
  public SwerveSubsystem() {
    var fl =
        createModule(new SparkMax(1, MotorType.kBrushless), new SparkMax(2, MotorType.kBrushless),
            new CANcoder(3), "frontleft", new Translation2d(Inches.of(10), Inches.of(10)));
    var fr =
        createModule(new SparkMax(4, MotorType.kBrushless), new SparkMax(5, MotorType.kBrushless),
            new CANcoder(6), "frontright", new Translation2d(Inches.of(10), Inches.of(-10)));
    var bl =
        createModule(new SparkMax(7, MotorType.kBrushless), new SparkMax(8, MotorType.kBrushless),
            new CANcoder(9), "backleft", new Translation2d(Inches.of(-10), Inches.of(10)));
    var br =
        createModule(new SparkMax(10, MotorType.kBrushless), new SparkMax(11, MotorType.kBrushless),
            new CANcoder(12), "backright", new Translation2d(Inches.of(-10), Inches.of(-10)));
    SwerveDriveConfig config =
        new SwerveDriveConfig(this, fl, fr, bl, br)
            .withGyro(gyro.getYaw().asSupplier())
            //.withGyroVelocity(gyro.getAngularVelocityZDevice().asSupplier())
            //.withGyroAngularVelocityScaleFactor(1)
            .withMaximumChassisSpeed(MetersPerSecond.of(4), RotationsPerSecond.of(360))
            .withStartingPose(new Pose2d(0, 0, Rotation2d.fromDegrees(0)))
            .withTranslationController(new PIDController(1, 0, 0))
            .withRotationController(new PIDController(1, 0, 0));
    drive = new SwerveDrive(config);

    SmartDashboard.putData("Field", field);
  }

  public Command setRobotRelativeChassisSpeeds(ChassisSpeeds speeds) {
    return run(() -> drive.setRobotRelativeChassisSpeeds(speeds));
  }

  public Command driveToPose(Pose2d pose) {
    return drive.driveToPose(pose);
  }

  public Command driveRobotRelative(Supplier<ChassisSpeeds> speedsSupplier) {
    return drive.drive(speedsSupplier);
  }

  /**
   * Drive the robot field-relative using a driver controller, converting joystick axes into
   * {@link ChassisSpeeds} via {@link SwerveInputStream}.
   *
   * @param controller Driver controller to read translation/rotation axes from.
   * @return {@link Command} that drives the robot while scheduled.
   */
  public Command driveWithJoystick(CommandXboxController controller) {
    SwerveInputStream inputStream =
        SwerveInputStream.of(drive, () -> - controller.getLeftY(), () -> - controller.getLeftX())
            .withControllerRotationAxis(() -> - controller.getRightX())
            .withMaximumLinearVelocity(MetersPerSecond.of(4))
            .withMaximumAngularVelocity(DegreesPerSecond.of(360))
            .withDeadband(0.05)
            .withCubeTranslationControllerAxis()
            .withAllianceRelativeControl();

    return drive.drive(()
                           -> ChassisSpeeds.fromFieldRelativeSpeeds(
                               inputStream.get(), new Rotation2d(drive.getGyroAngle())));
  }

  public Command lock() {
    return run(drive::lockPose);
  }

  @Override
  public void periodic() {
    drive.updateTelemetry();
    field.setRobotPose(drive.getPose());
  }

  @Override
  public void simulationPeriodic() {
    drive.simIterate();
    gyro.getSimState().setRawYaw(drive.getSimPose().getRotation().getRadians());
  }

  public Pose2d getPose() {
    return drive.getPose();
  }

  public ChassisSpeeds getFieldOrientedChassisSpeed() {
    return drive.getFieldRelativeSpeed();
  }

  public Angle getGyroAngle() {
    return drive.getGyroAngle();
  }
}
