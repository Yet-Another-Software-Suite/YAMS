// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package first.robot.mechanisms;

import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.DegreesPerSecond;
import static org.wpilib.units.Units.Inches;
import static org.wpilib.units.Units.MetersPerSecond;
import static org.wpilib.units.Units.Radians;

import com.ctre.phoenix6.CANBus;
import org.wpilib.hardware.bus.CANPort;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.revrobotics.util.CANPorts;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import org.wpilib.math.controller.PIDController;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.math.system.DCMotor;
import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;
import java.util.function.Supplier;
import yams.commands3.config.SmartMotorControllerConfig;
import yams.commands3.config.SwerveDriveConfig;
import yams.commands3.swerve.SwerveDrive;
import yams.commands3.swerve.SwerveInputStream;
import yams.commands3.telemetry.SwerveInputStreamTelemetry;
import yams.core.gearing.GearBox;
import yams.core.gearing.MechanismGearing;
import yams.core.mechanisms.config.SwerveModuleConfig;
import yams.core.mechanisms.swerve.SwerveModule;
import yams.core.motorcontrollers.SmartMotorController;
import yams.core.motorcontrollers.local.SparkWrapper;
import yams.core.telemetry.enums.TelemetryVerbosity;

public class SwerveMechanism implements Mechanism {
  private final SwerveDrive drive;
  /** Active driver input, set by the teleop opmode. */
  private SwerveInputStream inputStream;

  private SwerveModule createModule(
      SparkMax drive,
      SparkMax azimuth,
      CANcoder absoluteEncoder,
      String moduleName,
      Translation2d location) {
    MechanismGearing driveGearing = new MechanismGearing(GearBox.fromReductionStages(6.75));
    MechanismGearing azimuthGearing = new MechanismGearing(GearBox.fromReductionStages(12.8));
    SmartMotorControllerConfig driveCfg =
        new SmartMotorControllerConfig(this)
            .withWheelDiameter(Inches.of(4))
            .withClosedLoopController(0.4, 0, 0)
            //            .withFeedforward(new SimpleMotorFeedforward(0, 0.7, 0.1))
            .withGearing(driveGearing)
            .withStatorCurrentLimit(Amps.of(40))
            .withTelemetry("driveMotor", TelemetryVerbosity.HIGH);
    SmartMotorControllerConfig azimuthCfg =
        new SmartMotorControllerConfig(this)
            .withClosedLoopController(3.8476, 0, 0)
            .withContinuousWrapping(Radians.of(-Math.PI), Radians.of(Math.PI))
            .withGearing(azimuthGearing)
            .withStatorCurrentLimit(Amps.of(40))
            .withTelemetry("angleMotor", TelemetryVerbosity.HIGH);
    SmartMotorController driveSMC = new SparkWrapper(drive, DCMotor.getNEO(1), driveCfg);
    SmartMotorController azimuthSMC = new SparkWrapper(azimuth, DCMotor.getNEO(1), azimuthCfg);
    SwerveModuleConfig moduleConfig =
        new SwerveModuleConfig(driveSMC, azimuthSMC)
            .withAbsoluteEncoder(absoluteEncoder.getAbsolutePosition().asSupplier())
            .withTelemetry(moduleName, TelemetryVerbosity.HIGH)
            .withLocation(location)
            .withOptimization(true);
    return new SwerveModule(moduleConfig);
  }

  private final Pigeon2 gyro = new Pigeon2(14, new CANBus(CANPort.CAN_S0));

  public SwerveMechanism()
  {
    var fl = createModule(new SparkMax(CANPorts.fromBusId(1), 1, MotorType.kBrushless),
                          new SparkMax(CANPorts.fromBusId(1), 2, MotorType.kBrushless),
                          new CANcoder(3, new CANBus(CANPort.CAN_S0)),
                          "frontleft",
                          new Translation2d(Inches.of(24), Inches.of(24)));
    var fr = createModule(new SparkMax(CANPorts.fromBusId(1), 4, MotorType.kBrushless),
                          new SparkMax(CANPorts.fromBusId(1), 5, MotorType.kBrushless),
                          new CANcoder(6, new CANBus(CANPort.CAN_S0)),
                          "frontright",
                          new Translation2d(Inches.of(24), Inches.of(-24)));
    var bl = createModule(new SparkMax(CANPorts.fromBusId(1), 7, MotorType.kBrushless),
                          new SparkMax(CANPorts.fromBusId(1), 8, MotorType.kBrushless),
                          new CANcoder(9, new CANBus(CANPort.CAN_S0)),
                          "backleft",
                          new Translation2d(Inches.of(-24), Inches.of(24)));
    var br = createModule(new SparkMax(CANPorts.fromBusId(1), 10, MotorType.kBrushless),
                          new SparkMax(CANPorts.fromBusId(1), 11, MotorType.kBrushless),
                          new CANcoder(12, new CANBus(CANPort.CAN_S0)),
                          "backright",
                          new Translation2d(Inches.of(-24), Inches.of(-24)));
    SwerveDriveConfig config = new SwerveDriveConfig(this, fl, fr, bl, br)
        .withGyro(gyro::getRotation3d)
        .withMaximumChassisSpeed(MetersPerSecond.of(4), DegreesPerSecond.of(360))
        .withStartingPose(new Pose2d(0, 0, Rotation2d.fromDegrees(0)))
        .withTranslationController(new PIDController(1, 0, 0))
        .withRotationController(new PIDController(1, 0, 0));
    drive = new SwerveDrive(config);
  }

  public Command setRobotRelativeChassisSpeeds(ChassisVelocities speeds)
  {
    return run(coroutine -> {
      while (true) {
        drive.setRobotRelativeChassisSpeeds(speeds);
        coroutine.yield();
      }
    }).named("Swerve Set Robot Relative Chassis Speeds");
  }

  public Command driveToPose(Pose2d pose) {
    return drive.driveToPose(pose);
  }

  public Command driveRobotRelative(Supplier<ChassisVelocities> speedsSupplier)
  {
    return drive.drive(speedsSupplier);
  }

  /**
   * Drive with field relative {@link ChassisVelocities}. Runs until canceled.
   *
   * @param velocities Field relative {@link ChassisVelocities}, read every loop.
   * @return {@link Command} that drives the robot.
   */
  public Command driveFieldRelative(Supplier<ChassisVelocities> velocities) {
    return run(coroutine -> {
      while (true) {
        drive.setFieldRelativeChassisSpeeds(velocities.get());
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
  public Command driveInputStream() {
    return run(coroutine -> {
      while (true) {
        drive.setFieldRelativeChassisSpeeds(inputStream.get());
        coroutine.yield();
      }
    }).named("Swerve Drive Input Stream");
  }

  /**
   * Set the active driver input.
   *
   * @param inputStream Field relative {@link SwerveInputStream} driven by {@link #driveInputStream()}.
   */
  public void setInputStream(SwerveInputStream inputStream) {
    // Stop publishing the replaced stream's telemetry, e.g. when another teleop opmode is selected.
    if (this.inputStream != null) {
      this.inputStream.getTelemetry().ifPresent(SwerveInputStreamTelemetry::close);
    }
    this.inputStream = inputStream;
  }

  /**
   * Get the active driver input, so commands can read or adjust it.
   *
   * @return Active {@link SwerveInputStream}.
   */
  public SwerveInputStream getInputStream() {
    return inputStream;
  }

  public Command lock() {
    return run(coroutine -> {
      while (true) {
        drive.lockPose();
        coroutine.yield();
      }
    }).named("Swerve Lock");
  }

  public void periodic() {
    drive.updateTelemetry();
  }

  public void simulationPeriodic() {
    drive.simIterate();
    gyro.getSimState().setRawYaw(drive.getSimPose().getRotation().getRadians());
  }

  /**
   * Underlying YAMS {@link SwerveDrive}, for building a {@link SwerveInputStream}.
   *
   * @return {@link SwerveDrive} driven by this mechanism.
   */
  public SwerveDrive getSwerveDrive() {
    return drive;
  }

  public Pose2d getPose() {
    return drive.getPose();
  }

  public ChassisVelocities getFieldOrientedChassisSpeed()
  {
    return drive.getFieldRelativeSpeed();
  }

  public Rotation3d getGyroRotation3d() {
    return drive.getGyroRotation3d();
  }
}
