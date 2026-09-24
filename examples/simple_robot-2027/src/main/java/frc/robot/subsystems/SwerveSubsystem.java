// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package frc.robot.subsystems;

import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.DegreesPerSecond;
import static org.wpilib.units.Units.Inches;
import static org.wpilib.units.Units.MetersPerSecond;
import static org.wpilib.units.Units.Radians;
import static org.wpilib.units.Units.RotationsPerSecond;

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
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.math.system.DCMotor;
import org.wpilib.units.measure.Angle;
import org.wpilib.smartdashboard.Field2d;
import org.wpilib.tunable.Tunables;
import org.wpilib.command2.Command;
import org.wpilib.command2.SubsystemBase;
import org.wpilib.command2.button.CommandNiDsXboxController;
import java.util.function.Supplier;
import yams.core.gearing.GearBox;
import yams.core.gearing.MechanismGearing;
import yams.commands2.config.SwerveDriveConfig;
import yams.core.mechanisms.config.SwerveModuleConfig;
import yams.commands2.swerve.SwerveDrive;
import yams.core.mechanisms.swerve.SwerveModule;
import yams.core.mechanisms.swerve.utility.SwerveInputStream;
import yams.core.motorcontrollers.SmartMotorController;
import yams.commands2.config.SmartMotorControllerConfig;
import yams.core.motorcontrollers.local.SparkWrapper;

public class SwerveSubsystem extends SubsystemBase {
  private final SwerveDrive drive;
  private final Field2d field = new Field2d();

  public SwerveModule createModule(
      SparkMax drive,
      SparkMax azimuth,
      CANcoder absoluteEncoder,
      String moduleName,
      Translation2d location) {
    MechanismGearing driveGearing = new MechanismGearing(GearBox.fromReductionStages(6.75));
    MechanismGearing azimuthGearing = new MechanismGearing(GearBox.fromReductionStages(12.8));
    SmartMotorControllerConfig driveCfg =
        (SmartMotorControllerConfig) new SmartMotorControllerConfig(this)
            .withWheelDiameter(Inches.of(4))
            .withClosedLoopController(0.4, 0, 0)
            //            .withFeedforward(new SimpleMotorFeedforward(0, 0.7, 0.1))
            .withGearing(driveGearing)
            .withStatorCurrentLimit(Amps.of(40))
            .withTelemetry("driveMotor", SmartMotorControllerConfig.TelemetryVerbosity.HIGH);
    SmartMotorControllerConfig azimuthCfg =
        (SmartMotorControllerConfig) new SmartMotorControllerConfig(this)
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

  private final Pigeon2 gyro = new Pigeon2(14, new CANBus(CANPort.CAN_S0));

  public SwerveSubsystem()
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
    SwerveDriveConfig config = (SwerveDriveConfig) new SwerveDriveConfig(this, fl, fr, bl, br)
        .withGyro(gyro.getYaw().asSupplier())
        .withMaximumChassisSpeed(MetersPerSecond.of(4), RotationsPerSecond.of(360))
        .withStartingPose(new Pose2d(0, 0, Rotation2d.fromDegrees(0)))
        .withTranslationController(new PIDController(1, 0, 0))
        .withRotationController(new PIDController(1, 0, 0));
    drive = new SwerveDrive(config);

    Tunables.publish("Field", field);
  }

  public Command setRobotRelativeChassisSpeeds(ChassisVelocities speeds)
  {
    return run(() -> drive.setRobotRelativeChassisSpeeds(speeds));
  }

  public Command driveToPose(Pose2d pose) {
    return drive.driveToPose(pose);
  }

  public Command driveRobotRelative(Supplier<ChassisVelocities> speedsSupplier)
  {
    return drive.drive(speedsSupplier);
  }

  /**
   * Drive the robot field-relative using a driver controller, converting joystick axes into {@link
   * ChassisVelocities} via {@link SwerveInputStream}.
   *
   * @param controller Driver controller to read translation/rotation axes from.
   * @return {@link Command} that drives the robot while scheduled.
   */
  public Command driveWithJoystick(CommandNiDsXboxController controller) {
    SwerveInputStream inputStream =
        SwerveInputStream.of(drive, () -> -controller.getLeftY(), () -> -controller.getLeftX())
            .withControllerRotationAxis(() -> -controller.getRightX())
            .withMaximumLinearVelocity(MetersPerSecond.of(4))
            .withMaximumAngularVelocity(DegreesPerSecond.of(360))
            .withDeadband(0.05)
            .withCubeTranslationControllerAxis()
            .withAllianceRelativeControl();

    return drive.drive(
        () -> inputStream.get().toRobotRelative(new Rotation2d(drive.getGyroAngle())));
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

  public ChassisVelocities getFieldOrientedChassisSpeed()
  {
    return drive.getFieldRelativeSpeed();
  }

  public Angle getGyroAngle() {
    return drive.getGyroAngle();
  }
}
