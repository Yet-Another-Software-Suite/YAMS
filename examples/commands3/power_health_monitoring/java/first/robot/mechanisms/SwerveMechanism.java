// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package first.robot.mechanisms;

import static first.robot.Constants.PowerHealthConstants.MONITOR_CONFIG;
import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Inches;
import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.Rotations;
import static org.wpilib.units.Units.Seconds;
import static org.wpilib.units.Units.Volts;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import first.robot.PowerHealthMonitor;
import java.util.ArrayList;
import java.util.List;
import java.util.function.DoubleSupplier;
import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;
import org.wpilib.hardware.bus.CANPort;
import org.wpilib.math.controller.PIDController;
import org.wpilib.math.controller.SimpleMotorFeedforward;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.math.system.DCMotor;
import org.wpilib.networktables.DoublePublisher;
import org.wpilib.networktables.IntegerPublisher;
import org.wpilib.networktables.NetworkTable;
import org.wpilib.networktables.NetworkTableInstance;
import org.wpilib.networktables.StringPublisher;
import org.wpilib.system.RobotController;
import org.wpilib.system.Timer;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.Current;
import yams.commands3.config.SmartMotorControllerConfig;
import yams.commands3.config.SwerveDriveConfig;
import yams.commands3.swerve.SwerveDrive;
import yams.core.gearing.MechanismGearing;
import yams.core.mechanisms.config.SwerveModuleConfig;
import yams.core.mechanisms.swerve.SwerveModule;
import yams.core.motorcontrollers.SmartMotorController;
import yams.core.motorcontrollers.local.SparkWrapper;
import yams.core.telemetry.enums.TelemetryVerbosity;

/** YAMS SwerveDrive (Commands v3) paired with a standalone PowerHealthMonitor. */
public class SwerveMechanism implements Mechanism {
  private static final Pose2d STARTING_POSE = new Pose2d(Meters.of(2), Meters.of(2), Rotation2d.ZERO);
  private static final double MAX_LINEAR_MPS = 4.5;
  private static final double MAX_OMEGA_RAD_PER_SEC = Math.toRadians(360.0);

  private static final Translation2d[] MODULE_LOCATIONS = {
    new Translation2d(Inches.of(12), Inches.of(12)),
    new Translation2d(Inches.of(12), Inches.of(-12)),
    new Translation2d(Inches.of(-12), Inches.of(12)),
    new Translation2d(Inches.of(-12), Inches.of(-12))
  };

  private final SwerveDrive drive;
  private final Pigeon2 gyro = new Pigeon2(14, new CANBus(CANPort.CAN_S0));
  private final List<SmartMotorController> allMotors = new ArrayList<>();
  private final PowerHealthMonitor powerMonitor = new PowerHealthMonitor(MONITOR_CONFIG);

  private final DoublePublisher voltagePub;
  private final DoublePublisher currentPub;
  private final DoublePublisher throttleScalePub;
  private final DoublePublisher socPub;
  private final IntegerPublisher brownoutCountPub;
  private final StringPublisher statePub;

  private SmartMotorControllerConfig buildDriveCfg() {
    return new SmartMotorControllerConfig(this)
        .withWheelDiameter(Inches.of(4))
        .withClosedLoopController(0.3, 0, 0)
        .withGearing(new MechanismGearing(6.75))
        .withFeedforward(new SimpleMotorFeedforward(0, 0.85))
        .withMotorInverted(false)
        .withStatorCurrentLimit(Amps.of(40))
        .withTelemetry("driveMotor", TelemetryVerbosity.LOW);
  }

  private SmartMotorControllerConfig buildAzimuthCfg() {
    return new SmartMotorControllerConfig(this)
        .withClosedLoopController(4, 0, 0)
        .withContinuousWrapping(Rotations.of(-0.5), Rotations.of(0.5))
        .withGearing(new MechanismGearing(12.8))
        .withMotorInverted(false)
        .withStatorCurrentLimit(Amps.of(20))
        .withTelemetry("angleMotor", TelemetryVerbosity.LOW);
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
    allMotors.add(driveMotor);
    allMotors.add(azimuthMotor);

    return new SwerveModule(
        new SwerveModuleConfig(driveMotor, azimuthMotor)
            .withAbsoluteEncoder(absoluteEncoder.getAbsolutePosition().asSupplier())
            .withAbsoluteEncoderOffset(encoderOffset)
            .withTelemetry(moduleName, TelemetryVerbosity.LOW)
            .withLocation(location)
            .withOptimization(true));
  }

  public SwerveMechanism() {
    CANBus bus = new CANBus(CANPort.CAN_S0);
    SwerveModule fl =
        createModule(
            new SparkMax(CANPort.CAN_S0, 1, MotorType.kBrushless),
            new SparkMax(CANPort.CAN_S0, 2, MotorType.kBrushless),
            new CANcoder(3, bus),
            Degrees.of(0),
            "frontleft",
            MODULE_LOCATIONS[0]);
    SwerveModule fr =
        createModule(
            new SparkMax(CANPort.CAN_S0, 4, MotorType.kBrushless),
            new SparkMax(CANPort.CAN_S0, 5, MotorType.kBrushless),
            new CANcoder(6, bus),
            Degrees.of(0),
            "frontright",
            MODULE_LOCATIONS[1]);
    SwerveModule bl =
        createModule(
            new SparkMax(CANPort.CAN_S0, 7, MotorType.kBrushless),
            new SparkMax(CANPort.CAN_S0, 8, MotorType.kBrushless),
            new CANcoder(9, bus),
            Degrees.of(0),
            "backleft",
            MODULE_LOCATIONS[2]);
    SwerveModule br =
        createModule(
            new SparkMax(CANPort.CAN_S0, 10, MotorType.kBrushless),
            new SparkMax(CANPort.CAN_S0, 11, MotorType.kBrushless),
            new CANcoder(12, bus),
            Degrees.of(0),
            "backright",
            MODULE_LOCATIONS[3]);

    SwerveDriveConfig cfg =
        new SwerveDriveConfig(this, fl, fr, bl, br)
            .withGyro(gyro::getRotation3d)
            .withStartingPose(STARTING_POSE)
            .withTranslationController(new PIDController(1, 0, 0))
            .withRotationController(new PIDController(5, 0, 0))
            .withTelemetry("swerve", TelemetryVerbosity.LOW);
    drive = new SwerveDrive(cfg);

    NetworkTable table = NetworkTableInstance.getDefault().getTable("PowerHealth");
    voltagePub = table.getDoubleTopic("VoltageVolts").publish();
    currentPub = table.getDoubleTopic("TotalCurrentAmps").publish();
    throttleScalePub = table.getDoubleTopic("ThrottleScale").publish();
    socPub = table.getDoubleTopic("RemainingSoC").publish();
    brownoutCountPub = table.getIntegerTopic("BrownoutCount").publish();
    statePub = table.getStringTopic("State").publish();
  }

  public Current getTotalMotorCurrent() {
    double totalAmps = 0.0;
    for (SmartMotorController motor : allMotors) {
      double amps = motor.getSupplyCurrent().orElse(motor.getStatorCurrent()).in(Amps);
      if (Double.isFinite(amps)) {
        totalAmps += Math.abs(amps);
      }
    }
    return Amps.of(totalAmps);
  }

  public PowerHealthMonitor getPowerMonitor() {
    return powerMonitor;
  }

  public Command driveFieldRelative(
      DoubleSupplier vxNorm, DoubleSupplier vyNorm, DoubleSupplier omegaNorm) {
    return run(
            coroutine -> {
              while (true) {
                double scale = powerMonitor.getThrottleScale();
                drive.setFieldRelativeChassisSpeeds(
                    new ChassisVelocities(
                        vxNorm.getAsDouble() * MAX_LINEAR_MPS * scale,
                        vyNorm.getAsDouble() * MAX_LINEAR_MPS * scale,
                        omegaNorm.getAsDouble() * MAX_OMEGA_RAD_PER_SEC * scale));
                coroutine.yield();
              }
            })
        .named("Drive with Power Health Throttle");
  }

  public void periodic() {
    drive.updateTelemetry();
    Current totalCurrent = getTotalMotorCurrent();
    powerMonitor.update(
        Seconds.of(Timer.getTimestamp()),
        RobotController.getMeasureBatteryVoltage(),
        totalCurrent);

    voltagePub.set(powerMonitor.getLastValidVoltage().in(Volts));
    currentPub.set(totalCurrent.in(Amps));
    throttleScalePub.set(powerMonitor.getThrottleScale());
    socPub.set(powerMonitor.getRemainingStateOfCharge());
    brownoutCountPub.set(powerMonitor.getBrownoutCount());
    statePub.set(powerMonitor.getState().name());
  }

  public void simulationPeriodic() {
    drive.simIterate();
    gyro.getSimState().setRawYaw(drive.getSimPose().getRotation().getMeasure());
  }
}
