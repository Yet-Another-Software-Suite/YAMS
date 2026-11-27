// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package first.robot.mechanisms;

import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.DegreesPerSecond;
import static org.wpilib.units.Units.Inches;
import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.MetersPerSecond;
import static org.wpilib.units.Units.Rotations;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
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
import org.wpilib.smartdashboard.Field2d;
import org.wpilib.units.measure.Angle;
import yams.commands3.config.SmartMotorControllerConfig;
import yams.commands3.config.SwerveDriveConfig;
import yams.commands3.swerve.SwerveDrive;
import yams.commands3.swerve.SwerveInputStream;
import yams.core.gearing.MechanismGearing;
import yams.core.mechanisms.config.SwerveModuleConfig;
import yams.core.mechanisms.swerve.SwerveModule;
import yams.core.motorcontrollers.SmartMotorController;
import yams.core.motorcontrollers.local.SparkWrapper;
import yams.core.telemetry.enums.TelemetryVerbosity;

/** The swerve drive from the Swerve Drive tutorial, with the pose and field velocity the shot is worked out from. */
public class SwerveMechanism implements Mechanism {

  private final SwerveDrive drive;

  // The gyro: a Pigeon2 with CAN ID 14.
  private final Pigeon2 gyro = new Pigeon2(14, new CANBus(CANPort.CAN_S0));

  /** The config of one drive motor. Drive motors are velocity controlled. */
  private SmartMotorControllerConfig buildDriveCfg() {
    return new SmartMotorControllerConfig(this)
        // The wheel diameter turns wheel rotations into distance driven.
        .withWheelDiameter(Inches.of(4))
        // kP trims the velocity error that is left after the feedforward.
        .withClosedLoopController(0.3, 0, 0)
        // Gearing from the motor rotor to the wheel: 6.75:1, an MK4 L2 module.
        .withGearing(new MechanismGearing(6.75))
        // kV, in volts per wheel rotation per second, does most of the work of holding a speed.
        // 0.85 is a placeholder, about right for a NEO on 6.75:1.
        .withFeedforward(new SimpleMotorFeedforward(0, 0.85))
        // Set this to true if the wheel spins backward when it is told to drive forward.
        .withMotorInverted(false)
        .withStatorCurrentLimit(Amps.of(40))
        .withTelemetry("driveMotor", TelemetryVerbosity.HIGH);
  }

  /** The config of one azimuth (steering) motor. Azimuth motors are position controlled. */
  private SmartMotorControllerConfig buildAzimuthCfg() {
    return new SmartMotorControllerConfig(this)
        // kP, in volts per rotation of angle error. A placeholder.
        .withClosedLoopController(4, 0, 0)
        // The wheel turns all the way round, so -180 and +180 degrees are the same angle.
        .withContinuousWrapping(Rotations.of(-0.5), Rotations.of(0.5))
        // Gearing from the motor rotor to the module's steering axis: 12.8:1, an MK4 module.
        .withGearing(new MechanismGearing(12.8))
        // Set this to true if the wheel turns clockwise, seen from above, when told to turn counter-clockwise.
        .withMotorInverted(false)
        // The azimuth motor rarely works hard, so it gets a lower limit.
        .withStatorCurrentLimit(Amps.of(20))
        .withTelemetry("angleMotor", TelemetryVerbosity.HIGH);
  }

  /** All four modules share their gearing and gains, so one method builds each of them. */
  private SwerveModule createModule(SparkMax drive, SparkMax azimuth, CANcoder absoluteEncoder,
                                    Angle encoderOffset, String moduleName, Translation2d location) {
    SmartMotorController driveMotor = new SparkWrapper(drive, DCMotor.getNEO(1), buildDriveCfg());
    SmartMotorController azimuthMotor = new SparkWrapper(azimuth, DCMotor.getNEO(1), buildAzimuthCfg());

    return new SwerveModule(new SwerveModuleConfig(driveMotor, azimuthMotor)
        // The CANcoder tells the module which way it points when the robot turns on.
        .withAbsoluteEncoder(absoluteEncoder.getAbsolutePosition().asSupplier())
        // The CANcoder's raw reading when the wheel points forward with its bevel gear to the left.
        // Every module has its own. Measure yours on the real robot.
        .withAbsoluteEncoderOffset(encoderOffset)
        .withTelemetry(moduleName, TelemetryVerbosity.HIGH)
        // Where the module is on the robot: +X is forward, +Y is left.
        .withLocation(location)
        // Turn the module at most 90 degrees, reversing the wheel instead of turning further.
        .withOptimization(true));
  }

  public SwerveMechanism() {
    // Each module is 24 inches forward or back and 24 inches left or right of the robot center.
    // CAN IDs are grouped by module: drive, azimuth, CANcoder.
    var frontLeft = createModule(new SparkMax(CANPort.CAN_S0, 1, MotorType.kBrushless),
                                 new SparkMax(CANPort.CAN_S0, 2, MotorType.kBrushless),
                                 new CANcoder(3, new CANBus(CANPort.CAN_S0)), Degrees.of(0), "frontleft",
                                 new Translation2d(Inches.of(24), Inches.of(24)));
    var frontRight = createModule(new SparkMax(CANPort.CAN_S0, 4, MotorType.kBrushless),
                                  new SparkMax(CANPort.CAN_S0, 5, MotorType.kBrushless),
                                  new CANcoder(6, new CANBus(CANPort.CAN_S0)), Degrees.of(0), "frontright",
                                  new Translation2d(Inches.of(24), Inches.of(-24)));
    var backLeft = createModule(new SparkMax(CANPort.CAN_S0, 7, MotorType.kBrushless),
                                new SparkMax(CANPort.CAN_S0, 8, MotorType.kBrushless),
                                new CANcoder(9, new CANBus(CANPort.CAN_S0)), Degrees.of(0), "backleft",
                                new Translation2d(Inches.of(-24), Inches.of(24)));
    var backRight = createModule(new SparkMax(CANPort.CAN_S0, 10, MotorType.kBrushless),
                                 new SparkMax(CANPort.CAN_S0, 11, MotorType.kBrushless),
                                 new CANcoder(12, new CANBus(CANPort.CAN_S0)), Degrees.of(0), "backright",
                                 new Translation2d(Inches.of(-24), Inches.of(-24)));

    SwerveDriveConfig config = new SwerveDriveConfig(this, frontLeft, frontRight, backLeft, backRight)
        // The gyro's attitude. Its yaw is the heading used for field relative driving.
        .withGyro(gyro::getRotation3d)
        // Start about 3.3 m from the target, inside the 1.5 to 4.5 m the shot tables cover, so
        // the shot works as soon as the simulation starts.
        .withStartingPose(new Pose2d(Meters.of(2), Meters.of(2), Rotation2d.ZERO))
        .withTranslationController(new PIDController(1, 0, 0))
        // Turns the robot to face the aim point. A module holds still when its wheel would turn
        // slower than its velocity deadband, 0.1 m/s by default, so a heading controller stops
        // turning once its output gets that small: with a kP of 1 that is about 6.6 degrees
        // short, outside the 2 degree aim lock. A kP of 5 stops within about 1.3 degrees. Tune it
        // once the robot drives well: a heading that lags behind the aim point while driving
        // past the target misses.
        .withRotationController(new PIDController(5, 0, 0))
        // Publishes under NT:/Mechanisms/swerve. HIGH also publishes the live tuning keys.
        .withTelemetry("swerve", TelemetryVerbosity.HIGH);

    drive = new SwerveDrive(config);
  }

  /** Drive with field relative chassis speeds. */
  public Command driveFieldRelative(Supplier<ChassisVelocities> speedsSupplier) {
    return run(coroutine -> {
      while (true) {
        drive.setFieldRelativeChassisSpeeds(speedsSupplier.get());
        coroutine.yield();
      }
    }).named("Field Oriented Drive");
  }

  /** Make the way the robot faces now "forward" for field relative driving. */
  public Command zeroGyro() {
    return run(coroutine -> {
      drive.zeroGyro();
    }).named("Zero Gyro");
  }

  /** Turn three controller axes into field relative chassis speeds. */
  public SwerveInputStream getChassisSpeedsSupplier(DoubleSupplier translationX, DoubleSupplier translationY,
                                                    DoubleSupplier rotation) {
    return SwerveInputStream.of(drive, translationX, translationY)
        .withControllerRotationAxis(rotation)
        // What a full stick means.
        .withMaximumLinearVelocity(MetersPerSecond.of(2))
        .withMaximumAngularVelocity(DegreesPerSecond.of(360))
        // Ignore a stick that is barely off center.
        .withDeadband(0.05)
        // Cube the sticks, for finer control at low speed.
        .withCubeTranslationControllerAxis()
        .withCubeRotationControllerAxis()
        // Publish the stream's inputs and outputs, and allow live tuning of it.
        .withTelemetry("Driver", TelemetryVerbosity.HIGH);
  }

  /** Where the robot is on the field. */
  public Pose2d getPose() {
    return drive.getPose();
  }

  /** How fast the robot is moving on the field. */
  public ChassisVelocities getFieldRelativeSpeed() {
    return drive.getFieldRelativeSpeed();
  }

  /** The drive's field, published with its telemetry. Draw the target and the aim point on it. */
  public Field2d getField2d() {
    return drive.getField2d();
  }

  /** Call this from Robot.robotPeriodic(), before the scheduler runs. */
  public void periodic() {
    // Updates the pose estimate, the telemetry and the drive's Field2d.
    drive.updateTelemetry();
  }

  /** Robot.simulationInit() runs this in its own Notifier, every robot loop. */
  public void simulationPeriodic() {
    // Runs the physics simulation of all eight motors.
    drive.simIterate();
    // In simulation the drive's heading comes from the simulated robot, not the Pigeon2. This
    // keeps the Pigeon2's own reading in step, for code that reads it directly.
    gyro.getSimState().setRawYaw(drive.getSimPose().getRotation().getMeasure());
  }
}
