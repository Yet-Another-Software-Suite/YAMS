// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package yams.mechanisms.config;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Microsecond;
import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import java.util.Optional;
import java.util.OptionalDouble;
import java.util.function.Supplier;
import yams.math.AntiTipping;
import yams.math.DerivativeTimeFilter;
import yams.mechanisms.swerve.SwerveDrive;
import yams.mechanisms.swerve.SwerveModule;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.telemetry.SwerveDriveTelemetryConfig;

/**
 * Swerve Drive Configuration
 *
 * <h2>Configuration Example</h2>
 *
 * <p>The following example builds a four-module swerve drive using NEO motors controlled by {@code SparkMax} controllers and CANcoder absolute encoders. Modules are ordered <b>clockwise from
 * front-left</b>: FL, FR, BL, BR. Each module needs its position relative to the robot centre of
 * rotation (X = forward, Y = left, in metres).
 *
 * <pre>{@code
 * // -- shared constants -------------------------------------------------------
 * MechanismGearing driveGearing   = new MechanismGearing(6.75);
 * MechanismGearing azimuthGearing = new MechanismGearing(12.8);
 * Distance         wheelDiameter  = Inches.of(4);
 * double           maxSpeedMps    = 4.5; // metres per second
 *
 * // -- helper that builds one module -----------------------------------------
 * SwerveModule buildModule(SubsystemBase subsystem,
 *                          SparkMax       driveMotor,
 *                          SparkMax       azimuthMotor,
 *                          CANcoder       encoder,
 *                          String         name,
 *                          Distance       front,
 *                          Distance       left) {
 *
 *     SmartMotorControllerConfig driveCfg = new SmartMotorControllerConfig(subsystem)
 *         .withWheelDiameter(wheelDiameter)
 *         .withGearing(driveGearing)
 *         .withClosedLoopController(0.3, 0, 0)
 *         .withFeedforward(new SimpleMotorFeedforward(0,
 *             12.0 / (maxSpeedMps / wheelDiameter.in(Meters)), 0.01))
 *         .withStatorCurrentLimit(Amps.of(40))
 *         .withTelemetry("driveMotor", TelemetryVerbosity.HIGH);
 *
 *     SmartMotorControllerConfig azimuthCfg = new SmartMotorControllerConfig(subsystem)
 *         .withGearing(azimuthGearing)
 *         .withClosedLoopController(1.0, 0, 0)
 *         .withFeedforward(new SimpleMotorFeedforward(0, 1))
 *         .withStatorCurrentLimit(Amps.of(20))
 *         .withTelemetry("angleMotor", TelemetryVerbosity.HIGH);
 *
 *     SmartMotorController driveSMC   = new SparkWrapper(driveMotor,   DCMotor.getNEO(1), driveCfg);
 *     SmartMotorController azimuthSMC = new SparkWrapper(azimuthMotor, DCMotor.getNEO(1), azimuthCfg);
 *
 *     SwerveModuleConfig moduleConfig = new SwerveModuleConfig(driveSMC, azimuthSMC)
 *         .withAbsoluteEncoder(encoder.getAbsolutePosition().asSupplier())
 *         .withLocation(front, left)
 *         .withOptimization(true)
 *         .withTelemetry(name, TelemetryVerbosity.HIGH);
 *
 *     return new SwerveModule(moduleConfig);
 * }
 *
 * // -- build all four modules (trackwidth = 0.6 m, wheelbase = 0.6 m) -------
 * Distance halfTrack    = Meters.of(0.3);  // half of 0.6 m trackwidth
 * Distance halfWheelbase = Meters.of(0.3); // half of 0.6 m wheelbase
 *
 * SwerveModule fl = buildModule(this,
 *     new SparkMax(1, MotorType.kBrushless), new SparkMax(2, MotorType.kBrushless),
 *     new CANcoder(3), "frontleft",  halfWheelbase,  halfTrack);
 * SwerveModule fr = buildModule(this,
 *     new SparkMax(4, MotorType.kBrushless), new SparkMax(5, MotorType.kBrushless),
 *     new CANcoder(6), "frontright", halfWheelbase, halfTrack.unaryMinus());
 * SwerveModule bl = buildModule(this,
 *     new SparkMax(7, MotorType.kBrushless), new SparkMax(8, MotorType.kBrushless),
 *     new CANcoder(9), "backleft",  halfWheelbase.unaryMinus(),  halfTrack);
 * SwerveModule br = buildModule(this,
 *     new SparkMax(10, MotorType.kBrushless), new SparkMax(11, MotorType.kBrushless),
 *     new CANcoder(12), "backright", halfWheelbase.unaryMinus(), halfTrack.unaryMinus());
 *
 * // -- assemble the SwerveDriveConfig ----------------------------------------
 * Pigeon2 gyro = new Pigeon2(14);
 *
 * SwerveDriveConfig config = new SwerveDriveConfig(this, fl, fr, bl, br)
 *     .withGyro(gyro.getYaw().asSupplier())
 *     .withMaximumChassisSpeed(MetersPerSecond.of(maxSpeedMps), DegreesPerSecond.of(360))
 *     .withTranslationController(new PIDController(1.0, 0, 0))
 *     .withRotationController(new PIDController(1.0, 0, 0))
 *     .withStartingPose(new Pose2d());
 * }</pre>
 */
public class SwerveDriveConfig {
  /** {@link SwerveModule}s for the {@link SwerveDrive}. */
  private SwerveModule[] modules;

  /** Telemetry name for the {@link SwerveDrive}. */
  private String telemetryName = "swerve";

  /** Telemetry verbosity */
  private Optional<TelemetryVerbosity> telemetryVerbosity = Optional.empty();

  /**
   * User specified {@link SwerveDriveTelemetryConfig}, takes precedence over {@link #telemetryVerbosity} if present.
   */
  private Optional<SwerveDriveTelemetryConfig> specifiedTelemetryConfig = Optional.empty();

  /** Supplier of the gyro's attitude, see {@link #withGyro(Supplier)}. */
  private Optional<Supplier<Rotation3d>> gyroSupplier = Optional.empty();

  /** Axis of the gyro that points up through the robot, see {@link #withGyroHeadingAxis(GyroAxis)}. */
  private GyroAxis gyroHeadingAxis = GyroAxis.YAW;

  /** Anti-tipping correction, see {@link #withAntiTipping(AntiTipping)}. */
  private Optional<AntiTipping> antiTipping = Optional.empty();

  /** Module state optimization for every module, see {@link #withModuleStateOptimization(boolean)}. */
  private Optional<Boolean> moduleStateOptimization = Optional.empty();

  /** Gyro angular velocity supplier. */
  private Optional<Supplier<AngularVelocity>> gyroAngularVelocitySupplier = Optional.empty();

  /**
   * Derives the gyro angular velocity from the gyro angle ({@link #getGyroRotation3d()}) when {@link #gyroAngularVelocitySupplier} is not configured, in both simulation and real robot code.
   */
  private final DerivativeTimeFilter gyroAngularVelocityFilter = new DerivativeTimeFilter(Milliseconds.of(20));

  /** Gyro angle with its wraps removed, in radians, for deriving the gyro angular velocity. */
  private double continuousGyroAngleRadians = 0;

  /** Whether {@link #continuousGyroAngleRadians} has been read yet. */
  private boolean continuousGyroAngleRead = false;

  /**
   * Alert shown once if {@link #angularVelocitySkewCorrection(ChassisSpeeds)} runs without a {@link #gyroAngularVelocitySupplier} configured, warning that the gyro angular velocity is being
   * derived from the gyro angle instead.
   */
  private Alert noGyroAngularVelocitySupplierAlert = null;

  /** Gyro offset. */
  private Optional<Angle> gyroOffset = Optional.empty();

  /** Gyro inverted. */
  private boolean gyroInverted = false;

  /** Starting pose on the field. */
  private Pose2d initialPose = new Pose2d();

  /** Maximum speed of the chassis. */
  private Optional<LinearVelocity> maximumChassisLinearVelocity = Optional.empty();

  /** Maximum angular speed of the chassis. */
  private Optional<AngularVelocity> maximumChassisAngularVelocity = Optional.empty();

  /** Maximum speed of the modules. */
  private Optional<LinearVelocity> maximumModuleLinearVelocity = Optional.empty();

  /** Discretization time for the pose estimation. */
  private Optional<Time> discretizationSeconds = Optional.empty();

  /** Angular velocity scale factor. */
  private OptionalDouble angularVelocityScaleFactor = OptionalDouble.empty();

  /** Center of Rotation */
  private Optional<Translation2d> centerOfRotation = Optional.empty();

  /** Translation PID controller. */
  private Optional<PIDController> translationController = Optional.empty();

  /** Rotation PID controller. */
  private Optional<PIDController> rotationController = Optional.empty();

  /** Simulated translation PID controller. */
  private Optional<PIDController> simTranslationController = Optional.empty();

  /** Simulated rotation PID controller. */
  private Optional<PIDController> simRotationController = Optional.empty();

  /** Discretization time for the pose estimation. */
  private Optional<Time> simDiscretizationSeconds = Optional.empty();

  /** Angular velocity scale factor. */
  private OptionalDouble simAngularVelocityScaleFactor = OptionalDouble.empty();

  /** Swerve drive subsystem. */
  private Subsystem subsystem;

  /**
   * Create the {@link SwerveDriveConfig} for the {@link SwerveDrive}
   *
   * @param modules         {@link SwerveModule}s for the {@link SwerveDrive}
   * @param swerveSubsystem SwerveDrive subsystem.
   */
  public SwerveDriveConfig(Subsystem swerveSubsystem, SwerveModule... modules) {
    subsystem = swerveSubsystem;
    this.modules = modules;
  }

  /**
   * Create the {@link SwerveDriveConfig} for the {@link SwerveDrive}
   *
   * @implNote Must define a Subsystem with {@link #withSubsystem(Subsystem)} and modules with
   *           {@link #withModules(SwerveModule...)}
   */
  public SwerveDriveConfig() {
  }

  private SwerveDriveConfig(SwerveDriveConfig cfg) {
    this.telemetryVerbosity = cfg.telemetryVerbosity;
    this.specifiedTelemetryConfig = cfg.specifiedTelemetryConfig;
    this.initialPose = cfg.initialPose;
    this.maximumChassisLinearVelocity = cfg.maximumChassisLinearVelocity;
    this.maximumChassisAngularVelocity = cfg.maximumChassisAngularVelocity;
    this.maximumModuleLinearVelocity = cfg.maximumModuleLinearVelocity;
    this.discretizationSeconds = cfg.discretizationSeconds;
    this.angularVelocityScaleFactor = cfg.angularVelocityScaleFactor;
    this.centerOfRotation = cfg.centerOfRotation;
    this.translationController = cfg.translationController;
    this.rotationController = cfg.rotationController;
    this.simTranslationController = cfg.simTranslationController;
    this.simRotationController = cfg.simRotationController;
    this.simDiscretizationSeconds = cfg.simDiscretizationSeconds;
    this.simAngularVelocityScaleFactor = cfg.simAngularVelocityScaleFactor;
    this.gyroHeadingAxis = cfg.gyroHeadingAxis;
    this.antiTipping = cfg.antiTipping;
    this.moduleStateOptimization = cfg.moduleStateOptimization;
    // Intentionally not copying these, as they are not user-configurable.
    //    this.gyroSupplier = cfg.gyroSupplier;
    //    this.gyroAngularVelocitySupplier = cfg.gyroAngularVelocitySupplier;
    //    this.gyroOffset = cfg.gyroOffset;
    //    this.gyroInverted = cfg.gyroInverted;
    //    this.telemetryName = cfg.telemetryName;
    //    this.modules = cfg.modules;
    //    this.subsystem = cfg.subsystem;
    //    this.noGyroAngularVelocitySupplierAlert = cfg.noGyroAngularVelocitySupplierAlert;
  }

  /**
   * Clone the {@link SwerveDriveConfig} without modules, subsystem, telemetry name, gyro supplier,
   * gyro angular velocity supplier, gyro offset, and gyro inversion.
   *
   * @return New {@link SwerveDriveConfig}
   */
  public SwerveDriveConfig clone() {
    return new SwerveDriveConfig(this);
  }

  /**
   * Define a {@link Subsystem} for the {@link SwerveDrive}
   *
   * @param subsystem {@link Subsystem} for the {@link SwerveDrive}
   * @return {@link SwerveDriveConfig} for chaining.
   */
  public SwerveDriveConfig withSubsystem(Subsystem subsystem) {
    this.subsystem = subsystem;
    return this;
  }

  /**
   * Set the {@link SwerveModule}s for the {@link SwerveDrive}.
   *
   * @param modules {@link SwerveModule}s for the {@link SwerveDrive}.
   * @return {@link SwerveDriveConfig} for chaining.
   */
  public SwerveDriveConfig withModules(SwerveModule... modules) {
    this.modules = modules;
    return this;
  }

  /**
   * Set the translation PID controller.
   *
   * @param controller {@link PIDController} for the translation, input units are meters.
   * @return {@link SwerveDriveConfig} for chaining.
   */
  public SwerveDriveConfig withSimTranslationController(PIDController controller) {
    simTranslationController = Optional.ofNullable(controller);
    return this;
  }

  /**
   * Set the rotation PID controller.
   *
   * @param controller {@link PIDController} for the rotation, input units are radians.
   * @return {@link SwerveDriveConfig} for chaining.
   */
  public SwerveDriveConfig withSimRotationController(PIDController controller) {
    if (controller != null) {
      controller.enableContinuousInput(-Math.PI, Math.PI);
    }
    simRotationController = Optional.ofNullable(controller);
    return this;
  }

  /**
   * Set the translation PID controller.
   *
   * @param controller {@link PIDController} for the translation, input units are meters.
   * @return {@link SwerveDriveConfig} for chaining.
   */
  public SwerveDriveConfig withTranslationController(PIDController controller) {
    translationController = Optional.ofNullable(controller);
    return this;
  }

  /**
   * Set the rotation PID controller.
   *
   * @param controller {@link PIDController} for the rotation, input units are radians.
   * @return {@link SwerveDriveConfig} for chaining.
   */
  public SwerveDriveConfig withRotationController(PIDController controller) {
    if (controller != null) {
      controller.enableContinuousInput(-Math.PI, Math.PI);
    }
    rotationController = Optional.ofNullable(controller);
    return this;
  }

  /**
   * Set the center of rotation; 0,0 is the center of the robot.
   *
   * @param centerOfRotation {@link Translation2d} of the center of rotation in Meters, X is
   *                         forward, Y is left.
   * @return {@link SwerveDriveConfig} for chaining.
   */
  public SwerveDriveConfig withCenterOfRotation(Translation2d centerOfRotation) {
    this.centerOfRotation = Optional.ofNullable(centerOfRotation);
    return this;
  }

  /**
   * Set the center of rotation; 0,0 is the center of the robot.
   *
   * @param forward Forward distance from the center of robot.
   * @param left    Left distance from the center of robot.
   * @return {@link SwerveDriveConfig} for chaining.
   */
  public SwerveDriveConfig withCenterOfRotation(Distance forward, Distance left) {
    this.centerOfRotation = Optional.ofNullable(new Translation2d(forward, left));
    return this;
  }

  /**
   * Set the discretization time for the pose estimation.
   *
   * @param dt Discretization time for the pose estimation.
   * @return {@link SwerveDriveConfig} for chaining.
   */
  public SwerveDriveConfig withSimDiscretizationTime(Time dt) {
    simDiscretizationSeconds = Optional.ofNullable(dt);
    return this;
  }

  /**
   * Set the angular velocity scale factor to improve the accuracy of the pose estimation.
   *
   * @param scaleFactor Scale factor to apply to the gyro angular velocity, [0, 1].
   * @return {@link SwerveDriveConfig} for chaining.
   */
  public SwerveDriveConfig withSimGyroAngularVelocityScaleFactor(double scaleFactor) {
    simAngularVelocityScaleFactor = OptionalDouble.of(scaleFactor);
    return this;
  }

  /**
   * Set the discretization time for the pose estimation.
   *
   * @param dt Discretization time for the pose estimation.
   * @return {@link SwerveDriveConfig} for chaining.
   */
  public SwerveDriveConfig withDiscretizationTime(Time dt) {
    discretizationSeconds = Optional.ofNullable(dt);
    return this;
  }

  /**
   * Set the angular velocity scale factor to improve the accuracy of the pose estimation.
   *
   * @param scaleFactor Scale factor to apply to the gyro angular velocity, [0, 1].
   * @return {@link SwerveDriveConfig} for chaining.
   */
  public SwerveDriveConfig withGyroAngularVelocityScaleFactor(double scaleFactor) {
    angularVelocityScaleFactor = OptionalDouble.of(scaleFactor);
    return this;
  }

  /**
   * Angular velocity of the gyro.
   *
   * @param angularVelocitySupplier {@link Supplier<AngularVelocity>} for the gyro angular velocity.
   * @return {@link SwerveDriveConfig} for chaining.
   */
  public SwerveDriveConfig withGyroVelocity(Supplier<AngularVelocity> angularVelocitySupplier) {
    gyroAngularVelocitySupplier = Optional.ofNullable(angularVelocitySupplier);
    return this;
  }

  /**
   * Set the gyro, as its full attitude. The attitude's yaw is the robot's heading, and its roll and
   * pitch are what anti-tipping (see {@link #withAntiTipping(AntiTipping)}) reads the tilt from. For
   * a gyro that is not mounted flat, say which of its axes points up with
   * {@link #withGyroHeadingAxis(GyroAxis)}.
   *
   * @param gyro {@link Supplier} of the gyro's attitude, such as {@code pigeon::getRotation3d}.
   * @return {@link SwerveDriveConfig} for chaining.
   */
  public SwerveDriveConfig withGyro(Supplier<Rotation3d> gyro) {
    gyroSupplier = Optional.ofNullable(gyro);
    continuousGyroAngleRead = false;
    return this;
  }

  /**
   * Set which axis of the gyro points up through the robot, for a gyro that is not mounted flat.
   * The attitude from {@link #withGyro(Supplier)} is rotated into the robot's frame before the
   * heading and anti-tipping read it.
   *
   * @param axis {@link GyroAxis} that points up. {@link GyroAxis#YAW} by default.
   * @return {@link SwerveDriveConfig} for chaining.
   */
  public SwerveDriveConfig withGyroHeadingAxis(GyroAxis axis) {
    gyroHeadingAxis = axis;
    continuousGyroAngleRead = false;
    return this;
  }

  /**
   * Drive the robot back under itself when it starts to tip. Past the tipping threshold the robot
   * is driven toward the side it is falling to at {@code kP * sin(tilt)}, up to the maximum
   * correction speed. It reads the tilt from the gyro, see {@link #withGyro(Supplier)}.
   *
   * @param kP                 Correction speed for a robot on its side.
   * @param tippingThreshold   Tilt past which the correction starts, between 0 and 90 degrees.
   * @param maxCorrectionSpeed Largest correction speed.
   * @return {@link SwerveDriveConfig} for chaining.
   * @throws IllegalArgumentException if kP or the maximum correction speed is not positive, or the
   *                                  threshold is not between 0 and 90 degrees.
   */
  public SwerveDriveConfig withAntiTipping(LinearVelocity kP, Angle tippingThreshold, LinearVelocity maxCorrectionSpeed) {
    if (kP.in(MetersPerSecond) <= 0) {
      throw new IllegalArgumentException("Anti-tipping kP must be positive, but was " + kP);
    }
    if (maxCorrectionSpeed.in(MetersPerSecond) <= 0) {
      throw new IllegalArgumentException("Anti-tipping maximum correction speed must be positive, but was " + maxCorrectionSpeed);
    }
    if (tippingThreshold.in(Radians) <= 0 || tippingThreshold.in(Radians) >= Math.PI / 2) {
      throw new IllegalArgumentException("Anti-tipping threshold must be between 0 and 90 degrees, but was " + tippingThreshold);
    }
    return withAntiTipping(new AntiTipping(kP.in(MetersPerSecond), tippingThreshold.in(Radians), maxCorrectionSpeed.in(MetersPerSecond)));
  }

  /**
   * Drive the robot back under itself when it starts to tip, with an {@link AntiTipping} you made,
   * which can be retuned while the robot runs. It reads the tilt from the gyro, see
   * {@link #withGyro(Supplier)}.
   *
   * @param antiTipping {@link AntiTipping} to correct with, or null for none.
   * @return {@link SwerveDriveConfig} for chaining.
   */
  public SwerveDriveConfig withAntiTipping(AntiTipping antiTipping) {
    this.antiTipping = Optional.ofNullable(antiTipping);
    return this;
  }

  /**
   * Get the anti-tipping correction for the robot's current attitude.
   *
   * @return Robot relative {@link ChassisSpeeds} to add to the drive's speeds. Zero when anti-tipping
   *         is not configured or no gyro is set.
   */
  public ChassisSpeeds getAntiTippingCorrection() {
    if (antiTipping.isEmpty() || gyroSupplier.isEmpty()) {
      return new ChassisSpeeds();
    }
    final Rotation3d attitude = getGyroRotation3d();
    // The correction is in the frame the attitude's yaw is measured in: turn it to the robot's.
    return ChassisSpeeds.fromFieldRelativeSpeeds(antiTipping.get().calculate(attitude), new Rotation2d(attitude.getZ()));
  }

  /**
   * Turn module state optimization on or off for every module of the drive, in place of
   * {@link SwerveModuleConfig#withOptimization(boolean)} on each one.
   *
   * @param enabled True to enable optimization, false otherwise.
   * @return {@link SwerveDriveConfig} for chaining.
   */
  public SwerveDriveConfig withModuleStateOptimization(boolean enabled) {
    moduleStateOptimization = Optional.of(enabled);
    if (modules != null) {
      for (SwerveModule module : modules) {
        module.getConfig().withOptimization(enabled);
      }
    }
    return this;
  }

  /**
   * Get the module state optimization set for every module of the drive.
   *
   * @return Empty when it was not set with {@link #withModuleStateOptimization(boolean)}.
   */
  public Optional<Boolean> getModuleStateOptimization() {
    return moduleStateOptimization;
  }

  /**
   * Set the gyro offset.
   *
   * @param offset Offset to apply to the gyro.
   * @return {@link SwerveDriveConfig} for chaining.
   */
  public SwerveDriveConfig withGyroOffset(Angle offset) {
    gyroOffset = Optional.ofNullable(offset);
    return this;
  }

  /**
   * Set the gyro inverted.
   *
   * @param inverted Inverted state of the gyro.
   * @return {@link SwerveDriveConfig} for chaining.
   */
  public SwerveDriveConfig withGyroInverted(boolean inverted) {
    gyroInverted = inverted;
    return this;
  }

  /**
   * Maximum speed of the chassis to desaturate towards.
   *
   * @param speed           Linear velocity of the Chassis.
   * @param angularVelocity Angular velocity of the Chassis.
   * @return {@link SwerveDriveConfig} for chaining.
   */
  public SwerveDriveConfig withMaximumChassisSpeed(LinearVelocity speed, AngularVelocity angularVelocity) {
    maximumChassisLinearVelocity = Optional.ofNullable(speed);
    maximumChassisAngularVelocity = Optional.ofNullable(angularVelocity);
    return this;
  }

  /**
   * Set the maximum speed of the modules to desaturate towards. {@link edu.wpi.first.math.kinematics.SwerveModuleState}s generated from
   * {@link ChassisSpeeds} are scaled down together so no module exceeds this speed.
   *
   * @param speed Linear velocity of the modules.
   * @return {@link SwerveDriveConfig} for chaining.
   */
  public SwerveDriveConfig withMaximumModuleSpeed(LinearVelocity speed) {
    maximumModuleLinearVelocity = Optional.ofNullable(speed);
    return this;
  }

  /**
   * Set the starting pose of the robot.
   *
   * @param pose {@link Pose2d} to set the robot to. {@code new Pose2d()}
   * @return {@link SwerveDriveConfig} for chaining.
   */
  public SwerveDriveConfig withStartingPose(Pose2d pose) {
    initialPose = pose;
    return this;
  }

  //  /**
  //   * Sets up MapleSim Physics Collision support on the simulated {@link yams.mechanisms.swerve.SwerveDrive}.
  //   * This integration is simplified and may result in slight differences compared to the real robot.
  //   * For more information read these
  //   * <a href="https://shenzhen-robotics-alliance.github.io/maple-sim/swerve-simulation-overview">MapleSim Docs</a>
  //   *
  //   * @param mapleConfig {@link DriveTrainSimulationConfig} for the simulated {@link yams.mechanisms.swerve.SwerveDrive}.
  //   * @param startingPose initial pose of the simulated {@link yams.mechanisms.swerve.SwerveDrive}.
  //   * @return {@link SwerveDriveConfig} for chaining.
  //   */
  //  public SwerveDriveConfig withMapleSim(DriveTrainSimulationConfig mapleConfig, Pose2d
  // startingPose)
  //  {
  //      if (RobotBase.isSimulation()) {
  //          this.mapleDriveSim = Optional.of(new SelfControlledSwerveDriveSimulation(new
  // SwerveDriveSimulation(mapleConfig, startingPose)));
  //          this.initialPose = startingPose;
  //          // Register the drivetrain sim with the SimulatedArena
  //
  // SimulatedArena.getInstance().addDriveTrainSimulation(mapleDriveSim.get().getDriveTrainSimulation());
  //      }
  //      return this;
  //  }

  /**
   * Configure telemetry for the {@link SwerveModule} mechanism.
   *
   * @param name               Telemetry Name
   * @param telemetryVerbosity Telemetry verbosity to apply.
   * @return {@link SwerveDriveConfig} for chaining.
   */
  public SwerveDriveConfig withTelemetry(String name, TelemetryVerbosity telemetryVerbosity) {
    this.telemetryName = name;
    this.telemetryVerbosity = Optional.ofNullable(telemetryVerbosity);
    return this;
  }

  /**
   * Configure telemetry for the {@link SwerveDrive} with a {@link SwerveDriveTelemetryConfig}.
   *
   * @param name            Telemetry Name
   * @param telemetryConfig Config that specifies what to log.
   * @return {@link SwerveDriveConfig} for chaining.
   */
  public SwerveDriveConfig withTelemetry(String name, SwerveDriveTelemetryConfig telemetryConfig) {
    this.telemetryName = name;
    this.telemetryVerbosity = Optional.empty();
    this.specifiedTelemetryConfig = Optional.ofNullable(telemetryConfig);
    return this;
  }

  /**
   * Get the user specified {@link SwerveDriveTelemetryConfig}, if configured via {@link #withTelemetry(String,SwerveDriveTelemetryConfig)}.
   *
   * @return {@link SwerveDriveTelemetryConfig} if configured.
   */
  public Optional<SwerveDriveTelemetryConfig> getSwerveDriveTelemetryConfig() {
    return specifiedTelemetryConfig;
  }

  /**
   * Get the telemetry name for the {@link SwerveDrive}.
   *
   * @return Telemetry name for the {@link SwerveDrive}.
   */
  public String getTelemetryName() {
    return telemetryName;
  }

  /**
   * Get the center of rotation.
   *
   * @return {@link Translation2d} of the center of rotation in Meters, X is forward, Y is left.
   */
  public Optional<Translation2d> getCenterOfRotation() {
    return centerOfRotation;
  }

  /**
   * Get the telemetry verbosity for the {@link SwerveModule}.
   *
   * @return {@link TelemetryVerbosity} for the {@link SwerveModule}.
   */
  public Optional<TelemetryVerbosity> getTelemetryVerbosity() {
    return telemetryVerbosity;
  }

  /**
   * Get the {@link SwerveModule}s for the {@link SwerveDrive}.
   *
   * @return {@link SwerveModule}s for the {@link SwerveDrive}.
   */
  public SwerveModule[] getModules() {
    return modules;
  }

  /**
   * Get the robot's attitude from the gyro, with the heading axis, inversion and offset applied:
   * its yaw is the robot's heading.
   *
   * @return The robot's attitude.
   * @throws IllegalStateException if no gyro is set.
   */
  public Rotation3d getGyroRotation3d() {
    if (gyroSupplier.isEmpty()) {
      throw new IllegalStateException("Gyro supplier is not set! Please use .withGyro() to set the gyro supplier!");
    }
    Rotation3d attitude = gyroHeadingAxis.toRobotAttitude(gyroSupplier.get().get());
    if (gyroInverted) {
      attitude = new Rotation3d(attitude.getX(), attitude.getY(), -attitude.getZ());
    }
    return attitude.rotateBy(new Rotation3d(0, 0, -getGyroOffset().in(Radians)));
  }

  /**
   * Get the starting pose of the robot.
   *
   * @return {@link Pose2d} of the robot.
   */
  public Pose2d getInitialPose() {
    return initialPose;
  }

  /**
   * Get the maximum speed of the chassis.
   *
   * @return Maximum speed of the chassis.
   */
  public Optional<LinearVelocity> getMaximumChassisLinearVelocity() {
    return maximumChassisLinearVelocity;
  }

  /**
   * Get the maximum angular speed of the chassis.
   *
   * @return Maximum angular speed of the chassis.
   */
  public Optional<AngularVelocity> getMaximumChassisAngularVelocity() {
    return maximumChassisAngularVelocity;
  }

  /**
   * Get the maximum speed of the modules.
   *
   * @return Maximum speed of the modules.
   */
  public Optional<LinearVelocity> getMaximumModuleLinearVelocity() {
    return maximumModuleLinearVelocity;
  }

  /**
   * Correct for skew that worsens as angular velocity increases
   *
   * @param robotRelativeVelocity The chassis speeds to set the robot to achieve.
   * @return {@link ChassisSpeeds} of the robot after angular velocity skew correction.
   */
  private ChassisSpeeds angularVelocitySkewCorrection(ChassisSpeeds robotRelativeVelocity) {
    AngularVelocity gyroAngularVelocity;
    if (gyroAngularVelocitySupplier.isPresent()) {
      gyroAngularVelocity = gyroAngularVelocitySupplier.get().get();
    } else {
      if (noGyroAngularVelocitySupplierAlert == null) {
        noGyroAngularVelocitySupplierAlert = new Alert("YAMS",
            getTelemetryName() + " has an angular velocity scale factor configured but no gyro angular velocity " + "supplier (see SwerveDriveConfig#withGyroVelocity); deriving it from the gyro angle instead.", Alert.AlertType.kInfo);
        noGyroAngularVelocitySupplierAlert.set(true);
      }
      // Follow the heading across a wrap, so the derivative does not see a full turn in one loop.
      final double gyroAngleRadians = getGyroRotation3d().getZ();
      continuousGyroAngleRadians = continuousGyroAngleRead
                                   ? continuousGyroAngleRadians + MathUtil.angleModulus(gyroAngleRadians - continuousGyroAngleRadians)
                                   : gyroAngleRadians;
      continuousGyroAngleRead = true;
      gyroAngularVelocity = Radians.per(Microsecond).of(gyroAngularVelocityFilter.derivative(continuousGyroAngleRadians));
    }
    var angularVelocityScale = (RobotBase.isSimulation() ? simAngularVelocityScaleFactor.orElse(angularVelocityScaleFactor.orElseThrow()) : angularVelocityScaleFactor.orElseThrow());
    var angularVelocity = new Rotation2d(gyroAngularVelocity.in(RadiansPerSecond) * angularVelocityScale);
    if (angularVelocity.getRadians() != 0.0) {
      var gyroRotation = getGyroRotation3d().toRotation2d();
      ChassisSpeeds fieldRelativeVelocity = ChassisSpeeds.fromRobotRelativeSpeeds(robotRelativeVelocity, gyroRotation);
      robotRelativeVelocity = ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeVelocity, gyroRotation.plus(angularVelocity));
    }
    return robotRelativeVelocity;
  }

  /**
   * Optimize the given chassis speeds.
   *
   * @param speeds {@link ChassisSpeeds} to optimize.
   * @return Optimized {@link ChassisSpeeds}.
   */
  public ChassisSpeeds optimizeRobotRelativeChassisSpeeds(ChassisSpeeds speeds) {
    if (antiTipping.isPresent()) {
      speeds = speeds.plus(getAntiTippingCorrection());
    }
    if (angularVelocityScaleFactor.isPresent()) {
      speeds = angularVelocitySkewCorrection(speeds);
    }
    if (discretizationSeconds.isPresent()) {
      speeds = ChassisSpeeds.discretize(speeds, (RobotBase.isSimulation() ? simDiscretizationSeconds.orElse(discretizationSeconds.get()) : discretizationSeconds.get()).in(Seconds));
    }
    return speeds;
  }

  /**
   * Get the gyro offset.
   *
   * @return Gyro offset.
   */
  public Angle getGyroOffset() {
    return gyroOffset.orElse(Rotations.of(0));
  }

  /**
   * Get the translation PID controller, if one is configured. The drive runs without one; only
   * driving to a pose needs it.
   *
   * @return Translation PID controller, or empty if none is configured.
   */
  public Optional<PIDController> getTranslationPID() {
    return RobotBase.isSimulation() ? simTranslationController.or(() -> translationController) : translationController;
  }

  /**
   * Get the rotation PID controller, if one is configured. The drive runs without one; only driving
   * to a pose and the heading, aim and translation only modes of
   * {@link yams.mechanisms.swerve.utility.SwerveInputStream} need it.
   *
   * @return Rotation PID controller, or empty if none is configured.
   */
  public Optional<PIDController> getRotationPID() {
    return RobotBase.isSimulation() ? simRotationController.or(() -> rotationController) : rotationController;
  }

  /**
   * Cube the {@link Translation2d} magnitude given in Polar coordinates.
   *
   * @param translation {@link Translation2d} to manipulate.
   * @return Cubed magnitude from {@link Translation2d}.
   */
  public static Translation2d cubeTranslation(Translation2d translation) {
    if (Math.hypot(translation.getX(), translation.getY()) <= 1.0E-6) {
      return translation;
    }
    return new Translation2d(Math.pow(translation.getNorm(), 3), translation.getAngle());
  }

  /**
   * Scale the {@link Translation2d} Polar coordinate magnitude.
   *
   * @param translation {@link Translation2d} to use.
   * @param scalar      Multiplier for the Polar coordinate magnitude to use.
   * @return {@link Translation2d} scaled by given magnitude scalar.
   */
  public static Translation2d scaleTranslation(Translation2d translation, double scalar) {
    if (Math.hypot(translation.getX(), translation.getY()) <= 1.0E-6) {
      return translation;
    }
    return new Translation2d(translation.getNorm() * scalar, translation.getAngle());
  }

  /**
   * Get the discretization time for the pose estimation used by {@link #optimizeRobotRelativeChassisSpeeds(ChassisSpeeds)}
   *
   * @return Discretization time for the pose estimation.
   */
  public Optional<Time> getDiscretization() {
    return RobotBase.isSimulation() ? simDiscretizationSeconds : discretizationSeconds;
  }

  //  /**
  //   * Get the {@link SelfControlledSwerveDriveSimulation} if it is configured.
  //   *
  //   * @return the {@link SelfControlledSwerveDriveSimulation} if it is configured, otherwise
  // throws an exception.
  //   */
  //  public Optional<SelfControlledSwerveDriveSimulation> getMapleDriveSim() {
  //      if (RobotBase.isSimulation()) {
  //          return mapleDriveSim;
  //      }
  //      throw new IllegalStateException("RobotBase is not in Simulation, MapleDriveSim is
  // empty!");
  //  }

  /**
   * Get the swerve drive subsystem.
   *
   * @return Swerve drive subsystem.
   */
  public Subsystem getSubsystem() {
    return subsystem;
  }

  /**
   * Use an external feedback sensor for the {@link SwerveModule}s.
   *
   * @return External feedback sensor for the {@link SwerveModule}s.
   */
  public boolean useExternalFeedbackSensor() {
    return true;
  }
}
