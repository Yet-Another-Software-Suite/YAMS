// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package yams.core.mechanisms.config;

import static org.wpilib.units.Units.Microsecond;
import static org.wpilib.units.Units.Milliseconds;
import static org.wpilib.units.Units.Radians;
import static org.wpilib.units.Units.RadiansPerSecond;
import static org.wpilib.units.Units.Rotations;
import static org.wpilib.units.Units.Seconds;

import java.util.Optional;
import java.util.OptionalDouble;
import java.util.function.Supplier;
import org.wpilib.framework.RobotBase;
import org.wpilib.math.controller.PIDController;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Distance;
import org.wpilib.units.measure.LinearVelocity;
import org.wpilib.units.measure.Time;
import org.wpilib.util.Alert;
import yams.core.math.DerivativeTimeFilter;
import yams.core.mechanisms.swerve.SwerveDrive;
import yams.core.mechanisms.swerve.SwerveModule;
import yams.core.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.core.telemetry.SwerveDriveTelemetryConfig;

/**
 * Swerve Drive Configuration.
 *
 * <p>Holds modules, gyro supplier, and drive controllers. Modules are ordered clockwise from
 * front-left: FL, FR, BL, BR. See {@link yams.commands2.config.SwerveDriveConfig} for the
 * Subsystem-bound subclass and a full construction example.
 */
public class SwerveDriveConfig {
  /**
   * {@link SwerveModule}s for the {@link SwerveDrive}.
   */
  private SwerveModule[] modules;
  /**
   * Telemetry name for the {@link SwerveDrive}.
   */
  private String telemetryName = "swerve";
  /**
   * Telemetry verbosity
   */
  private Optional<TelemetryVerbosity> telemetryVerbosity = Optional.empty();
  /**
   * User specified {@link SwerveDriveTelemetryConfig}, takes precedence over {@link
   * #telemetryVerbosity} if present.
   */
  private Optional<SwerveDriveTelemetryConfig> specifiedTelemetryConfig = Optional.empty();
  /**
   * Gyro supplier.
   */
  private Optional<Supplier<Angle>> gyroSupplier = Optional.empty();
  /**
   * Gyro angular velocity supplier.
   */
  private Optional<Supplier<AngularVelocity>> gyroAngularVelocitySupplier = Optional.empty();
  /**
   * Derives the gyro angular velocity from the gyro angle ({@link #getGyroAngle()}) when {@link
   * #gyroAngularVelocitySupplier} is not configured, in both simulation and real robot code.
   */
  private final DerivativeTimeFilter gyroAngularVelocityFilter = new DerivativeTimeFilter(Milliseconds.of(20));
  /**
   * Alert shown once if {@link #angularVelocitySkewCorrection(ChassisVelocities)} runs without a
   * {@link #gyroAngularVelocitySupplier} configured, warning that the gyro angular velocity is
   * being
   * derived from the gyro angle instead.
   */
  private Alert noGyroAngularVelocitySupplierAlert = null;
  /**
   * Gyro offset.
   */
  private Optional<Angle> gyroOffset = Optional.empty();
  /**
   * Gyro inverted.
   */
  private boolean gyroInverted = false;
  /**
   * Starting pose on the field.
   */
  private Pose2d initialPose = new Pose2d();
  /**
   * Maximum speed of the chassis.
   */
  private Optional<LinearVelocity> maximumChassisLinearVelocity = Optional.empty();
  /**
   * Maximum angular speed of the chassis.
   */
  private Optional<AngularVelocity> maximumChassisAngularVelocity = Optional.empty();
  /**
   * Maximum speed of the modules.
   */
  private Optional<LinearVelocity> maximumModuleLinearVelocity = Optional.empty();
  /**
   * Discretization time for the pose estimation.
   */
  private Optional<Time> discretizationSeconds = Optional.empty();
  /**
   * Angular velocity scale factor.
   */
  private OptionalDouble angularVelocityScaleFactor = OptionalDouble.empty();
  /**
   * Center of Rotation
   */
  private Optional<Translation2d> centerOfRotation = Optional.empty();
  /**
   * Translation PID controller.
   */
  private Optional<PIDController> translationController = Optional.empty();
  /**
   * Rotation PID controller.
   */
  private Optional<PIDController> rotationController = Optional.empty();
  /**
   * Simulated translation PID controller.
   */
  private Optional<PIDController> simTranslationController = Optional.empty();
  /**
   * Simulated rotation PID controller.
   */
  private Optional<PIDController> simRotationController = Optional.empty();
  /**
   * Discretization time for the pose estimation.
   */
  private Optional<Time> simDiscretizationSeconds = Optional.empty();
  /**
   * Angular velocity scale factor.
   */
  private OptionalDouble simAngularVelocityScaleFactor = OptionalDouble.empty();

  /**
   * Create the {@link SwerveDriveConfig} for the {@link SwerveDrive}
   *
   * @param modules {@link SwerveModule}s for the {@link SwerveDrive}
   * @implNote Protected so only {@link yams.commands2.config.SwerveDriveConfig} can construct
   *           this.
   */
  protected SwerveDriveConfig(SwerveModule... modules) {
    this.modules = modules;
  }

  /**
   * Create the {@link SwerveDriveConfig} for the {@link SwerveDrive}
   *
   * @implNote Must define modules with {@link #withModules(SwerveModule...)}. Protected so only
   *           {@link yams.commands2.config.SwerveDriveConfig} can construct this.
   */
  protected SwerveDriveConfig() {
  }

  protected SwerveDriveConfig(SwerveDriveConfig cfg) {
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
   * Get the {@link SwerveModule}s for the {@link SwerveDrive}.
   *
   * @param gyro {@link Supplier} for the gyro.
   * @return {@link SwerveDriveConfig} for chaining.
   */
  public SwerveDriveConfig withGyro(Supplier<Angle> gyro) {
    gyroSupplier = Optional.ofNullable(gyro);
    return this;
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
   * Set the maximum speed of the modules to desaturate towards.
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
  //   * Sets up MapleSim Physics Collision support on the simulated {@link
  //   yams.mechanisms.swerve.SwerveDrive}.
  //   * This integration is simplified and may result in slight differences compared to the real
  //   robot.
  //   * For more information read these
  //   * <a
  //   href="https://shenzhen-robotics-alliance.github.io/maple-sim/swerve-simulation-overview">MapleSim
  //   Docs</a>
  //   *
  //   * @param mapleConfig {@link DriveTrainSimulationConfig} for the simulated {@link
  //   yams.mechanisms.swerve.SwerveDrive}.
  //   * @param startingPose initial pose of the simulated {@link
  //   yams.mechanisms.swerve.SwerveDrive}.
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
   * Get the user specified {@link SwerveDriveTelemetryConfig}, if configured via {@link
   * #withTelemetry(String,SwerveDriveTelemetryConfig)}.
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
   * Get the gyro angle with inversions and offsets applied.
   *
   * @return {@link Angle} of the gyro.
   */
  public Angle getGyroAngle() {
    if (gyroSupplier.isEmpty()) {
      throw new IllegalStateException("Gyro supplier is not set! Please use .withGyro() to set the gyro supplier!");
    }
    return (gyroInverted ? gyroSupplier.get().get().unaryMinus() : gyroSupplier.get().get()).minus(gyroOffset.orElse(Rotations.of(0)));
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
   * @return {@link ChassisVelocities} of the robot after angular velocity skew correction.
   */
  private ChassisVelocities angularVelocitySkewCorrection(ChassisVelocities robotRelativeVelocity) {
    AngularVelocity gyroAngularVelocity;
    if (gyroAngularVelocitySupplier.isPresent()) {
      gyroAngularVelocity = gyroAngularVelocitySupplier.get().get();
    } else {
      if (noGyroAngularVelocitySupplierAlert == null) {
        noGyroAngularVelocitySupplierAlert = new Alert("YAMS",
            getTelemetryName() + " has an angular velocity scale factor configured but no gyro angular velocity " + ("supplier (see SwerveDriveConfig#withGyroVelocity); deriving it from the gyro " + "angle instead."), Alert.Level.LOW);
        noGyroAngularVelocitySupplierAlert.set(true);
      }
      gyroAngularVelocity = Radians.per(Microsecond).of(gyroAngularVelocityFilter.derivative(getGyroAngle().in(Radians)));
    }
    var angularVelocityScale = (RobotBase.isSimulation() ? simAngularVelocityScaleFactor.orElse(angularVelocityScaleFactor.orElseThrow()) : angularVelocityScaleFactor.orElseThrow());
    var angularVelocity = new Rotation2d(gyroAngularVelocity.in(RadiansPerSecond) * angularVelocityScale);
    if (angularVelocity.getRadians() != 0.0) {
      var gyroRotation = new Rotation2d(getGyroAngle());
      ChassisVelocities fieldRelativeVelocity = robotRelativeVelocity.toFieldRelative(gyroRotation);
      robotRelativeVelocity = fieldRelativeVelocity.toRobotRelative(gyroRotation.plus(angularVelocity));
    }
    return robotRelativeVelocity;
  }

  /**
   * Optimize the given chassis speeds.
   *
   * @param speeds {@link ChassisVelocities} to optimize.
   * @return Optimized {@link ChassisVelocities}.
   */
  public ChassisVelocities optimizeRobotRelativeChassisSpeeds(ChassisVelocities speeds) {
    if (angularVelocityScaleFactor.isPresent()) {
      speeds = angularVelocitySkewCorrection(speeds);
    }
    if (discretizationSeconds.isPresent()) {
      speeds = speeds.discretize((RobotBase.isSimulation() ? simDiscretizationSeconds.orElse(discretizationSeconds.get()) : discretizationSeconds.get()).in(Seconds));
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
   * Get the translation PID controller.
   *
   * @return Translation PID controller.
   */
  public PIDController getTranslationPID() {
    return (RobotBase.isSimulation() ? simTranslationController.orElse(translationController.orElseThrow()) : translationController.orElseThrow());
  }

  /**
   * Get the rotation PID controller.
   *
   * @return Rotation PID controller.
   */
  public PIDController getRotationPID() {
    return (RobotBase.isSimulation() ? simRotationController.orElse(rotationController.orElseThrow()) : rotationController.orElseThrow());
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
    return new Translation2d(Math.pow(translation.getNorm(), 3), translation.getAngle().orElse(new Rotation2d()));
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
    return new Translation2d(translation.getNorm() * scalar, translation.getAngle().orElse(new Rotation2d()));
  }

  /**
   * Get the discretization time for the pose estimation used by
   * {@link #optimizeRobotRelativeChassisSpeeds(ChassisVelocities)}
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
   * Use an external feedback sensor for the {@link SwerveModule}s.
   *
   * @return External feedback sensor for the {@link SwerveModule}s.
   */
  public boolean useExternalFeedbackSensor() {
    return true;
  }
}
