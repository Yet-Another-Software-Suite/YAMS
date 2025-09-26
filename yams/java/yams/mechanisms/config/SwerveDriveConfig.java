package yams.mechanisms.config;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import java.util.Optional;
import java.util.OptionalDouble;
import java.util.function.Supplier;
import yams.mechanisms.swerve.SwerveModule;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;

/**
 * Swerve Drive Configuration
 */
public class SwerveDriveConfig
{

  /**
   * Telemetry verbosity
   */
  private       Optional<TelemetryVerbosity>        telemetryVerbosity            = Optional.empty();
  /**
   * {@link SwerveModule}s for the {@link yams.mechanisms.swerve.SwerveDrive}.
   */
  private final SwerveModule[]                      modules;
  /**
   * Gyro supplier.
   */
  private       Optional<Supplier<Angle>>           gyroSupplier                  = Optional.empty();
  /**
   * Gyro angular velocity supplier.
   */
  private       Optional<Supplier<AngularVelocity>> gyroAngularVelocitySupplier   = Optional.empty();
  /**
   * Gyro offset.
   */
  private       Optional<Angle>                     gyroOffset                    = Optional.empty();
  /**
   * Gyro inverted.
   */
  private       boolean                             gyroInverted                  = false;
  /**
   * Starting pose on the field.
   */
  private       Pose2d                              initialPose                   = new Pose2d();
  /**
   * Maximum speed of the chassis.
   */
  private       Optional<LinearVelocity>            maximumChassisLinearVelocity  = Optional.empty();
  /**
   * Maximum angular speed of the chassis.
   */
  private       Optional<AngularVelocity>           maximumChassisAngularVelocity = Optional.empty();
  /**
   * Maximum speed of the modules.
   */
  private       Optional<LinearVelocity>            maximumModuleLinearVelocity   = Optional.empty();
  /**
   * Discretization time for the pose estimation.
   */
  private       Optional<Time>                      discretizationSeconds         = Optional.empty();
  /**
   * Angular velocity scale factor.
   */
  private       OptionalDouble                      angularVelocityScaleFactor    = OptionalDouble.empty();
  /**
   * Center of Rotation
   */
  private       Optional<Translation2d>             centerOfRotation              = Optional.empty();
  /**
   * Translation PID controller.
   */
  private       Optional<PIDController>             translationController         = Optional.empty();
  /**
   * Rotation PID controller.
   */
  private       Optional<PIDController>             rotationController            = Optional.empty();
  /**
   * Simulated translation PID controller.
   */
  private       Optional<PIDController>             simTranslationController      = Optional.empty();
  /**
   * Simulated rotation PID controller.
   */
  private       Optional<PIDController>             simRotationController         = Optional.empty();
  /**
   * Discretization time for the pose estimation.
   */
  private       Optional<Time>                      simDiscretizationSeconds      = Optional.empty();
  /**
   * Angular velocity scale factor.
   */
  private       OptionalDouble                      simAngularVelocityScaleFactor = OptionalDouble.empty();
  /**
   * Swerve drive subsystem.
   */
  private       Subsystem                           subsystem;

  /**
   * Create the {@link SwerveDriveConfig} for the {@link yams.mechanisms.swerve.SwerveDrive}
   *
   * @param modules {@link SwerveModule}s for the {@link yams.mechanisms.swerve.SwerveDrive}
   */
  public SwerveDriveConfig(Subsystem swerveSubsystem, SwerveModule... modules)
  {
    subsystem = swerveSubsystem;
    this.modules = modules;
  }

  /**
   * Set the translation PID controller.
   *
   * @param controller {@link PIDController} for the translation, input units are meters.
   * @return {@link SwerveDriveConfig} for chaining.
   */
  public SwerveDriveConfig withSimTranslationController(PIDController controller)
  {
    simTranslationController = Optional.ofNullable(controller);
    return this;
  }

  /**
   * Set the rotation PID controller.
   *
   * @param controller {@link PIDController} for the rotation, input units are radians.
   * @return {@link SwerveDriveConfig} for chaining.
   */
  public SwerveDriveConfig withSimRotationController(PIDController controller)
  {
    simRotationController = Optional.ofNullable(controller);
    return this;
  }

  /**
   * Set the translation PID controller.
   *
   * @param controller {@link PIDController} for the translation, input units are meters.
   * @return {@link SwerveDriveConfig} for chaining.
   */
  public SwerveDriveConfig withTranslationController(PIDController controller)
  {
    translationController = Optional.ofNullable(controller);
    return this;
  }

  /**
   * Set the rotation PID controller.
   *
   * @param controller {@link PIDController} for the rotation, input units are radians.
   * @return {@link SwerveDriveConfig} for chaining.
   */
  public SwerveDriveConfig withRotationController(PIDController controller)
  {
    rotationController = Optional.ofNullable(controller);
    return this;
  }

  /**
   * Set the center of rotation; 0,0 is the center of the robot.
   *
   * @param centerOfRotation {@link Translation2d} of the center of rotation in Meters, X is forward, Y is left.
   * @return {@link SwerveDriveConfig} for chaining.
   */
  public SwerveDriveConfig withCenterOfRotation(Translation2d centerOfRotation)
  {
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
  public SwerveDriveConfig withCenterOfRotation(Distance forward, Distance left)
  {
    this.centerOfRotation = Optional.ofNullable(new Translation2d(forward, left));
    return this;
  }

  /**
   * Set the discretization time for the pose estimation.
   *
   * @param dt Discretization time for the pose estimation.
   * @return {@link SwerveDriveConfig} for chaining.
   */
  public SwerveDriveConfig withSimDiscretizationTime(Time dt)
  {
    simDiscretizationSeconds = Optional.ofNullable(dt);
    return this;
  }

  /**
   * Set the angular velocity scale factor to improve the accuracy of the pose estimation.
   *
   * @param scaleFactor Scale factor to apply to the gyro angular velocity, [0, 1].
   * @return {@link SwerveDriveConfig} for chaining.
   */
  public SwerveDriveConfig withSimGyroAngularVelocityScaleFactor(double scaleFactor)
  {
    simAngularVelocityScaleFactor = OptionalDouble.of(scaleFactor);
    return this;
  }

  /**
   * Set the discretization time for the pose estimation.
   *
   * @param dt Discretization time for the pose estimation.
   * @return {@link SwerveDriveConfig} for chaining.
   */
  public SwerveDriveConfig withDiscretizationTime(Time dt)
  {
    discretizationSeconds = Optional.ofNullable(dt);
    return this;
  }

  /**
   * Set the angular velocity scale factor to improve the accuracy of the pose estimation.
   *
   * @param scaleFactor Scale factor to apply to the gyro angular velocity, [0, 1].
   * @return {@link SwerveDriveConfig} for chaining.
   */
  public SwerveDriveConfig withGyroAngularVelocityScaleFactor(double scaleFactor)
  {
    angularVelocityScaleFactor = OptionalDouble.of(scaleFactor);
    return this;
  }

  /**
   * Angular velocity of the gyro.
   *
   * @param angularVelocitySupplier {@link Supplier<AngularVelocity>} for the gyro angular velocity.
   * @return {@link SwerveDriveConfig} for chaining.
   */
  public SwerveDriveConfig withGyroVelocity(Supplier<AngularVelocity> angularVelocitySupplier)
  {
    gyroAngularVelocitySupplier = Optional.ofNullable(angularVelocitySupplier);
    return this;
  }

  /**
   * Get the {@link SwerveModule}s for the {@link yams.mechanisms.swerve.SwerveDrive}.
   *
   * @param gyro {@link Supplier} for the gyro.
   * @return {@link SwerveDriveConfig} for chaining.
   */
  public SwerveDriveConfig withGyro(Supplier<Angle> gyro)
  {
    gyroSupplier = Optional.ofNullable(gyro);
    return this;
  }

  /**
   * Set the gyro offset.
   *
   * @param offset Offset to apply to the gyro.
   * @return {@link SwerveDriveConfig} for chaining.
   */
  public SwerveDriveConfig withGyroOffset(Angle offset)
  {
    gyroOffset = Optional.ofNullable(offset);
    return this;
  }

  /**
   * Set the gyro inverted.
   *
   * @param inverted Inverted state of the gyro.
   * @return {@link SwerveDriveConfig} for chaining.
   */
  public SwerveDriveConfig withGyroInverted(boolean inverted)
  {
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
  public SwerveDriveConfig withMaximumChassisSpeed(LinearVelocity speed, AngularVelocity angularVelocity)
  {
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
  public SwerveDriveConfig withMaximumModuleSpeed(LinearVelocity speed)
  {
    maximumModuleLinearVelocity = Optional.ofNullable(speed);
    return this;
  }

  /**
   * Set the starting pose of the robot.
   *
   * @param pose {@link Pose2d} to set the robot to.  {@code new Pose2d()}
   * @return {@link SwerveDriveConfig} for chaining.
   */
  public SwerveDriveConfig withStartingPose(Pose2d pose)
  {
    initialPose = pose;
    return this;
  }

  /**
   * Configure telemetry for the {@link yams.mechanisms.swerve.SwerveModule} mechanism.
   *
   * @param telemetryVerbosity Telemetry verbosity to apply.
   * @return {@link SwerveDriveConfig} for chaining.
   */
  public SwerveDriveConfig withTelemetry(TelemetryVerbosity telemetryVerbosity)
  {
    this.telemetryVerbosity = Optional.ofNullable(telemetryVerbosity);
    return this;
  }

  /**
   * Get the center of rotation.
   *
   * @return {@link Translation2d} of the center of rotation in Meters, X is forward, Y is left.
   */
  public Optional<Translation2d> getCenterOfRotation()
  {
    return centerOfRotation;
  }

  /**
   * Get the telemetry verbosity for the {@link yams.mechanisms.swerve.SwerveModule}.
   *
   * @return {@link TelemetryVerbosity} for the {@link yams.mechanisms.swerve.SwerveModule}.
   */
  public Optional<TelemetryVerbosity> getTelemetryVerbosity()
  {
    return telemetryVerbosity;
  }

  /**
   * Get the {@link SwerveModule}s for the {@link yams.mechanisms.swerve.SwerveDrive}.
   *
   * @return {@link SwerveModule}s for the {@link yams.mechanisms.swerve.SwerveDrive}.
   */
  public SwerveModule[] getModules()
  {
    return modules;
  }

  /**
   * Get the gyro angle with inversions and offsets applied.
   *
   * @return {@link Angle} of the gyro.
   */
  public Angle getGyroAngle()
  {
    if (gyroSupplier.isEmpty())
    {
      throw new IllegalStateException("Gyro supplier is not set! Please use .withGyro() to set the gyro supplier!");
    }
    return (gyroInverted ? gyroSupplier.get().get().unaryMinus() : gyroSupplier.get().get()).minus(gyroOffset.orElse(
        Rotations.of(0)));
  }

  /**
   * Get the starting pose of the robot.
   *
   * @return {@link Pose2d} of the robot.
   */
  public Pose2d getInitialPose()
  {
    return initialPose;
  }

  /**
   * Get the maximum speed of the chassis.
   *
   * @return Maximum speed of the chassis.
   */
  public Optional<LinearVelocity> getMaximumChassisLinearVelocity()
  {
    return maximumChassisLinearVelocity;
  }

  /**
   * Get the maximum angular speed of the chassis.
   *
   * @return Maximum angular speed of the chassis.
   */
  public Optional<AngularVelocity> getMaximumChassisAngularVelocity()
  {
    return maximumChassisAngularVelocity;
  }

  /**
   * Get the maximum speed of the modules.
   *
   * @return Maximum speed of the modules.
   */
  public Optional<LinearVelocity> getMaximumModuleLinearVelocity()
  {
    return maximumModuleLinearVelocity;
  }

  /**
   * Correct for skew that worsens as angular velocity increases
   *
   * @param robotRelativeVelocity The chassis speeds to set the robot to achieve.
   * @return {@link ChassisSpeeds} of the robot after angular velocity skew correction.
   */
  private ChassisSpeeds angularVelocitySkewCorrection(ChassisSpeeds robotRelativeVelocity)
  {
    var angularVelocity = new Rotation2d(gyroAngularVelocitySupplier.orElseThrow().get().in(RadiansPerSecond) *
                                         (RobotBase.isSimulation() ?
                                          simAngularVelocityScaleFactor.orElse(angularVelocityScaleFactor.orElseThrow()) :
                                          angularVelocityScaleFactor.orElseThrow()));
    if (angularVelocity.getRadians() != 0.0)
    {
      var           gyroRotation          = new Rotation2d(gyroSupplier.orElseThrow().get());
      ChassisSpeeds fieldRelativeVelocity = ChassisSpeeds.fromRobotRelativeSpeeds(robotRelativeVelocity, gyroRotation);
      robotRelativeVelocity = ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeVelocity,
                                                                    gyroRotation.plus(angularVelocity));
    }
    return robotRelativeVelocity;
  }

  /**
   * Optimize the given chassis speeds.
   *
   * @param speeds {@link ChassisSpeeds} to optimize.
   * @return Optimized {@link ChassisSpeeds}.
   */
  public ChassisSpeeds optimizeRobotRelativeChassisSpeeds(ChassisSpeeds speeds)
  {
    if (angularVelocityScaleFactor.isPresent())
    {
      speeds = angularVelocitySkewCorrection(speeds);
    }
    if (discretizationSeconds.isPresent())
    {
      speeds = ChassisSpeeds.discretize(speeds, (RobotBase.isSimulation() ?
                                                 simDiscretizationSeconds.orElse(discretizationSeconds.get()) :
                                                 discretizationSeconds.get()).in(Seconds));
    }
    return speeds;
  }

  /**
   * Get the gyro offset.
   *
   * @return Gyro offset.
   */
  public Angle getGyroOffset()
  {
    return gyroOffset.orElse(Rotations.of(0));
  }

  /**
   * Get the translation PID controller.
   *
   * @return Translation PID controller.
   */
  public PIDController getTranslationPID()
  {
    return (RobotBase.isSimulation() ? simTranslationController.orElse(translationController.orElseThrow())
                                     : translationController.orElseThrow());
  }

  /**
   * Get the rotation PID controller.
   *
   * @return Rotation PID controller.
   */
  public PIDController getRotationPID()
  {
    return (RobotBase.isSimulation() ? simRotationController.orElse(rotationController.orElseThrow())
                                     : rotationController.orElseThrow());
  }

  /**
   * Get the swerve drive subsystem.
   *
   * @return Swerve drive subsystem.
   */
  public Subsystem getSubsystem()
  {
    return subsystem;
  }
}
