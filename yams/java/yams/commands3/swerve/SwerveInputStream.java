// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package yams.commands3.swerve;

import static org.wpilib.units.Units.MetersPerSecond;
import static org.wpilib.units.Units.Radians;
import static org.wpilib.units.Units.RadiansPerSecond;
import static org.wpilib.units.Units.RotationsPerSecond;

import java.util.Optional;
import java.util.function.Supplier;
import org.wpilib.driverstation.Alliance;
import org.wpilib.driverstation.MatchState;
import org.wpilib.math.controller.PIDController;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.math.util.MathUtil;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.LinearVelocity;
import yams.core.exceptions.SwerveDriveConfigurationException;
import yams.core.mechanisms.config.SwerveDriveConfig;
import yams.core.mechanisms.swerve.SwerveDrive;

/**
 * Turns driver inputs into {@link ChassisVelocities} for a {@link SwerveDrive}, for Commands v3.
 *
 * <p>Unlike {@code yams.commands2.swerve.SwerveInputStream}, which pulls its inputs from suppliers,
 * this stream holds plain values. A drive coroutine reads the controller, sets the stick values and
 * modes on the stream, and calls {@link #get()} every loop:
 *
 * <pre>{@code
 * SwerveInputStream input = new SwerveInputStream(drive)
 *     .withMaximumLinearVelocity(MetersPerSecond.of(4))
 *     .withDeadband(0.05)
 *     .withCubeTranslationControllerAxis(true)
 *     .withAllianceRelativeControl(true);
 *
 * Command teleop = mechanism.run(coroutine -> {
 *   while (true) {
 *     input.withTranslation(-controller.getLeftY(), -controller.getLeftX())
 *         .withRotation(-controller.getRightX())
 *         .withAim(controller.getRightTriggerAxis() > 0.5);
 *     drive.setFieldRelativeChassisSpeeds(input.get());
 *     coroutine.yield();
 *   }
 * }).named("Teleop Drive");
 * }</pre>
 *
 * <p>The stream picks one rotation mode each time {@link #get()} is called, in priority order:
 * translation only (hold the heading the robot had when the mode started), aim at a target, face a
 * heading, then angular velocity from the rotation axis. The heading, aim and translation only modes
 * use the rotation PID controller of the {@link SwerveDrive}. Translation is always taken from the
 * translation axes.
 */
public class SwerveInputStream implements Supplier<ChassisVelocities> {
  /** {@link SwerveDrive} used for the heading, pose and PID controllers. */
  private final SwerveDrive swerveDrive;

  /** Translation X axis, [-1, 1]. */
  private double translationX = 0;
  /** Translation Y axis, [-1, 1]. */
  private double translationY = 0;
  /** Rotation axis for angular velocity control, [-1, 1]. */
  private double rotation = 0;

  /** Face {@link #heading} while true. */
  private boolean headingEnabled = false;
  /** Field relative heading to face in {@link Mode#HEADING}. */
  private Optional<Angle> heading = Optional.empty();
  /** Aim at {@link #aimTarget} while true. */
  private boolean aimEnabled = false;
  /** Target to aim at in {@link Mode#AIM}. */
  private Optional<Pose2d> aimTarget = Optional.empty();
  /** Hold the current heading and only translate while true. */
  private boolean translationOnlyEnabled = false;
  /** Heading held in {@link Mode#TRANSLATION_ONLY}. */
  private Optional<Rotation2d> lockedHeading = Optional.empty();

  /** Treat the translation axes as robot relative. */
  private boolean robotRelative = false;
  /** Flip the translation for the red alliance. */
  private boolean allianceRelative = false;
  /** Cube the translation magnitude. */
  private boolean translationCube = false;
  /** Cube the rotation axis. */
  private boolean omegaCube = false;
  /** Rotate the output translation by {@link #translationHeadingOffset} while true. */
  private boolean translationHeadingOffsetEnabled = false;
  /** Offset applied to the output translation. */
  private Rotation2d translationHeadingOffset = Rotation2d.ZERO;

  /** Axis deadband. */
  private Optional<Double> axisDeadband = Optional.empty();
  /** Translation axis scalar, (0, 1]. */
  private Optional<Double> translationAxisScale = Optional.empty();
  /** Rotation axis scalar, (0, 1]. */
  private Optional<Double> omegaAxisScale = Optional.empty();
  /** Maximum chassis velocity if the {@link SwerveDrive} does not configure one. */
  private LinearVelocity maximumChassisLinearVelocity = MetersPerSecond.of(4);
  /** Maximum chassis angular velocity if the {@link SwerveDrive} does not configure one. */
  private AngularVelocity maximumChassisAngularVelocity = RotationsPerSecond.of(1);

  /** Mode used by the last {@link #get()}. */
  private Mode currentMode = Mode.ANGULAR_VELOCITY;

  /**
   * Create a {@link SwerveInputStream} with every axis at zero and every mode off.
   *
   * @param drive {@link SwerveDrive} to generate {@link ChassisVelocities} for.
   */
  public SwerveInputStream(SwerveDrive drive) {
    swerveDrive = drive;
  }

  /**
   * Set the translation axes.
   *
   * @param x Translation X axis (forward, field or robot relative), [-1, 1].
   * @param y Translation Y axis (left, field or robot relative), [-1, 1].
   * @return this
   */
  public SwerveInputStream withTranslation(double x, double y) {
    translationX = x;
    translationY = y;
    return this;
  }

  /**
   * Set the rotation axis used for angular velocity control.
   *
   * @param rotation Counterclockwise rotation axis, [-1, 1].
   * @return this
   */
  public SwerveInputStream withRotation(double rotation) {
    this.rotation = rotation;
    return this;
  }

  /**
   * Enable or disable facing the heading set with {@link #withHeading(Angle)}.
   *
   * @param enabled Face the heading while true.
   * @return this
   */
  public SwerveInputStream withHeadingControl(boolean enabled) {
    headingEnabled = enabled;
    return this;
  }

  /**
   * Set the field relative heading to face while heading control is enabled.
   *
   * @param heading Field relative heading, blue-origin where 0 degrees faces the red alliance wall.
   * @return this
   */
  public SwerveInputStream withHeading(Angle heading) {
    this.heading = Optional.ofNullable(heading);
    return this;
  }

  /**
   * Enable or disable aiming at the target set with {@link #withAimTarget(Pose2d)}.
   *
   * @param enabled Aim while true.
   * @return this
   */
  public SwerveInputStream withAim(boolean enabled) {
    aimEnabled = enabled;
    return this;
  }

  /**
   * Set the field relative target to aim at while aiming is enabled.
   *
   * @param target Target {@link Pose2d}, blue-origin.
   * @return this
   */
  public SwerveInputStream withAimTarget(Pose2d target) {
    aimTarget = Optional.ofNullable(target);
    return this;
  }

  /**
   * Enable or disable translation only mode, which holds the heading the robot had when the mode
   * started. Overrides aiming and heading control.
   *
   * @param enabled Only translate while true.
   * @return this
   */
  public SwerveInputStream withTranslationOnly(boolean enabled) {
    translationOnlyEnabled = enabled;
    return this;
  }

  /**
   * Treat the translation axes as robot relative (forward is the front of the robot). The output is
   * still field relative, so it can always be passed to
   * {@link SwerveDrive#setFieldRelativeChassisSpeeds}.
   *
   * @param enabled Robot relative translation while true.
   * @return this
   */
  public SwerveInputStream withRobotRelative(boolean enabled) {
    robotRelative = enabled;
    return this;
  }

  /**
   * Flip the translation for the red alliance, so forward on the stick always drives away from
   * your own alliance wall.
   *
   * @param enabled Alliance relative translation while true.
   * @return this
   */
  public SwerveInputStream withAllianceRelativeControl(boolean enabled) {
    allianceRelative = enabled;
    return this;
  }

  /**
   * Cube the translation magnitude for a non-linear response.
   *
   * @param enabled Cube while true.
   * @return this
   */
  public SwerveInputStream withCubeTranslationControllerAxis(boolean enabled) {
    translationCube = enabled;
    return this;
  }

  /**
   * Cube the rotation axis for a non-linear response.
   *
   * @param enabled Cube while true.
   * @return this
   */
  public SwerveInputStream withCubeRotationControllerAxis(boolean enabled) {
    omegaCube = enabled;
    return this;
  }

  /**
   * Rotate the output translation by an offset.
   *
   * @param offset  Offset to apply.
   * @param enabled Apply the offset while true.
   * @return this
   */
  public SwerveInputStream withTranslationHeadingOffset(Rotation2d offset, boolean enabled) {
    translationHeadingOffset = offset;
    translationHeadingOffsetEnabled = enabled;
    return this;
  }

  /**
   * Set a deadband for every axis.
   *
   * @param deadband Deadband, [0, 1). 0 disables it.
   * @return this
   */
  public SwerveInputStream withDeadband(double deadband) {
    axisDeadband = deadband == 0 ? Optional.empty() : Optional.of(deadband);
    return this;
  }

  /**
   * Scale the translation axes by a constant.
   *
   * @param scale Translation scalar, (0, 1]. 0 disables it.
   * @return this
   */
  public SwerveInputStream withScaleTranslation(double scale) {
    translationAxisScale = scale == 0 ? Optional.empty() : Optional.of(scale);
    return this;
  }

  /**
   * Scale the rotation axis by a constant.
   *
   * @param scale Rotation scalar, (0, 1]. 0 disables it.
   * @return this
   */
  public SwerveInputStream withScaleRotation(double scale) {
    omegaAxisScale = scale == 0 ? Optional.empty() : Optional.of(scale);
    return this;
  }

  /**
   * Maximum linear velocity for full stick, used if the {@link SwerveDrive} does not configure one.
   * Default is 4 meters per second.
   *
   * @param velocity Maximum linear velocity.
   * @return this
   */
  public SwerveInputStream withMaximumLinearVelocity(LinearVelocity velocity) {
    maximumChassisLinearVelocity = velocity;
    return this;
  }

  /**
   * Maximum angular velocity for full stick, used if the {@link SwerveDrive} does not configure one.
   * Default is 1 rotation per second.
   *
   * @param velocity Maximum angular velocity.
   * @return this
   */
  public SwerveInputStream withMaximumAngularVelocity(AngularVelocity velocity) {
    maximumChassisAngularVelocity = velocity;
    return this;
  }

  /**
   * Zero every axis and turn off heading control, aiming and translation only mode. The static
   * configuration (deadband, scaling, cubing, alliance and robot relative translation, limits) is kept.
   * Useful at the start of a drive command when the stream is shared between commands.
   *
   * @return this
   */
  public SwerveInputStream reset() {
    translationX = 0;
    translationY = 0;
    rotation = 0;
    headingEnabled = false;
    aimEnabled = false;
    translationOnlyEnabled = false;
    return this;
  }

  /**
   * Calculate the {@link ChassisVelocities} for the current inputs and modes.
   *
   * @return Field relative {@link ChassisVelocities}.
   */
  @Override
  public ChassisVelocities get() {
    var config = swerveDrive.getConfig();
    double maximumVelocity = config.getMaximumChassisLinearVelocity().orElse(maximumChassisLinearVelocity).in(MetersPerSecond);
    double maximumAngularVelocity = config.getMaximumChassisAngularVelocity().orElse(maximumChassisAngularVelocity).in(RadiansPerSecond);

    Translation2d translation = scaleTranslation(applyDeadband(translationX), applyDeadband(translationY));
    if (translationCube) {
      translation = SwerveDriveConfig.cubeTranslation(translation);
    }
    translation = applyAllianceRelative(translation);
    double vx = translation.getX() * maximumVelocity;
    double vy = translation.getY() * maximumVelocity;

    Mode mode = findMode();
    if (mode != currentMode) {
      transitionMode(mode);
    }
    currentMode = mode;

    double omega = switch (mode) {
      case TRANSLATION_ONLY -> requireRotationPID(config).calculate(swerveDrive.getGyroAngle().in(Radians), lockedHeading.orElseThrow().getRadians());
      case AIM -> {
        Rotation2d currentHeading = new Rotation2d(swerveDrive.getGyroAngle());
        Translation2d relativeTarget = aimTarget.orElseThrow().relativeTo(swerveDrive.getPose()).getTranslation();
        Rotation2d target = new Rotation2d(relativeTarget.getX(), relativeTarget.getY()).plus(currentHeading);
        yield requireRotationPID(config).calculate(currentHeading.getRadians(), target.getRadians());
      }
      case HEADING -> requireRotationPID(config).calculate(swerveDrive.getGyroAngle().in(Radians), heading.orElseThrow().in(Radians));
      case ANGULAR_VELOCITY -> {
        final double deadbanded = applyDeadband(rotation);
        final double axis = omegaAxisScale.map(scale -> deadbanded * scale).orElse(deadbanded);
        yield (omegaCube ? Math.pow(axis, 3) : axis) * maximumAngularVelocity;
      }
    };

    ChassisVelocities speeds = new ChassisVelocities(vx, vy, omega);
    if (robotRelative) {
      speeds = speeds.toFieldRelative(new Rotation2d(swerveDrive.getGyroAngle()));
    }
    if (translationHeadingOffsetEnabled) {
      Translation2d offsetTranslation = new Translation2d(speeds.vx, speeds.vy).rotateBy(translationHeadingOffset);
      speeds = new ChassisVelocities(offsetTranslation.getX(), offsetTranslation.getY(), speeds.omega);
    }
    return speeds;
  }

  /** Pick the rotation mode for the current inputs. */
  private Mode findMode() {
    if (translationOnlyEnabled) {
      return Mode.TRANSLATION_ONLY;
    }
    if (aimEnabled && aimTarget.isPresent()) {
      return Mode.AIM;
    }
    if (headingEnabled && heading.isPresent()) {
      return Mode.HEADING;
    }
    return Mode.ANGULAR_VELOCITY;
  }

  /** Reset the rotation PID and lock the heading when switching modes. */
  private void transitionMode(Mode mode) {
    lockedHeading = mode == Mode.TRANSLATION_ONLY ? Optional.of(new Rotation2d(swerveDrive.getGyroAngle())) : Optional.empty();
    if (mode != Mode.ANGULAR_VELOCITY) {
      swerveDrive.resetRotationPID();
    }
  }

  private double applyDeadband(double axis) {
    return axisDeadband.map(deadband -> MathUtil.applyDeadband(axis, deadband)).orElse(axis);
  }

  private Translation2d scaleTranslation(double x, double y) {
    return translationAxisScale.map(scale -> SwerveDriveConfig.scaleTranslation(new Translation2d(x, y), scale)).orElseGet(() -> new Translation2d(x, y));
  }

  private Translation2d applyAllianceRelative(Translation2d translation) {
    if (!allianceRelative) {
      return translation;
    }
    if (robotRelative) {
      throw new IllegalStateException("Cannot use robot relative translation with alliance relative control!");
    }
    if (MatchState.getAlliance().isPresent() && MatchState.getAlliance().get() == Alliance.RED) {
      return translation.rotateBy(Rotation2d.k180deg);
    }
    return translation;
  }

  private static PIDController requireRotationPID(SwerveDriveConfig config) {
    return config.getRotationPID().orElseThrow(() -> new SwerveDriveConfigurationException("No rotation PID controller configured", "Heading, aim, and translation only control are unavailable", "withRotationController(PIDController)"));
  }

  /** Rotation modes, highest priority first. */
  private enum Mode {
    /** Hold the heading the robot had when the mode started. */
    TRANSLATION_ONLY,
    /** Face a target pose. */
    AIM,
    /** Face a field relative heading. */
    HEADING,
    /** Rotate from the rotation axis. */
    ANGULAR_VELOCITY
  }
}
