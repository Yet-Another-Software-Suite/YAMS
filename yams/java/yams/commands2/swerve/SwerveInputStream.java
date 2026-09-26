// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package yams.commands2.swerve;

import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.MetersPerSecond;
import static org.wpilib.units.Units.Radians;
import static org.wpilib.units.Units.RadiansPerSecond;
import static org.wpilib.units.Units.RotationsPerSecond;

import java.util.NoSuchElementException;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.wpilib.driverstation.Alliance;
import org.wpilib.driverstation.DriverStationErrors;
import org.wpilib.driverstation.MatchState;
import org.wpilib.driverstation.NiDsXboxController;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.math.util.MathUtil;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.LinearVelocity;
import org.wpilib.math.controller.PIDController;
import yams.core.exceptions.SwerveDriveConfigurationException;
import yams.core.mechanisms.config.SwerveDriveConfig;
import yams.core.mechanisms.swerve.SwerveDrive;

/**
 * Helper class to easily transform Controller inputs into workable Chassis speeds. Intended to
 * easily create an interface that generates {@link ChassisVelocities} from
 * {@link NiDsXboxController} <p> <br /> Inspired by SciBorgs FRC 1155. <br /> Example: <pre>
 * {@code
 *   NiDsXboxController driverXbox = new NiDsXboxController(0);
 *
 *   SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
 *                                                                 () -> driverXbox.getLeftY() * -1,
 *                                                                 () -> driverXbox.getLeftX() * -1)
 * // Axis which give the desired translational angle and speed.
 *                                                             .withControllerRotationAxis(driverXbox::getRightX)
 * // Axis which give the desired angular velocity. .deadband(0.01)                  // Controller
 * deadband .scaleTranslation(0.8)           // Scaled controller translation axis
 *                                                             .allianceRelativeControl(true);  //
 * Alliance relative controls.
 *
 *   SwerveInputStream driveDirectAngle = driveAngularVelocity.copy()  // Copy the stream so further
 * changes do not affect driveAngularVelocity .withControllerHeadingAxis(driverXbox::getRightX,
 *                                                                                       driverXbox::getRightY)
 * // Axis which give the desired heading angle using trigonometry. .headingWhile(true); // Enable
 * heading based control.
 * }
 * </pre>
 *
 * <h2>Joystick-to-{@link yams.core.mechanisms.swerve.SwerveDrive} adapter</h2>
 * <p>
 * {@link SwerveInputStream} acts as a bridge between raw controller axis values (in the range
 * {@code [-1, 1]}) and the {@link ChassisVelocities} that
 * {@link yams.core.mechanisms.swerve.SwerveDrive} expects. It handles deadbands, axis scaling, non-linear
 * (cubed) response curves, field-relative / alliance-relative flipping, and multiple drive modes
 * (angular-velocity, heading-snap, translation-only, aim-at-target). Because it
 * implements
 * {@link java.util.function.Supplier}{@code <}{@link ChassisVelocities}{@code >} it can be passed
 * directly to a command that calls {@code swerveDrive.drive(inputStream.get())}.
 * </p>
 *
 * <h2>Typical usage with an {@link NiDsXboxController}</h2>
 * <pre>{@code
 * NiDsXboxController driver = new NiDsXboxController(0);
 *
 * // Angular-velocity stream: left stick translates, right stick X rotates
 * SwerveInputStream angularVelocityStream =
 *     SwerveInputStream.of(swerveDrive,
 *                          () -> -driver.getLeftY(),   // forward/back
 *                          () -> -driver.getLeftX())   // strafe
 *                      .withControllerRotationAxis(driver::getRightX)
 *                      .withDeadband(0.05)
 *                      .withScaleTranslation(0.8)
 *                      .withScaleRotation(0.6)
 *                      .withAllianceRelativeControl()  // auto-flip for Red alliance
 *                      .withCubeTranslationControllerAxis(); // non-linear response
 *
 * // Heading-snap variant: right stick X/Y picks a desired heading angle
 * SwerveInputStream headingStream = angularVelocityStream.clone()
 *     .withControllerHeadingAxis(driver::getRightX, driver::getRightY)
 *     .withHeadingControl(() -> driver.getRightStickButton());
 *
 * // In your periodic or a command, call .get() to obtain ChassisVelocities:
 * swerveDrive.drive(angularVelocityStream.get());
 * }</pre>
 */
public class SwerveInputStream implements Supplier<ChassisVelocities> {
  /**
   * Translation suppliers.
   */
  private final DoubleSupplier            controllerTranslationX;
  /**
   * Translational supplier.
   */
  private final DoubleSupplier            controllerTranslationY;
  /**
   * {@link SwerveDrive} object for transformations.
   */
  private final SwerveDrive               swerveDrive;
  /**
   * Rotation supplier as angular velocity.
   */
  private Optional<DoubleSupplier>        controllerOmega                     = Optional.empty();
  /**
   * Controller heading axis X, kept to hold the heading while the stick is inside the deadband.
   */
  private Optional<DoubleSupplier>        controllerHeadingX                  = Optional.empty();
  /**
   * Controller heading axis Y, kept to hold the heading while the stick is inside the deadband.
   */
  private Optional<DoubleSupplier>        controllerHeadingY                  = Optional.empty();
  /**
   * Field relative heading to face in {@link SwerveInputMode#HEADING}.
   */
  private Optional<Supplier<Angle>>       headingSupplier                     = Optional.empty();
  /**
   * Axis deadband for the controller.
   */
  private Optional<Double>                axisDeadband                        = Optional.empty();
  /**
   * Translational axis scalar value, should be between (0, 1].
   */
  private Optional<Double>                translationAxisScale                = Optional.empty();
  /**
   * Angular velocity axis scalar value, should be between (0, 1]
   */
  private Optional<Double>                omegaAxisScale                      = Optional.empty();
  /**
   * Target to aim at.
   */
  private Optional<Supplier<Pose2d>>      aimTarget                           = Optional.empty();
  /**
   * Output {@link ChassisVelocities} based on heading while this is True.
   */
  private Optional<BooleanSupplier>       headingEnabled                      = Optional.empty();
  /**
   * Locked heading for {@link SwerveInputMode#TRANSLATION_ONLY}
   */
  private Optional<Rotation2d>            lockedHeading                       = Optional.empty();
  /**
   * Output {@link ChassisVelocities} based on aim while this is True.
   */
  private Optional<BooleanSupplier>       aimEnabled                          = Optional.empty();
  /**
   * Maintain current heading and drive without rotating, ideally.
   */
  private Optional<BooleanSupplier>       translationOnlyEnabled              = Optional.empty();
  /**
   * Cube the translation magnitude from the controller.
   */
  private Optional<BooleanSupplier>       translationCube                     = Optional.empty();
  /**
   * Cube the angular velocity axis from the controller.
   */
  private Optional<BooleanSupplier>       omegaCube                           = Optional.empty();
  /**
   * Robot relative oriented output expected.
   */
  private Optional<BooleanSupplier>       robotRelative                       = Optional.empty();
  /**
   * Field oriented chassis output is relative to your current alliance.
   */
  private Optional<BooleanSupplier>       allianceRelative                    = Optional.empty();
  /**
   * Heading offset enable state.
   */
  private Optional<BooleanSupplier>       translationHeadingOffsetEnabled     = Optional.empty();
  /**
   * Heading offset to apply during heading based control.
   */
  private Optional<Rotation2d>            translationHeadingOffset            = Optional.empty();
  /**
   * Current {@link SwerveInputMode} to use.
   */
  private SwerveInputMode                 currentMode                         = SwerveInputMode.ANGULAR_VELOCITY;
  /**
   * Maximum chassis velocity, defaults to 4 Meters per Second
   */
  private LinearVelocity                  maximumChassisLinearVelocity        = MetersPerSecond.of(4);
  /**
   * Maximum chassis angular velocity, defaults to 1 Rotation per Second.
   */
  private AngularVelocity                 maximumChassisAngularVelocity       = RotationsPerSecond.of(1);

  /**
   * Create a {@link SwerveInputStream} for an easy way to generate {@link ChassisVelocities} from a
   * driver controller.
   *
   * @param drive {@link SwerveDrive} object for transformation.
   * @param x     Translation X input in range of [-1, 1]
   * @param y     Translation Y input in range of [-1, 1]
   */
  private SwerveInputStream(SwerveDrive drive, DoubleSupplier x, DoubleSupplier y) {
    controllerTranslationX = x;
    controllerTranslationY = y;
    swerveDrive = drive;
  }

  /**
   * Create a {@link SwerveInputStream} for an easy way to generate {@link ChassisVelocities} from a
   * driver controller.
   *
   * @param drive {@link SwerveDrive} object for transformation.
   * @param x     Translation X input in range of [-1, 1]
   * @param y     Translation Y input in range of [-1, 1]
   * @param rot   Rotation input in range of [-1, 1]
   */
  public SwerveInputStream(SwerveDrive drive, DoubleSupplier x, DoubleSupplier y, DoubleSupplier rot) {
    this(drive, x, y);
    controllerOmega = Optional.of(rot);
  }

  /**
   * Create a {@link SwerveInputStream} for an easy way to generate {@link ChassisVelocities} from a
   * driver controller.
   *
   * @param drive    {@link SwerveDrive} object for transformation.
   * @param x        Translation X input in range of [-1, 1]
   * @param y        Translation Y input in range of [-1, 1]
   * @param headingX Heading X input in range of [-1, 1]
   * @param headingY Heading Y input in range of [-1, 1]
   */
  public SwerveInputStream(SwerveDrive drive, DoubleSupplier x, DoubleSupplier y, DoubleSupplier headingX, DoubleSupplier headingY) {
    this(drive, x, y);
    withControllerHeadingAxis(headingX, headingY);
  }

  /**
   * Create basic {@link SwerveInputStream} without any rotation components.
   *
   * @param drive {@link SwerveDrive} object for transformation.
   * @param x     {@link DoubleSupplier} of the translation X axis of the controller joystick to
   *              use.
   * @param y     {@link DoubleSupplier} of the translation X axis of the controller joystick to
   *              use.
   * @return {@link SwerveInputStream} to use as you see fit.
   */
  public static SwerveInputStream of(SwerveDrive drive, DoubleSupplier x, DoubleSupplier y) {
    return new SwerveInputStream(drive, x, y);
  }

  /**
   * Clone the {@link SwerveInputStream} object.
   *
   * @param cfg {@link SwerveInputStream} to clone.
   */
  private SwerveInputStream(SwerveInputStream cfg) {
    swerveDrive = cfg.swerveDrive;
    controllerTranslationX = cfg.controllerTranslationX;
    controllerTranslationY = cfg.controllerTranslationY;
    controllerOmega = cfg.controllerOmega;
    controllerHeadingX = cfg.controllerHeadingX;
    controllerHeadingY = cfg.controllerHeadingY;
    axisDeadband = cfg.axisDeadband;
    translationAxisScale = cfg.translationAxisScale;
    omegaAxisScale = cfg.omegaAxisScale;
    aimTarget = cfg.aimTarget;
    headingEnabled = cfg.headingEnabled;
    headingSupplier = cfg.headingSupplier;
    aimEnabled = cfg.aimEnabled;
    currentMode = cfg.currentMode;
    translationOnlyEnabled = cfg.translationOnlyEnabled;
    lockedHeading = cfg.lockedHeading;
    omegaCube = cfg.omegaCube;
    translationCube = cfg.translationCube;
    robotRelative = cfg.robotRelative;
    allianceRelative = cfg.allianceRelative;
    translationHeadingOffsetEnabled = cfg.translationHeadingOffsetEnabled;
    translationHeadingOffset = cfg.translationHeadingOffset;
    maximumChassisLinearVelocity = cfg.maximumChassisLinearVelocity;
    maximumChassisAngularVelocity = cfg.maximumChassisAngularVelocity;
  }

  /**
   * Copy the {@link SwerveInputStream} object.
   *
   * @return Clone of current {@link SwerveInputStream}
   */
  @Override
  public SwerveInputStream clone() {
    return new SwerveInputStream(this);
  }

  /**
   * Maximum linear velocity to use for the {@link SwerveDrive} object, will be translated to Meters
   * per Second. Default is 4 Meters per Second.
   *
   * @param velocity Linear velocity to use.
   * @return {@link SwerveInputStream} to use as you see fit.
   */
  public SwerveInputStream withMaximumLinearVelocity(LinearVelocity velocity) {
    maximumChassisLinearVelocity = velocity;
    return this;
  }

  /**
   * Maximum angular velocity to use for the {@link SwerveDrive} object, will be translated to
   * Rotations per Second. Default is 1 Rotation per Second.
   *
   * @param velocity Angular velocity to use.
   * @return {@link SwerveInputStream} to use as you see fit.
   */
  public SwerveInputStream withMaximumAngularVelocity(AngularVelocity velocity) {
    maximumChassisAngularVelocity = velocity;
    return this;
  }

  /**
   * Set the stream to output robot relative {@link ChassisVelocities}
   *
   * @param enabled Robot-Relative {@link ChassisVelocities} output.
   * @return self
   */
  public SwerveInputStream withRobotRelative(BooleanSupplier enabled) {
    robotRelative = Optional.of(enabled);
    return this;
  }

  /**
   * Set the stream to output robot relative {@link ChassisVelocities}
   *
   * @return self
   */
  public SwerveInputStream withRobotRelative() {
    return withRobotRelative(() -> true);
  }

  /**
   * Heading offset enabled boolean supplier.
   *
   * @param angle   {@link Rotation2d} offset to apply
   * @param enabled Enable state
   * @return self
   */
  public SwerveInputStream withTranslationHeadingOffset(Rotation2d angle, BooleanSupplier enabled) {
    translationHeadingOffset = Optional.of(angle);
    translationHeadingOffsetEnabled = Optional.of(enabled);
    return this;
  }

  /**
   * Set the heading offset angle.
   *
   * @param angle {@link Rotation2d} offset to apply
   * @return self
   */
  public SwerveInputStream withTranslationHeadingOffset(Rotation2d angle) {
    return withTranslationHeadingOffset(angle, () -> true);
  }

  /**
   * Modify the output {@link ChassisVelocities} so that it is always relative to your alliance.
   * Has no effect while robot relative control is enabled.
   *
   * @return self
   */
  public SwerveInputStream withAllianceRelativeControl() {
    return withAllianceRelativeControl(() -> true);
  }

  /**
   * Modify the output {@link ChassisVelocities} so that it is always relative to your alliance.
   * Has no effect while robot relative control is enabled.
   *
   * @param enabled Alliance aware {@link ChassisVelocities} output.
   * @return self
   */
  public SwerveInputStream withAllianceRelativeControl(BooleanSupplier enabled) {
    allianceRelative = Optional.ofNullable(enabled);
    return this;
  }

  /**
   * Cube the angular velocity controller axis for a non-linear controls scheme.
   *
   * @param enabled Enabled state for the stream.
   * @return self.
   */
  public SwerveInputStream withCubeRotationControllerAxis(BooleanSupplier enabled) {
    omegaCube = Optional.of(enabled);
    return this;
  }

  /**
   * Cube the angular velocity controller axis for a non-linear controls scheme.
   *
   * @return self.
   */
  public SwerveInputStream withCubeRotationControllerAxis() {
    return withCubeRotationControllerAxis(() -> true);
  }

  /**
   * Cube the translation axis magnitude for a non-linear control scheme.
   *
   * @param enabled Enabled state for the stream
   * @return self
   */
  public SwerveInputStream withCubeTranslationControllerAxis(BooleanSupplier enabled) {
    translationCube = Optional.of(enabled);
    return this;
  }

  /**
   * Cube the translation axis magnitude for a non-linear control scheme
   *
   * @return self
   */
  public SwerveInputStream withCubeTranslationControllerAxis() {
    return withCubeTranslationControllerAxis(() -> true);
  }

  /**
   * Add a rotation axis for Angular Velocity control
   *
   * @param rot Rotation axis with values from [-1, 1]
   * @return self
   */
  public SwerveInputStream withControllerRotationAxis(DoubleSupplier rot) {
    controllerOmega = Optional.of(rot);
    return this;
  }

  /**
   * Add heading axis for Heading based control.
   *
   * @param headingX Heading X axis with values from [-1, 1]
   * @param headingY Heading Y axis with values from [-1, 1]
   * @return self
   */
  public SwerveInputStream withControllerHeadingAxis(DoubleSupplier headingX, DoubleSupplier headingY) {
    // The stick points at the heading to face; atan2(x, y) turns it into that heading.
    withHeading(() -> Radians.of(Math.atan2(headingX.getAsDouble(), headingY.getAsDouble())));
    controllerHeadingX = Optional.of(headingX);
    controllerHeadingY = Optional.of(headingY);
    return this;
  }

  /**
   * Set a deadband for all controller axis.
   *
   * @param deadband Deadband to set, should be between [0, 1)
   * @return self
   */
  public SwerveInputStream deadband(double deadband) {
    axisDeadband = deadband == 0 ? Optional.empty() : Optional.of(deadband);
    return this;
  }

  /**
   * Set a deadband for all controller axis.
   *
   * @param deadband Deadband to set, should be between [0, 1)
   * @return self
   */
  public SwerveInputStream withDeadband(double deadband) {
    return deadband(deadband);
  }

  /**
   * Scale the translation axis for {@link SwerveInputStream} by a constant scalar value.
   *
   * @param scaleTranslation Translation axis scalar value. (0, 1]
   * @return this
   */
  public SwerveInputStream withScaleTranslation(double scaleTranslation) {
    translationAxisScale = scaleTranslation == 0 ? Optional.empty() : Optional.of(scaleTranslation);
    return this;
  }

  /**
   * Scale the rotation axis input for {@link SwerveInputStream} to reduce the range in which they
   * operate.
   *
   * @param scaleRotation Angular velocity axis scalar value. (0, 1]
   * @return this
   */
  public SwerveInputStream withScaleRotation(double scaleRotation) {
    omegaAxisScale = scaleRotation == 0 ? Optional.empty() : Optional.of(scaleRotation);
    return this;
  }

  /**
   * Output {@link ChassisVelocities} based on heading while the supplier is True.
   *
   * @param trigger Supplier to use.
   * @return this.
   */
  public SwerveInputStream withHeadingControl(BooleanSupplier trigger) {
    headingEnabled = Optional.of(trigger);
    return this;
  }

  /**
   * Supply the field relative heading to face in heading mode, which is enabled with
   * {@link #withHeadingControl(BooleanSupplier)}. Replaces any controller heading axis.
   *
   * @param heading Field relative heading to face, blue-origin where 0 degrees faces the red
   *                alliance wall.
   * @return this.
   */
  public SwerveInputStream withHeading(Supplier<Angle> heading) {
    headingSupplier = Optional.ofNullable(heading);
    controllerHeadingX = Optional.empty();
    controllerHeadingY = Optional.empty();
    return this;
  }

  /**
   * Aim the {@link SwerveDrive} at this pose while driving.
   *
   * @param trigger   When True will enable aiming at the current target.
   * @param aimTarget {@link Pose2d} to point at.
   * @return this
   */
  public SwerveInputStream withAim(Supplier<Pose2d> aimTarget, BooleanSupplier trigger) {
    this.aimTarget = aimTarget.equals(Pose2d.ZERO) ? Optional.empty() : Optional.of(aimTarget);
    aimEnabled = Optional.of(trigger);
    return this;
  }

  /**
   * Enable locking of rotation and only translating, overrides everything.
   *
   * @param trigger Translation only while returns true.
   * @return this
   */
  public SwerveInputStream withTranslationOnly(BooleanSupplier trigger) {
    translationOnlyEnabled = Optional.of(trigger);
    return this;
  }

  /**
   * Find {@link SwerveInputMode} based off existing parameters of the {@link SwerveInputStream}
   *
   * @return The calculated {@link SwerveInputMode}, defaults to
   *         {@link SwerveInputMode#ANGULAR_VELOCITY}.
   */
  private SwerveInputMode findMode() {
    if (translationOnlyEnabled.isPresent() && translationOnlyEnabled.get().getAsBoolean()) {
      return SwerveInputMode.TRANSLATION_ONLY;
    } else if (aimEnabled.isPresent() && aimEnabled.get().getAsBoolean()) {
      if (aimTarget.isPresent()) {
        return SwerveInputMode.AIM;
      } else {
        DriverStationErrors.reportError("Attempting to enter AIM mode without target, please use " + "SwerveInputStream.aim() to select a target first!", false);
      }
    } else if (headingEnabled.isPresent() && headingEnabled.get().getAsBoolean()) {
      if (headingSupplier.isPresent()) {
        return SwerveInputMode.HEADING;
      } else {
        DriverStationErrors.reportError("Attempting to enter HEADING mode without a heading, please use " + "SwerveInputStream.withHeading or SwerveInputStream.withControllerHeadingAxis to add one!", false);
      }
    } else if (controllerOmega.isEmpty()) {
      DriverStationErrors.reportError("Attempting to enter ANGULAR_VELOCITY mode without a rotation axis, please use " + "SwerveInputStream.withControllerRotationAxis to add angular velocity axis!", false);
      return SwerveInputMode.TRANSLATION_ONLY;
    }
    return SwerveInputMode.ANGULAR_VELOCITY;
  }

  /**
   * Transition smoothly from one mode to another.
   *
   * @param newMode New mode to transition too.
   */
  private void transitionMode(SwerveInputMode newMode) {
    // Handle removing of current mode.
    switch (currentMode) {
      case TRANSLATION_ONLY -> {
        lockedHeading = Optional.empty();
        break;
      }
      case ANGULAR_VELOCITY -> {
        // Do nothing
        break;
      }
      case HEADING, AIM -> {
        swerveDrive.resetRotationPID();
        break;
      }
    }

    // Transitioning to new mode
    switch (newMode) {
      case TRANSLATION_ONLY -> {
        lockedHeading = Optional.of(new Rotation2d(swerveDrive.getGyroAngle()));
        swerveDrive.resetRotationPID();
        break;
      }
      case ANGULAR_VELOCITY -> {
        break;
      }
      case HEADING, AIM -> {
        swerveDrive.resetRotationPID();
        break;
      }
    }
  }

  /**
   * Apply the deadband if it exists.
   *
   * @param axisValue Axis value to apply the deadband too.
   * @return axis value with deadband, else axis value straight.
   */
  private double applyDeadband(double axisValue) {
    return axisDeadband.map(aDouble -> MathUtil.applyDeadband(axisValue, aDouble)).orElse(axisValue);
  }

  /**
   * Apply the scalar value if it exists.
   *
   * @param axisValue Axis value to apply the scalar to.
   * @return Axis value scaled by scalar value.
   */
  private double applyRotationalScalar(double axisValue) {
    return omegaAxisScale.map(aDouble -> axisValue * aDouble).orElse(axisValue);
  }

  /**
   * Scale the translational axis by the {@link SwerveInputStream#translationAxisScale} if it
   * exists.
   *
   * @param xAxis X axis to scale.
   * @param yAxis Y axis to scale.
   * @return Scaled {@link Translation2d}
   */
  private Translation2d applyTranslationScalar(double xAxis, double yAxis) {
    return translationAxisScale.map(aDouble -> SwerveDriveConfig.scaleTranslation(new Translation2d(xAxis, yAxis), aDouble)).orElseGet(() -> new Translation2d(xAxis, yAxis));
  }

  /**
   * Apply the cube transformation on the given {@link Translation2d}
   *
   * @param translation {@link Translation2d} representing controller input
   * @return Cubed {@link Translation2d} if the {@link SwerveInputStream#translationCube} is
   *         present.
   */
  private Translation2d applyTranslationCube(Translation2d translation) {
    if (translationCube.isPresent() && translationCube.get().getAsBoolean()) {
      return SwerveDriveConfig.cubeTranslation(translation);
    }
    return translation;
  }

  /**
   * Apply the cube transformation on the given rotation controller axis
   *
   * @param rotationAxis Rotation controller axis to cube.
   * @return Cubed axis value if the {@link SwerveInputStream#omegaCube} is present.
   */
  private double applyOmegaCube(double rotationAxis) {
    if (omegaCube.isPresent() && omegaCube.get().getAsBoolean()) {
      return Math.pow(rotationAxis, 3);
    }
    return rotationAxis;
  }

  /**
   * Change {@link ChassisVelocities} from robot relative if enabled.
   *
   * @param fieldRelativeSpeeds Field or robot relative speeds to translate into robot-relative
   *                            speeds.
   * @return Field relative {@link ChassisVelocities}.
   */
  private ChassisVelocities applyRobotRelativeTranslation(ChassisVelocities fieldRelativeSpeeds) {
    if (robotRelative.isPresent() && robotRelative.get().getAsBoolean()) {
      return fieldRelativeSpeeds.toFieldRelative(new Rotation2d(swerveDrive.getGyroAngle()));
    }
    return fieldRelativeSpeeds;
  }

  /**
   * Apply alliance aware translation which flips the {@link Translation2d} if the robot is on the
   * Red alliance. Skipped while robot relative control is enabled, since robot relative translation
   * does not depend on the alliance.
   *
   * @param fieldRelativeTranslation Field-relative {@link Translation2d} to flip.
   * @return Alliance-oriented {@link Translation2d}
   */
  private Translation2d applyAllianceAwareTranslation(Translation2d fieldRelativeTranslation) {
    if (robotRelative.isPresent() && robotRelative.get().getAsBoolean()) {
      return fieldRelativeTranslation;
    }
    if (allianceRelative.isPresent() && allianceRelative.get().getAsBoolean()) {
      if (MatchState.getAlliance().isPresent() && MatchState.getAlliance().get() == Alliance.RED) {
        return fieldRelativeTranslation.rotateBy(Rotation2d.k180deg);
      }
    }
    return fieldRelativeTranslation;
  }

  /**
   * Adds offset to translation if one is set.
   *
   * @param speeds {@link ChassisVelocities} to offset
   * @return Offsetted {@link ChassisVelocities}
   */
  private ChassisVelocities applyTranslationHeadingOffset(ChassisVelocities speeds) {
    if (translationHeadingOffsetEnabled.isPresent() && translationHeadingOffsetEnabled.get().getAsBoolean()) {
      if (translationHeadingOffset.isPresent()) {
        Translation2d speedsTranslation = new Translation2d(speeds.vx, speeds.vy).rotateBy(translationHeadingOffset.get());
        return new ChassisVelocities(speedsTranslation.getX(), speedsTranslation.getY(), speeds.omega);
      }
    }
    return speeds;
  }

  /**
   * Get the rotation PID controller needed by the heading, aim, and translation only modes.
   *
   * @param config {@link SwerveDriveConfig} of the drive.
   * @return Rotation PID controller.
   * @throws SwerveDriveConfigurationException if no rotation PID controller is configured.
   */
  private static PIDController requireRotationPID(SwerveDriveConfig config) {
    return config.getRotationPID().orElseThrow(() -> new SwerveDriveConfigurationException("No rotation PID controller configured", "Heading, aim, and translation only control are unavailable", "withRotationController(PIDController)"));
  }

  /**
   * Calculate the {@link ChassisVelocities} for the current controller inputs and active mode.
   *
   * @return Field relative {@link ChassisVelocities} for the current inputs.
   * @throws SwerveDriveConfigurationException if translation only mode is active (including the
   *                                           fallback to translation only mode when no
   *                                           controller rotation axis is set) and no rotation PID
   *                                           controller is configured.
   * @throws SwerveDriveConfigurationException if heading mode is active and no rotation PID
   *                                           controller is configured.
   * @throws SwerveDriveConfigurationException if aim mode is active and no rotation PID controller
   *                                           is configured.
   * @throws NoSuchElementException            if heading mode is requested without a heading
   *                                           configured and no controller rotation axis is set,
   *                                           so the stream falls back to angular velocity mode
   *                                           without an axis.
   * @throws NoSuchElementException            if aim mode is requested without an aim target
   *                                           configured and no controller rotation axis is set,
   *                                           so the stream falls back to angular velocity mode
   *                                           without an axis.
   */
  @Override
  public ChassisVelocities get() {
    var config = swerveDrive.getConfig();
    double maximumChassisVelocity = config.getMaximumChassisLinearVelocity().orElse(maximumChassisLinearVelocity).in(MetersPerSecond);
    double maximumChassisRotVelocity = config.getMaximumChassisAngularVelocity().orElse(maximumChassisAngularVelocity).in(RadiansPerSecond);
    Translation2d scaledTranslation = applyTranslationScalar(applyDeadband(controllerTranslationX.getAsDouble()), applyDeadband(controllerTranslationY.getAsDouble()));
    scaledTranslation = applyTranslationCube(scaledTranslation);
    scaledTranslation = applyAllianceAwareTranslation(scaledTranslation);

    double vxMetersPerSecond = scaledTranslation.getX() * maximumChassisVelocity;
    double vyMetersPerSecond = scaledTranslation.getY() * maximumChassisVelocity;
    double omegaRadiansPerSecond = 0;
    ChassisVelocities speeds = new ChassisVelocities();

    SwerveInputMode newMode = findMode();
    // Handle transitions here.
    if (currentMode != newMode) {
      transitionMode(newMode);
    }
    switch (newMode) {
      case TRANSLATION_ONLY -> {
        var azimuthPIDs = requireRotationPID(config);

        omegaRadiansPerSecond = azimuthPIDs.calculate(swerveDrive.getGyroAngle().in(Radians), lockedHeading.orElseThrow().getRadians());
        speeds = new ChassisVelocities(vxMetersPerSecond, vyMetersPerSecond, omegaRadiansPerSecond);
        break;
      }
      case ANGULAR_VELOCITY -> {
        omegaRadiansPerSecond = applyOmegaCube(applyRotationalScalar(applyDeadband(controllerOmega.orElseThrow().getAsDouble()))) * maximumChassisRotVelocity;
        speeds = new ChassisVelocities(vxMetersPerSecond, vyMetersPerSecond, omegaRadiansPerSecond);
        break;
      }
      case HEADING -> {
        var azimuthPIDs = requireRotationPID(config);
        omegaRadiansPerSecond = azimuthPIDs.calculate(swerveDrive.getGyroAngle().in(Radians), headingSupplier.orElseThrow().get().in(Radians));

        // Prevent rotation if controller heading inputs are not past axisDeadband
        if (controllerHeadingX.isPresent() && controllerHeadingY.isPresent() && axisDeadband.isPresent() &&
            Math.abs(controllerHeadingX.get().getAsDouble()) + Math.abs(controllerHeadingY.get().getAsDouble()) < axisDeadband.get()) {
          omegaRadiansPerSecond = 0;
        }
        speeds = new ChassisVelocities(vxMetersPerSecond, vyMetersPerSecond, omegaRadiansPerSecond);
        break;
      }
      case AIM -> {
        var azimuthPIDs = requireRotationPID(config);
        Rotation2d currentHeading = new Rotation2d(swerveDrive.getGyroAngle());
        Translation2d relativeTrl = aimTarget.orElseThrow().get().relativeTo(swerveDrive.getPose()).getTranslation();
        Rotation2d target = new Rotation2d(relativeTrl.getX(), relativeTrl.getY()).plus(currentHeading);
        omegaRadiansPerSecond = azimuthPIDs.calculate(currentHeading.getRadians(), target.getRadians());
        speeds = new ChassisVelocities(vxMetersPerSecond, vyMetersPerSecond, omegaRadiansPerSecond);
        break;
      }
    }

    currentMode = newMode;

    return applyTranslationHeadingOffset(applyRobotRelativeTranslation(speeds));
  }

  /**
   * Drive modes to keep track of.
   */
  enum SwerveInputMode {
    /**
     * Translation only mode, does not allow for rotation and maintains current heading.
     */
    TRANSLATION_ONLY,
    /**
     * Output based off angular velocity
     */
    ANGULAR_VELOCITY,
    /**
     * Output based off of heading.
     */
    HEADING,
    /**
     * Output based off of targeting.
     */
    AIM
  }
}
