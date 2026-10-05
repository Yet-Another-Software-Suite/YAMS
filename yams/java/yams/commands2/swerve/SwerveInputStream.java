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
  /** Translation suppliers. */
  private final DoubleSupplier controllerTranslationX;
  /** Translational supplier. */
  private final DoubleSupplier controllerTranslationY;
  /** {@link SwerveDrive} object for transformations. */
  private final SwerveDrive swerveDrive;
  /** Rotation supplier as angular velocity. */
  private Optional<DoubleSupplier> controllerOmega = Optional.empty();
  /** Controller heading axis X, kept to hold the heading while the stick is inside the deadband. */
  private Optional<DoubleSupplier> controllerHeadingX = Optional.empty();
  /** Controller heading axis Y, kept to hold the heading while the stick is inside the deadband. */
  private Optional<DoubleSupplier> controllerHeadingY = Optional.empty();
  /** Field relative heading to face in {@link SwerveInputMode#HEADING}. */
  private Optional<Supplier<Angle>> headingSupplier = Optional.empty();
  /** Axis deadband for the controller. */
  private Optional<Double> axisDeadband = Optional.empty();
  /** Translational axis scalar value, should be between (0, 1]. */
  private Optional<Double> translationAxisScale = Optional.empty();
  /** Angular velocity axis scalar value, should be between (0, 1] */
  private Optional<Double> omegaAxisScale = Optional.empty();
  /** Target to aim at. */
  private Optional<Supplier<Pose2d>> aimTarget = Optional.empty();
  /** Output {@link ChassisVelocities} based on heading while this is True. */
  private Optional<BooleanSupplier> headingEnabled = Optional.empty();
  /** Locked heading for {@link SwerveInputMode#TRANSLATION_ONLY} */
  private Optional<Rotation2d> lockedHeading = Optional.empty();
  /** Output {@link ChassisVelocities} based on aim while this is True. */
  private Optional<BooleanSupplier> aimEnabled = Optional.empty();
  /** Maintain current heading and drive without rotating, ideally. */
  private Optional<BooleanSupplier> translationOnlyEnabled = Optional.empty();
  /** Cube the translation magnitude from the controller. */
  private Optional<BooleanSupplier> translationCube = Optional.empty();
  /** Cube the angular velocity axis from the controller. */
  private Optional<BooleanSupplier> omegaCube = Optional.empty();
  /** Robot relative oriented output expected. */
  private Optional<BooleanSupplier> robotRelative = Optional.empty();
  /** Field oriented chassis output is relative to your current alliance. */
  private Optional<BooleanSupplier> allianceRelative = Optional.empty();
  /** Heading offset enable state. */
  private Optional<BooleanSupplier> translationHeadingOffsetEnabled = Optional.empty();
  /** Heading offset to apply during heading based control. */
  private Optional<Rotation2d> translationHeadingOffset = Optional.empty();
  /** Current {@link SwerveInputMode} to use. */
  private SwerveInputMode currentMode = SwerveInputMode.ANGULAR_VELOCITY;
  /** Maximum chassis velocity, defaults to 4 Meters per Second */
  private LinearVelocity maximumChassisLinearVelocity = MetersPerSecond.of(4);
  /** Maximum chassis angular velocity, defaults to 1 Rotation per Second. */
  private AngularVelocity maximumChassisAngularVelocity = RotationsPerSecond.of(1);

  private SwerveInputStream(SwerveDrive drive, DoubleSupplier x, DoubleSupplier y) {
    controllerTranslationX = x;
    controllerTranslationY = y;
    swerveDrive = drive;
  }

  public SwerveInputStream(SwerveDrive drive, DoubleSupplier x, DoubleSupplier y, DoubleSupplier rot) {
    this(drive, x, y);
    controllerOmega = Optional.of(rot);
  }

  public SwerveInputStream(
      SwerveDrive drive,
      DoubleSupplier x,
      DoubleSupplier y,
      DoubleSupplier headingX,
      DoubleSupplier headingY) {
    this(drive, x, y);
    withControllerHeadingAxis(headingX, headingY);
  }

  public static SwerveInputStream of(SwerveDrive drive, DoubleSupplier x, DoubleSupplier y) {
    return new SwerveInputStream(drive, x, y);
  }

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

  @Override
  public SwerveInputStream clone() {
    return new SwerveInputStream(this);
  }

  public String getCurrentModeName() {
    return currentMode.name();
  }

  public double getAxisDeadband() {
    return axisDeadband.orElse(0.0);
  }

  public void setAxisDeadband(double deadband) {
    axisDeadband = deadband == 0 ? Optional.empty() : Optional.of(deadband);
  }

  public double getTranslationAxisScale() {
    return translationAxisScale.orElse(1.0);
  }

  public void setTranslationAxisScale(double scaleTranslation) {
    translationAxisScale = scaleTranslation == 0 ? Optional.empty() : Optional.of(scaleTranslation);
  }

  public double getOmegaAxisScale() {
    return omegaAxisScale.orElse(1.0);
  }

  public void setOmegaAxisScale(double scaleRotation) {
    omegaAxisScale = scaleRotation == 0 ? Optional.empty() : Optional.of(scaleRotation);
  }

  public LinearVelocity getMaximumChassisLinearVelocity() {
    return maximumChassisLinearVelocity;
  }

  public void setMaximumChassisLinearVelocity(LinearVelocity velocity) {
    maximumChassisLinearVelocity = velocity;
  }

  public AngularVelocity getMaximumChassisAngularVelocity() {
    return maximumChassisAngularVelocity;
  }

  public void setMaximumChassisAngularVelocity(AngularVelocity velocity) {
    maximumChassisAngularVelocity = velocity;
  }

  public boolean isTranslationCubeEnabled() {
    return translationCube.isPresent() && translationCube.get().getAsBoolean();
  }

  public void setTranslationCubeEnabled(boolean enabled) {
    translationCube = enabled ? Optional.of(() -> true) : Optional.empty();
  }

  public boolean isOmegaCubeEnabled() {
    return omegaCube.isPresent() && omegaCube.get().getAsBoolean();
  }

  public void setOmegaCubeEnabled(boolean enabled) {
    omegaCube = enabled ? Optional.of(() -> true) : Optional.empty();
  }

  public boolean isAllianceRelativeEnabled() {
    return allianceRelative.isPresent() && allianceRelative.get().getAsBoolean();
  }

  public void setAllianceRelativeEnabled(boolean enabled) {
    allianceRelative = enabled ? Optional.of(() -> true) : Optional.empty();
  }

  public boolean isRobotRelativeEnabled() {
    return robotRelative.isPresent() && robotRelative.get().getAsBoolean();
  }

  public void setRobotRelativeEnabled(boolean enabled) {
    robotRelative = enabled ? Optional.of(() -> true) : Optional.empty();
  }

  public SwerveInputStream withMaximumLinearVelocity(LinearVelocity velocity) {
    maximumChassisLinearVelocity = velocity;
    return this;
  }

  public SwerveInputStream withMaximumAngularVelocity(AngularVelocity velocity) {
    maximumChassisAngularVelocity = velocity;
    return this;
  }

  public SwerveInputStream withRobotRelative(BooleanSupplier enabled) {
    robotRelative = Optional.of(enabled);
    return this;
  }

  public SwerveInputStream withRobotRelative() {
    return withRobotRelative(() -> true);
  }

  public SwerveInputStream withTranslationHeadingOffset(Rotation2d angle, BooleanSupplier enabled) {
    translationHeadingOffset = Optional.of(angle);
    translationHeadingOffsetEnabled = Optional.of(enabled);
    return this;
  }

  public SwerveInputStream withTranslationHeadingOffset(Rotation2d angle) {
    return withTranslationHeadingOffset(angle, () -> true);
  }

  public SwerveInputStream withAllianceRelativeControl() {
    return withAllianceRelativeControl(() -> true);
  }

  public SwerveInputStream withAllianceRelativeControl(BooleanSupplier enabled) {
    allianceRelative = Optional.ofNullable(enabled);
    return this;
  }

  public SwerveInputStream withCubeRotationControllerAxis(BooleanSupplier enabled) {
    omegaCube = Optional.of(enabled);
    return this;
  }

  public SwerveInputStream withCubeRotationControllerAxis() {
    return withCubeRotationControllerAxis(() -> true);
  }

  public SwerveInputStream withCubeTranslationControllerAxis(BooleanSupplier enabled) {
    translationCube = Optional.of(enabled);
    return this;
  }

  public SwerveInputStream withCubeTranslationControllerAxis() {
    return withCubeTranslationControllerAxis(() -> true);
  }

  public SwerveInputStream withControllerRotationAxis(DoubleSupplier rot) {
    controllerOmega = Optional.of(rot);
    return this;
  }

  public SwerveInputStream withControllerHeadingAxis(DoubleSupplier headingX, DoubleSupplier headingY) {
    withHeading(() -> Radians.of(Math.atan2(headingX.getAsDouble(), headingY.getAsDouble())));
    controllerHeadingX = Optional.of(headingX);
    controllerHeadingY = Optional.of(headingY);
    return this;
  }

  public SwerveInputStream deadband(double deadband) {
    axisDeadband = deadband == 0 ? Optional.empty() : Optional.of(deadband);
    return this;
  }

  public SwerveInputStream withDeadband(double deadband) {
    return deadband(deadband);
  }

  public SwerveInputStream withScaleTranslation(double scaleTranslation) {
    translationAxisScale = scaleTranslation == 0 ? Optional.empty() : Optional.of(scaleTranslation);
    return this;
  }

  public SwerveInputStream withScaleRotation(double scaleRotation) {
    omegaAxisScale = scaleRotation == 0 ? Optional.empty() : Optional.of(scaleRotation);
    return this;
  }

  public SwerveInputStream withHeadingControl(BooleanSupplier trigger) {
    headingEnabled = Optional.of(trigger);
    return this;
  }

  public SwerveInputStream withHeading(Supplier<Angle> heading) {
    headingSupplier = Optional.ofNullable(heading);
    controllerHeadingX = Optional.empty();
    controllerHeadingY = Optional.empty();
    return this;
  }

  public SwerveInputStream withAim(Supplier<Pose2d> aimTarget, BooleanSupplier trigger) {
    this.aimTarget = aimTarget.equals(Pose2d.ZERO) ? Optional.empty() : Optional.of(aimTarget);
    aimEnabled = Optional.of(trigger);
    return this;
  }

  public SwerveInputStream withTranslationOnly(BooleanSupplier trigger) {
    translationOnlyEnabled = Optional.of(trigger);
    return this;
  }

  private SwerveInputMode findMode() {
    if (translationOnlyEnabled.isPresent() && translationOnlyEnabled.get().getAsBoolean()) {
      return SwerveInputMode.TRANSLATION_ONLY;
    } else if (aimEnabled.isPresent() && aimEnabled.get().getAsBoolean()) {
      if (aimTarget.isPresent()) {
        return SwerveInputMode.AIM;
      } else {
        DriverStationErrors.reportError(
            "Attempting to enter AIM mode without target, please use "
                + "SwerveInputStream.aim() to select a target first!",
            false);
      }
    } else if (headingEnabled.isPresent() && headingEnabled.get().getAsBoolean()) {
      if (headingSupplier.isPresent()) {
        return SwerveInputMode.HEADING;
      } else {
        DriverStationErrors.reportError(
            "Attempting to enter HEADING mode without a heading, please use "
                + "SwerveInputStream.withHeading or SwerveInputStream.withControllerHeadingAxis to add one!",
            false);
      }
    } else if (controllerOmega.isEmpty()) {
      DriverStationErrors.reportError(
          "Attempting to enter ANGULAR_VELOCITY mode without a rotation axis, please use "
              + "SwerveInputStream.withControllerRotationAxis to add angular velocity axis!",
          false);
      return SwerveInputMode.TRANSLATION_ONLY;
    }
    return SwerveInputMode.ANGULAR_VELOCITY;
  }

  private void transitionMode(SwerveInputMode newMode) {
    switch (currentMode) {
      case TRANSLATION_ONLY -> {
        lockedHeading = Optional.empty();
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

    switch (newMode) {
      case TRANSLATION_ONLY -> {
        lockedHeading = Optional.of(swerveDrive.getGyroRotation3d().toRotation2d());
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

  private double applyDeadband(double axisValue) {
    return axisDeadband.map(aDouble -> MathUtil.applyDeadband(axisValue, aDouble)).orElse(axisValue);
  }

  private double applyRotationalScalar(double axisValue) {
    return omegaAxisScale.map(aDouble -> axisValue * aDouble).orElse(axisValue);
  }

  private Translation2d applyTranslationScalar(double xAxis, double yAxis) {
    return translationAxisScale
        .map(aDouble -> SwerveDriveConfig.scaleTranslation(new Translation2d(xAxis, yAxis), aDouble))
        .orElseGet(() -> new Translation2d(xAxis, yAxis));
  }

  private Translation2d applyTranslationCube(Translation2d translation) {
    if (translationCube.isPresent() && translationCube.get().getAsBoolean()) {
      return SwerveDriveConfig.cubeTranslation(translation);
    }
    return translation;
  }

  private double applyOmegaCube(double rotationAxis) {
    if (omegaCube.isPresent() && omegaCube.get().getAsBoolean()) {
      return Math.pow(rotationAxis, 3);
    }
    return rotationAxis;
  }

  private ChassisVelocities applyRobotRelativeTranslation(ChassisVelocities fieldRelativeSpeeds) {
    if (robotRelative.isPresent() && robotRelative.get().getAsBoolean()) {
      return fieldRelativeSpeeds.toFieldRelative(swerveDrive.getGyroRotation3d().toRotation2d());
    }
    return fieldRelativeSpeeds;
  }

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

  private ChassisVelocities applyTranslationHeadingOffset(ChassisVelocities speeds) {
    if (translationHeadingOffsetEnabled.isPresent() && translationHeadingOffsetEnabled.get().getAsBoolean()) {
      if (translationHeadingOffset.isPresent()) {
        Translation2d speedsTranslation = new Translation2d(speeds.vx, speeds.vy)
            .rotateBy(translationHeadingOffset.get());
        return new ChassisVelocities(speedsTranslation.getX(), speedsTranslation.getY(), speeds.omega);
      }
    }
    return speeds;
  }

  private static PIDController requireRotationPID(SwerveDriveConfig<?> config) {
    return config.getRotationPID()
        .orElseThrow(() -> new SwerveDriveConfigurationException(
            "No rotation PID controller configured",
            "Heading, aim, and translation only control are unavailable",
            "Use SwerveDriveConfig.withRotationController to configure a rotation PID controller"));
  }

  @Override
  public ChassisVelocities get() {
    var config = swerveDrive.getConfig();
    double maximumChassisVelocity =
        config.getMaximumChassisLinearVelocity().orElse(maximumChassisLinearVelocity).in(MetersPerSecond);
    double maximumChassisRotVelocity =
        config.getMaximumChassisAngularVelocity().orElse(maximumChassisAngularVelocity).in(RadiansPerSecond);
    Translation2d scaledTranslation =
        applyTranslationScalar(applyDeadband(controllerTranslationX.getAsDouble()), applyDeadband(controllerTranslationY.getAsDouble()));
    scaledTranslation = applyTranslationCube(scaledTranslation);
    scaledTranslation = applyAllianceAwareTranslation(scaledTranslation);

    double vxMetersPerSecond = scaledTranslation.getX() * maximumChassisVelocity;
    double vyMetersPerSecond = scaledTranslation.getY() * maximumChassisVelocity;
    double omegaRadiansPerSecond = 0;
    ChassisVelocities speeds = new ChassisVelocities();

    SwerveInputMode newMode = findMode();
    if (currentMode != newMode) {
      transitionMode(newMode);
    }
    switch (newMode) {
      case TRANSLATION_ONLY -> {
        var azimuthPIDs = requireRotationPID(config);
        omegaRadiansPerSecond =
            azimuthPIDs.calculate(
                swerveDrive.getGyroRotation3d().getZ(), lockedHeading.orElseThrow().getRadians());
        speeds = new ChassisVelocities(vxMetersPerSecond, vyMetersPerSecond, omegaRadiansPerSecond);
        break;
      }
      case ANGULAR_VELOCITY -> {
        omegaRadiansPerSecond =
            applyOmegaCube(applyRotationalScalar(applyDeadband(controllerOmega.orElseThrow().getAsDouble())))
                * maximumChassisRotVelocity;
        speeds = new ChassisVelocities(vxMetersPerSecond, vyMetersPerSecond, omegaRadiansPerSecond);
        break;
      }
      case HEADING -> {
        var azimuthPIDs = requireRotationPID(config);
        omegaRadiansPerSecond = azimuthPIDs.calculate(
            swerveDrive.getGyroRotation3d().getZ(), headingSupplier.orElseThrow().get().in(Radians));

        if (controllerHeadingX.isPresent()
            && controllerHeadingY.isPresent()
            && axisDeadband.isPresent()
            && Math.abs(controllerHeadingX.get().getAsDouble())
                    + Math.abs(controllerHeadingY.get().getAsDouble())
                < axisDeadband.get()) {
          omegaRadiansPerSecond = 0;
        }
        speeds = new ChassisVelocities(vxMetersPerSecond, vyMetersPerSecond, omegaRadiansPerSecond);
        break;
      }
      case AIM -> {
        var azimuthPIDs = requireRotationPID(config);
        Rotation2d currentHeading = swerveDrive.getGyroRotation3d().toRotation2d();
        Translation2d relativeTrl =
            aimTarget.orElseThrow().get().relativeTo(swerveDrive.getPose()).getTranslation();
        Rotation2d target = new Rotation2d(relativeTrl.getX(), relativeTrl.getY()).plus(currentHeading);
        omegaRadiansPerSecond = azimuthPIDs.calculate(currentHeading.getRadians(), target.getRadians());
        speeds = new ChassisVelocities(vxMetersPerSecond, vyMetersPerSecond, omegaRadiansPerSecond);
        break;
      }
    }

    currentMode = newMode;
    return applyTranslationHeadingOffset(applyRobotRelativeTranslation(speeds));
  }

  enum SwerveInputMode {
    TRANSLATION_ONLY,
    ANGULAR_VELOCITY,
    HEADING,
    AIM
  }
}