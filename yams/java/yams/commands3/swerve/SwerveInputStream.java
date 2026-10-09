// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package yams.commands3.swerve;

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
import yams.commands3.telemetry.SwerveInputStreamTelemetry;
import yams.core.exceptions.SwerveDriveConfigurationException;
import yams.core.mechanisms.config.SwerveDriveConfig;
import yams.core.mechanisms.swerve.SwerveDrive;
import yams.core.telemetry.enums.TelemetryVerbosity;

/**
 * Helper class to transform controller inputs into workable Chassis speeds for the Commands v3 API.
 */
public class SwerveInputStream implements Supplier<ChassisVelocities> {
  private final DoubleSupplier controllerTranslationX;
  private final DoubleSupplier controllerTranslationY;
  private final SwerveDrive swerveDrive;
  private Optional<DoubleSupplier> controllerOmega = Optional.empty();
  private Optional<DoubleSupplier> controllerHeadingX = Optional.empty();
  private Optional<DoubleSupplier> controllerHeadingY = Optional.empty();
  private Optional<Supplier<Angle>> headingSupplier = Optional.empty();
  private Optional<Double> axisDeadband = Optional.empty();
  private Optional<Double> translationAxisScale = Optional.empty();
  private Optional<Double> omegaAxisScale = Optional.empty();
  private Optional<Supplier<Pose2d>> aimTarget = Optional.empty();
  private Optional<BooleanSupplier> headingEnabled = Optional.empty();
  private Optional<Rotation2d> lockedHeading = Optional.empty();
  private Optional<BooleanSupplier> aimEnabled = Optional.empty();
  private Optional<BooleanSupplier> translationOnlyEnabled = Optional.empty();
  private Optional<BooleanSupplier> translationCube = Optional.empty();
  private Optional<BooleanSupplier> omegaCube = Optional.empty();
  private Optional<BooleanSupplier> robotRelative = Optional.empty();
  private Optional<BooleanSupplier> allianceRelative = Optional.empty();
  private Optional<BooleanSupplier> translationHeadingOffsetEnabled = Optional.empty();
  private Optional<Rotation2d> translationHeadingOffset = Optional.empty();
  private SwerveInputMode currentMode = SwerveInputMode.ANGULAR_VELOCITY;
  private LinearVelocity maximumChassisLinearVelocity = MetersPerSecond.of(4);
  private AngularVelocity maximumChassisAngularVelocity = RotationsPerSecond.of(1);
  private Optional<SwerveInputStreamTelemetry> telemetry = Optional.empty();

  private SwerveInputStream(SwerveDrive drive, DoubleSupplier x, DoubleSupplier y) {
    controllerTranslationX = x;
    controllerTranslationY = y;
    swerveDrive = drive;
    // Start from the drive's maximum chassis speeds; withMaximum*Velocity and live tuning override them.
    drive.getConfig().getMaximumChassisLinearVelocity().ifPresent(velocity -> maximumChassisLinearVelocity = velocity);
    drive.getConfig().getMaximumChassisAngularVelocity().ifPresent(velocity -> maximumChassisAngularVelocity = velocity);
  }

  /**
   * Create a {@link SwerveInputStream} that rotates the robot with an angular velocity axis.
   *
   * @param drive {@link SwerveDrive} the stream drives.
   * @param x     Field relative X (forward) translation axis, in [-1, 1].
   * @param y     Field relative Y (left) translation axis, in [-1, 1].
   * @param rot   Angular velocity axis, counter-clockwise positive, in [-1, 1].
   */
  public SwerveInputStream(SwerveDrive drive, DoubleSupplier x, DoubleSupplier y, DoubleSupplier rot) {
    this(drive, x, y);
    controllerOmega = Optional.of(rot);
  }

  /**
   * Create a {@link SwerveInputStream} that turns the robot to face the direction a controller stick points.
   * Heading control is only used while {@link #withHeadingControl(BooleanSupplier)} is true.
   *
   * @param drive    {@link SwerveDrive} the stream drives.
   * @param x        Field relative X (forward) translation axis, in [-1, 1].
   * @param y        Field relative Y (left) translation axis, in [-1, 1].
   * @param headingX Heading stick X axis, in [-1, 1].
   * @param headingY Heading stick Y axis, in [-1, 1].
   */
  public SwerveInputStream(
      SwerveDrive drive,
      DoubleSupplier x,
      DoubleSupplier y,
      DoubleSupplier headingX,
      DoubleSupplier headingY) {
    this(drive, x, y);
    withControllerHeadingAxis(headingX, headingY);
  }

  /**
   * Create a {@link SwerveInputStream} with only translation axes. Add rotation with
   * {@link #withControllerRotationAxis(DoubleSupplier)} or
   * {@link #withControllerHeadingAxis(DoubleSupplier, DoubleSupplier)}.
   *
   * @param drive {@link SwerveDrive} the stream drives.
   * @param x     Field relative X (forward) translation axis, in [-1, 1].
   * @param y     Field relative Y (left) translation axis, in [-1, 1].
   * @return A new {@link SwerveInputStream}.
   */
  public static SwerveInputStream of(SwerveDrive drive, DoubleSupplier x, DoubleSupplier y) {
    return new SwerveInputStream(drive, x, y);
  }

  /**
   * Create a {@link SwerveInputStream} that rotates the robot with an angular velocity axis.
   *
   * @param drive {@link SwerveDrive} the stream drives.
   * @param x     Field relative X (forward) translation axis, in [-1, 1].
   * @param y     Field relative Y (left) translation axis, in [-1, 1].
   * @param rot   Angular velocity axis, counter-clockwise positive, in [-1, 1].
   * @return A new {@link SwerveInputStream}.
   */
  public static SwerveInputStream of(SwerveDrive drive, DoubleSupplier x, DoubleSupplier y, DoubleSupplier rot) {
    return new SwerveInputStream(drive, x, y, rot);
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
    // Telemetry is bound to the stream it was created for, so a clone starts without any.
  }

  @Override
  public SwerveInputStream clone() {
    return new SwerveInputStream(this);
  }

  /**
   * Get the name of the drive mode used by the last {@link #get()} call.
   *
   * @return Mode name, e.g. {@code ANGULAR_VELOCITY}, {@code HEADING}, {@code AIM} or {@code TRANSLATION_ONLY}.
   */
  public String getCurrentModeName() {
    return currentMode.name();
  }

  /**
   * Get the controller axis deadband.
   *
   * @return Axis deadband, 0 when none is applied.
   */
  public double getAxisDeadband() {
    return axisDeadband.orElse(0.0);
  }

  /**
   * Set the controller axis deadband.
   *
   * @param deadband Deadband applied to every axis, 0 to disable.
   */
  public void setAxisDeadband(double deadband) {
    axisDeadband = deadband == 0 ? Optional.empty() : Optional.of(deadband);
  }

  /**
   * Get the translation axis scalar.
   *
   * @return Translation axis scalar, 1 when unscaled.
   */
  public double getTranslationAxisScale() {
    return translationAxisScale.orElse(1.0);
  }

  /**
   * Set the translation axis scalar.
   *
   * @param scale Scalar in (0, 1] applied to the translation axes, 0 to disable.
   */
  public void setTranslationAxisScale(double scale) {
    translationAxisScale = scale == 0 ? Optional.empty() : Optional.of(scale);
  }

  /**
   * Get the angular velocity axis scalar.
   *
   * @return Angular velocity axis scalar, 1 when unscaled.
   */
  public double getOmegaAxisScale() {
    return omegaAxisScale.orElse(1.0);
  }

  /**
   * Set the angular velocity axis scalar.
   *
   * @param scale Scalar in (0, 1] applied to the angular velocity axis, 0 to disable.
   */
  public void setOmegaAxisScale(double scale) {
    omegaAxisScale = scale == 0 ? Optional.empty() : Optional.of(scale);
  }

  /**
   * Get the chassis linear velocity a full translation axis maps to.
   *
   * @return Maximum chassis linear velocity.
   */
  public LinearVelocity getMaximumChassisLinearVelocity() {
    return maximumChassisLinearVelocity;
  }

  /**
   * Set the chassis linear velocity a full translation axis maps to.
   *
   * @param velocity Maximum chassis linear velocity.
   */
  public void setMaximumChassisLinearVelocity(LinearVelocity velocity) {
    maximumChassisLinearVelocity = velocity;
  }

  /**
   * Get the chassis angular velocity a full angular velocity axis maps to.
   *
   * @return Maximum chassis angular velocity.
   */
  public AngularVelocity getMaximumChassisAngularVelocity() {
    return maximumChassisAngularVelocity;
  }

  /**
   * Set the chassis angular velocity a full angular velocity axis maps to.
   *
   * @param velocity Maximum chassis angular velocity.
   */
  public void setMaximumChassisAngularVelocity(AngularVelocity velocity) {
    maximumChassisAngularVelocity = velocity;
  }

  /**
   * Check if the translation magnitude is cubed, for finer control near the center of the stick.
   *
   * @return True if translation cubing is enabled.
   */
  public boolean isTranslationCubeEnabled() {
    return translationCube.isPresent() && translationCube.get().getAsBoolean();
  }

  /**
   * Enable or disable cubing the translation magnitude.
   *
   * @param enabled True to cube the translation magnitude.
   */
  public void setTranslationCubeEnabled(boolean enabled) {
    translationCube = enabled ? Optional.of(() -> true) : Optional.empty();
  }

  /**
   * Check if the angular velocity axis is cubed, for finer control near the center of the stick.
   *
   * @return True if angular velocity cubing is enabled.
   */
  public boolean isOmegaCubeEnabled() {
    return omegaCube.isPresent() && omegaCube.get().getAsBoolean();
  }

  /**
   * Enable or disable cubing the angular velocity axis.
   *
   * @param enabled True to cube the angular velocity axis.
   */
  public void setOmegaCubeEnabled(boolean enabled) {
    omegaCube = enabled ? Optional.of(() -> true) : Optional.empty();
  }

  /**
   * Check if translation is flipped 180 degrees while on the red alliance.
   *
   * @return True if alliance relative control is enabled.
   */
  public boolean isAllianceRelativeEnabled() {
    return allianceRelative.isPresent() && allianceRelative.get().getAsBoolean();
  }

  /**
   * Enable or disable flipping translation 180 degrees while on the red alliance.
   *
   * @param enabled True to enable alliance relative control.
   */
  public void setAllianceRelativeEnabled(boolean enabled) {
    allianceRelative = enabled ? Optional.of(() -> true) : Optional.empty();
  }

  /**
   * Check if the translation axes are treated as robot relative.
   *
   * @return True if robot relative control is enabled.
   */
  public boolean isRobotRelativeEnabled() {
    return robotRelative.isPresent() && robotRelative.get().getAsBoolean();
  }

  /**
   * Enable or disable treating the translation axes as robot relative.
   *
   * @param enabled True to enable robot relative control.
   */
  public void setRobotRelativeEnabled(boolean enabled) {
    robotRelative = enabled ? Optional.of(() -> true) : Optional.empty();
  }

  /**
   * Publish this stream's telemetry under {@code SwerveInputStream/<name>}, updated every time the stream is read. At
   * {@link TelemetryVerbosity#HIGH} the stream is also live tunable: see {@link SwerveInputStreamTelemetry}. Replaces
   * any telemetry this stream already had. A {@link #clone()} does not copy the telemetry.
   *
   * @param name      Name of the stream in NetworkTables (e.g., "drive").
   * @param verbosity {@link TelemetryVerbosity} to publish at.
   * @return this, for chaining.
   */
  public SwerveInputStream withTelemetry(String name, TelemetryVerbosity verbosity) {
    telemetry.ifPresent(SwerveInputStreamTelemetry::close);
    telemetry = Optional.of(new SwerveInputStreamTelemetry(this, name, verbosity));
    return this;
  }

  /**
   * Get this stream's telemetry, created by {@link #withTelemetry(String, TelemetryVerbosity)}.
   *
   * @return The {@link SwerveInputStreamTelemetry}, or empty if telemetry is not enabled.
   */
  public Optional<SwerveInputStreamTelemetry> getTelemetry() {
    return telemetry;
  }

  /**
   * Set the chassis linear velocity a full translation axis maps to. Defaults to the drive's maximum chassis linear
   * velocity.
   *
   * @param velocity Maximum chassis linear velocity.
   * @return this, for chaining.
   */
  public SwerveInputStream withMaximumLinearVelocity(LinearVelocity velocity) {
    maximumChassisLinearVelocity = velocity;
    return this;
  }

  /**
   * Set the chassis angular velocity a full angular velocity axis maps to. Defaults to the drive's maximum chassis
   * angular velocity.
   *
   * @param velocity Maximum chassis angular velocity.
   * @return this, for chaining.
   */
  public SwerveInputStream withMaximumAngularVelocity(AngularVelocity velocity) {
    maximumChassisAngularVelocity = velocity;
    return this;
  }

  /**
   * Treat the translation axes as robot relative while enabled. The output is still field relative, and alliance
   * relative flipping is skipped.
   *
   * @param enabled Robot relative control is used while this is true.
   * @return this, for chaining.
   */
  public SwerveInputStream withRobotRelative(BooleanSupplier enabled) {
    robotRelative = Optional.of(enabled);
    return this;
  }

  /**
   * Always treat the translation axes as robot relative.
   *
   * @return this, for chaining.
   */
  public SwerveInputStream withRobotRelative() {
    return withRobotRelative(() -> true);
  }

  /**
   * Rotate the output translation by an offset while enabled.
   *
   * @param angle   Offset to rotate the translation by.
   * @param enabled The offset is applied while this is true.
   * @return this, for chaining.
   */
  public SwerveInputStream withTranslationHeadingOffset(Rotation2d angle, BooleanSupplier enabled) {
    translationHeadingOffset = Optional.of(angle);
    translationHeadingOffsetEnabled = Optional.of(enabled);
    return this;
  }

  /**
   * Always rotate the output translation by an offset.
   *
   * @param angle Offset to rotate the translation by.
   * @return this, for chaining.
   */
  public SwerveInputStream withTranslationHeadingOffset(Rotation2d angle) {
    return withTranslationHeadingOffset(angle, () -> true);
  }

  /**
   * Always flip translation 180 degrees while on the red alliance, so forward is away from the driver station.
   *
   * @return this, for chaining.
   */
  public SwerveInputStream withAllianceRelativeControl() {
    return withAllianceRelativeControl(() -> true);
  }

  /**
   * Flip translation 180 degrees while on the red alliance, so forward is away from the driver station.
   *
   * @param enabled Alliance relative control is used while this is true.
   * @return this, for chaining.
   */
  public SwerveInputStream withAllianceRelativeControl(BooleanSupplier enabled) {
    allianceRelative = Optional.ofNullable(enabled);
    return this;
  }

  /**
   * Cube the angular velocity axis while enabled, for finer control near the center of the stick.
   *
   * @param enabled The axis is cubed while this is true.
   * @return this, for chaining.
   */
  public SwerveInputStream withCubeRotationControllerAxis(BooleanSupplier enabled) {
    omegaCube = Optional.of(enabled);
    return this;
  }

  /**
   * Always cube the angular velocity axis, for finer control near the center of the stick.
   *
   * @return this, for chaining.
   */
  public SwerveInputStream withCubeRotationControllerAxis() {
    return withCubeRotationControllerAxis(() -> true);
  }

  /**
   * Cube the translation magnitude while enabled, for finer control near the center of the stick.
   *
   * @param enabled The magnitude is cubed while this is true.
   * @return this, for chaining.
   */
  public SwerveInputStream withCubeTranslationControllerAxis(BooleanSupplier enabled) {
    translationCube = Optional.of(enabled);
    return this;
  }

  /**
   * Always cube the translation magnitude, for finer control near the center of the stick.
   *
   * @return this, for chaining.
   */
  public SwerveInputStream withCubeTranslationControllerAxis() {
    return withCubeTranslationControllerAxis(() -> true);
  }

  /**
   * Set the axis used for angular velocity control.
   *
   * @param rot Angular velocity axis, counter-clockwise positive, in [-1, 1].
   * @return this, for chaining.
   */
  public SwerveInputStream withControllerRotationAxis(DoubleSupplier rot) {
    controllerOmega = Optional.of(rot);
    return this;
  }

  /**
   * Face the direction a controller stick points while heading control is enabled. The robot stops turning while
   * the stick is inside the deadband. Replaces any heading from {@link #withHeading(Supplier)}.
   *
   * @param headingX Heading stick X axis, in [-1, 1].
   * @param headingY Heading stick Y axis, in [-1, 1].
   * @return this, for chaining.
   */
  public SwerveInputStream withControllerHeadingAxis(DoubleSupplier headingX, DoubleSupplier headingY) {
    withHeading(() -> Radians.of(Math.atan2(headingX.getAsDouble(), headingY.getAsDouble())));
    controllerHeadingX = Optional.of(headingX);
    controllerHeadingY = Optional.of(headingY);
    return this;
  }

  /**
   * Set the deadband applied to every controller axis.
   *
   * @param deadband Axis deadband, 0 to disable.
   * @return this, for chaining.
   */
  public SwerveInputStream deadband(double deadband) {
    axisDeadband = deadband == 0 ? Optional.empty() : Optional.of(deadband);
    return this;
  }

  /**
   * Set the deadband applied to every controller axis.
   *
   * @param deadband Axis deadband, 0 to disable.
   * @return this, for chaining.
   */
  public SwerveInputStream withDeadband(double deadband) {
    return deadband(deadband);
  }

  /**
   * Scale the translation axes.
   *
   * @param scaleTranslation Scalar in (0, 1], 0 to disable.
   * @return this, for chaining.
   */
  public SwerveInputStream withScaleTranslation(double scaleTranslation) {
    translationAxisScale = scaleTranslation == 0 ? Optional.empty() : Optional.of(scaleTranslation);
    return this;
  }

  /**
   * Scale the angular velocity axis.
   *
   * @param scaleRotation Scalar in (0, 1], 0 to disable.
   * @return this, for chaining.
   */
  public SwerveInputStream withScaleRotation(double scaleRotation) {
    omegaAxisScale = scaleRotation == 0 ? Optional.empty() : Optional.of(scaleRotation);
    return this;
  }

  /**
   * Face the heading from {@link #withHeading(Supplier)} or
   * {@link #withControllerHeadingAxis(DoubleSupplier, DoubleSupplier)} while enabled. Requires a rotation controller
   * on the {@link SwerveDrive}.
   *
   * @param trigger Heading control is used while this is true.
   * @return this, for chaining.
   */
  public SwerveInputStream withHeadingControl(BooleanSupplier trigger) {
    headingEnabled = Optional.of(trigger);
    return this;
  }

  /**
   * Set the field relative heading to face while heading control is enabled. Replaces any controller heading axis.
   *
   * @param heading Field relative heading supplier.
   * @return this, for chaining.
   */
  public SwerveInputStream withHeading(Supplier<Angle> heading) {
    headingSupplier = Optional.ofNullable(heading);
    controllerHeadingX = Optional.empty();
    controllerHeadingY = Optional.empty();
    return this;
  }

  /**
   * Face a target pose while enabled. Requires a rotation controller on the {@link SwerveDrive}.
   *
   * @param aimTarget Field relative pose to aim at, null for no target.
   * @param trigger   Aiming is used while this is true.
   * @return this, for chaining.
   */
  public SwerveInputStream withAim(Supplier<Pose2d> aimTarget, BooleanSupplier trigger) {
    this.aimTarget = Optional.ofNullable(aimTarget);
    aimEnabled = Optional.of(trigger);
    return this;
  }

  /**
   * Hold the heading the robot had when enabled and only translate. Requires a rotation controller on the
   * {@link SwerveDrive}.
   *
   * @param trigger Translation only control is used while this is true.
   * @return this, for chaining.
   */
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
                + "SwerveInputStream.withAim to select a target first!",
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
    double maximumChassisVelocity = maximumChassisLinearVelocity.in(MetersPerSecond);
    double maximumChassisRotVelocity = maximumChassisAngularVelocity.in(RadiansPerSecond);
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
    ChassisVelocities fieldRelativeSpeeds = applyTranslationHeadingOffset(applyRobotRelativeTranslation(speeds));
    telemetry.ifPresent(SwerveInputStreamTelemetry::updateTelemetry);
    return fieldRelativeSpeeds;
  }

  enum SwerveInputMode {
    TRANSLATION_ONLY,
    ANGULAR_VELOCITY,
    HEADING,
    AIM
  }
}