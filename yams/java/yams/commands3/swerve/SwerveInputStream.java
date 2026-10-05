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
import yams.core.exceptions.SwerveDriveConfigurationException;
import yams.core.mechanisms.config.SwerveDriveConfig;
import yams.core.mechanisms.swerve.SwerveDrive;

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

  public void setTranslationAxisScale(double scale) {
    translationAxisScale = scale == 0 ? Optional.empty() : Optional.of(scale);
  }

  public double getOmegaAxisScale() {
    return omegaAxisScale.orElse(1.0);
  }

  public void setOmegaAxisScale(double scale) {
    omegaAxisScale = scale == 0 ? Optional.empty() : Optional.of(scale);
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