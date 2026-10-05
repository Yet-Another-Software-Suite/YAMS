// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package yams.commands3.telemetry;

import static org.wpilib.units.Units.MetersPerSecond;
import static org.wpilib.units.Units.RadiansPerSecond;

import java.util.Objects;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.networktables.BooleanEntry;
import org.wpilib.networktables.DoubleEntry;
import org.wpilib.networktables.NetworkTable;
import org.wpilib.networktables.NetworkTableInstance;
import yams.commands3.swerve.SwerveInputStream;

/**
 * Telemetry and live tuning support for {@link SwerveInputStream}.
 *
 * <p>Publishes the current state and configuration of a SwerveInputStream to NetworkTables for
 * monitoring on the dashboard. Also enables real-time tuning of deadband, scale, and maximum
 * velocity parameters via NetworkTables entries.
 *
 * <p>Usage:
 *
 * <pre>{@code
 * SwerveInputStream driveStream = SwerveInputStream.of(drive, leftY, leftX)
 *     .withControllerRotationAxis(rightX)
 *     .withDeadband(0.05);
 *
 * SwerveInputStreamTelemetry telemetry = new SwerveInputStreamTelemetry(driveStream, "drive");\n * telemetry.update(); // Call in robot periodic or your command's execute()
 * }</pre>
 */
public class SwerveInputStreamTelemetry {
  private final SwerveInputStream stream;
  private final NetworkTable table;

  // Tuning entries
  private final DoubleEntry deadbandEntry;
  private final DoubleEntry translationScaleEntry;
  private final DoubleEntry rotationScaleEntry;
  private final DoubleEntry maxLinearVelocityEntry;
  private final DoubleEntry maxAngularVelocityEntry;
  private final BooleanEntry translationCubeEntry;
  private final BooleanEntry rotationCubeEntry;
  private final BooleanEntry allianceRelativeEntry;
  private final BooleanEntry robotRelativeEntry;

  /**
   * Create telemetry for a SwerveInputStream.
   *
   * @param stream The SwerveInputStream to monitor and tune.
   * @param name   The NetworkTables subtable name (e.g., "drive").
   */
  public SwerveInputStreamTelemetry(SwerveInputStream stream, String name) {
    this.stream = Objects.requireNonNull(stream, "stream cannot be null");
    this.table = NetworkTableInstance.getDefault().getTable("SwerveInputStream").getSubTable(name);

    // Initialize tuning entries
    this.deadbandEntry = table.getDoubleTopic("deadband")
        .getEntry(stream.getAxisDeadband());
    this.translationScaleEntry = table.getDoubleTopic("translationScale")
        .getEntry(stream.getTranslationAxisScale());
    this.rotationScaleEntry = table.getDoubleTopic("rotationScale")
        .getEntry(stream.getOmegaAxisScale());
    this.maxLinearVelocityEntry = table.getDoubleTopic("maxLinearVelocity")
        .getEntry(stream.getMaximumChassisLinearVelocity().in(MetersPerSecond));
    this.maxAngularVelocityEntry = table.getDoubleTopic("maxAngularVelocity")
        .getEntry(stream.getMaximumChassisAngularVelocity().in(RadiansPerSecond));
    this.translationCubeEntry = table.getBooleanTopic("translationCube")
        .getEntry(stream.isTranslationCubeEnabled());
    this.rotationCubeEntry = table.getBooleanTopic("rotationCube")
        .getEntry(stream.isOmegaCubeEnabled());
    this.allianceRelativeEntry = table.getBooleanTopic("allianceRelative")
        .getEntry(stream.isAllianceRelativeEnabled());
    this.robotRelativeEntry = table.getBooleanTopic("robotRelative")
        .getEntry(stream.isRobotRelativeEnabled());
  }

  /**
   * Update telemetry and apply any live-tuned values.
   *
   * <p>Call this method once per robot loop (e.g., in periodic() or in your command's execute())
   * to publish the current state and check for tuning changes.
   */
  public void update() {
    updateMode();
    updateDeadband();
    updateTranslationScale();
    updateRotationScale();
    updateMaxLinearVelocity();
    updateMaxAngularVelocity();
    updateTranslationCube();
    updateRotationCube();
    updateAllianceRelative();
    updateRobotRelative();
  }

  /**
   * Update the current drive mode.
   */
  private void updateMode() {
    table.getStringTopic("mode").getEntry("UNKNOWN").set(stream.getCurrentModeName());
  }

  /**
   * Update deadband from dashboard and apply to stream if changed.
   */
  private void updateDeadband() {
    deadbandEntry.set(stream.getAxisDeadband());
    double tuned = deadbandEntry.get();
    if (tuned != stream.getAxisDeadband()) {
      stream.setAxisDeadband(tuned);
    }
  }

  /**
   * Update translation scale from dashboard and apply to stream if changed.
   */
  private void updateTranslationScale() {
    translationScaleEntry.set(stream.getTranslationAxisScale());
    double tuned = translationScaleEntry.get();
    if (tuned != stream.getTranslationAxisScale() && tuned > 0.0 && tuned <= 1.0) {
      stream.setTranslationAxisScale(tuned);
    }
  }

  /**
   * Update rotation scale from dashboard and apply to stream if changed.
   */
  private void updateRotationScale() {
    rotationScaleEntry.set(stream.getOmegaAxisScale());
    double tuned = rotationScaleEntry.get();
    if (tuned != stream.getOmegaAxisScale() && tuned > 0.0 && tuned <= 1.0) {
      stream.setOmegaAxisScale(tuned);
    }
  }

  /**
   * Update maximum linear velocity from dashboard and apply to stream if changed.
   */
  private void updateMaxLinearVelocity() {
    maxLinearVelocityEntry.set(stream.getMaximumChassisLinearVelocity().in(MetersPerSecond));
    double tuned = maxLinearVelocityEntry.get();
    if (tuned != stream.getMaximumChassisLinearVelocity().in(MetersPerSecond) && tuned > 0.0) {
      stream.setMaximumChassisLinearVelocity(MetersPerSecond.of(tuned));
    }
  }

  /**
   * Update maximum angular velocity from dashboard and apply to stream if changed.
   */
  private void updateMaxAngularVelocity() {
    maxAngularVelocityEntry.set(stream.getMaximumChassisAngularVelocity().in(RadiansPerSecond));
    double tuned = maxAngularVelocityEntry.get();
    if (tuned != stream.getMaximumChassisAngularVelocity().in(RadiansPerSecond) && tuned > 0.0) {
      stream.setMaximumChassisAngularVelocity(RadiansPerSecond.of(tuned));
    }
  }

  /**
   * Update translation cube response from dashboard and apply to stream if changed.
   */
  private void updateTranslationCube() {
    translationCubeEntry.set(stream.isTranslationCubeEnabled());
    boolean tuned = translationCubeEntry.get();
    if (tuned != stream.isTranslationCubeEnabled()) {
      stream.setTranslationCubeEnabled(tuned);
    }
  }

  /**
   * Update rotation cube response from dashboard and apply to stream if changed.
   */
  private void updateRotationCube() {
    rotationCubeEntry.set(stream.isOmegaCubeEnabled());
    boolean tuned = rotationCubeEntry.get();
    if (tuned != stream.isOmegaCubeEnabled()) {
      stream.setOmegaCubeEnabled(tuned);
    }
  }

  /**
   * Update alliance-relative control from dashboard and apply to stream if changed.
   */
  private void updateAllianceRelative() {
    allianceRelativeEntry.set(stream.isAllianceRelativeEnabled());
    boolean tuned = allianceRelativeEntry.get();
    if (tuned != stream.isAllianceRelativeEnabled()) {
      stream.setAllianceRelativeEnabled(tuned);
    }
  }

  /**
   * Update robot-relative control from dashboard and apply to stream if changed.
   */
  private void updateRobotRelative() {
    robotRelativeEntry.set(stream.isRobotRelativeEnabled());
    boolean tuned = robotRelativeEntry.get();
    if (tuned != stream.isRobotRelativeEnabled()) {
      stream.setRobotRelativeEnabled(tuned);
    }
  }
}
